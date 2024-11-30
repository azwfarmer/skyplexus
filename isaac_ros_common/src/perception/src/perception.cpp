#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <time.h>
#include "inttypes.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/console/parse.h>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include "std_msgs/msg/header.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

class Perception : public rclcpp::Node
{
  public:
    Perception() : Node("perception")
    {
      intrinsic_matrix_ = Eigen::Matrix3d();
      intrinsic_matrix_ << 1379.886229, 0, 997.763474,
                       0, 1387.794387, 561.948650,
                       0, 0, 1;

      world2camera_ = Eigen::Matrix<double,3,4>();
      world2camera_ << 0, -1, 0, 0,
                      0, 0, -1, 0,
                      1, 0, 0, 0;

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/color/image_raw", 10, std::bind(&Perception::cam_callback, this, _1));
      
      radar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "altos/altosRadar", 10, std::bind(&Perception::radar_callback, this, _1));

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

      lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "ouster/points", qos, std::bind(&Perception::lidar_callback, this, _1));

      overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);

      latest_radar_ = sensor_msgs::msg::PointCloud2();
      latest_lidar_ = sensor_msgs::msg::PointCloud2();
      radar_exists_ = false;
      lidar_exists_ = false;
    }

  private:
    void cam_callback(const sensor_msgs::msg::Image & msg)
    {
      Eigen::Affine3d t_tocamera_frombase_link;
      Eigen::Affine3d t_tobase_link_fromos_lidar;
      Eigen::Affine3d t_tobase_link_fromaltosRadar;

      if(radar_exists_ && lidar_exists_)
      {
        try {
          t_tocamera_frombase_link = tf2::transformToEigen(tf_buffer_->lookupTransform("camera_color_optical_frame", "base_link", msg.header.stamp, 1000ms));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "base_link->camera_color_optical_frame failed: %s", ex.what());
          return;
        }

        try {
          t_tobase_link_fromos_lidar = tf2::transformToEigen(tf_buffer_->lookupTransform("base_link", "os_imu", latest_lidar_.header.stamp, 1000ms));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "os_lidar->base_link failed: %s", ex.what());
          return;
        }

        try {
          t_tobase_link_fromaltosRadar = tf2::transformToEigen(tf_buffer_->lookupTransform("base_link","altosRadar", latest_radar_.header.stamp, 1000ms));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "altosRadar->base_link failed: %s", ex.what());
          return;
        }

        // Eigen::MatrixXd radar_projection_matrix = (intrinsic_matrix_ * world2camera_ * radar_extrinsic).matrix();
        // radar_projection_matrix.conservativeResize(3,7);
        // radar_projection_matrix(Eigen::all, Eigen::seq(3,Eigen::last)).setZero();

        // Eigen::MatrixXd lidar_projection_matrix = (intrinsic_matrix_ * world2camera_ * lidar_extrinsic).matrix();
        // lidar_projection_matrix.conservativeResize(3,10);
        // lidar_projection_matrix(Eigen::all, Eigen::seq(3,Eigen::last)).setZero();

        Eigen::MatrixXd radar_matrix = radarPointCloud2ToEigen(latest_radar_);
        Eigen::MatrixXd lidar_matrix = lidarPointCloud2ToEigen(latest_lidar_);

        int numLidarPts = static_cast<int>(lidar_matrix.cols());
        int numRadarPts = static_cast<int>(radar_matrix.cols());

        if(numLidarPts + numRadarPts > 1)
        {
          Eigen::MatrixXd radar_extrinsic = (t_tocamera_frombase_link * t_tobase_link_fromaltosRadar).matrix();
          Eigen::MatrixXd lidar_extrinsic = (t_tocamera_frombase_link * t_tobase_link_fromos_lidar).matrix();

          Eigen::MatrixXd radar_matrix_camera_frame = world2camera_ * radar_extrinsic * radar_matrix(Eigen::seq(0,3),Eigen::all);
          Eigen::MatrixXd lidar_matrix_camera_frame = world2camera_ * lidar_extrinsic * lidar_matrix(Eigen::seq(0,3),Eigen::all);

          Eigen::MatrixXd transformed_lidar_matrix = Eigen::MatrixXd(4,numLidarPts);
          transformed_lidar_matrix.topLeftCorner(3, numLidarPts) = intrinsic_matrix_ * lidar_matrix_camera_frame(Eigen::seq(0,2),Eigen::all);
          // transformed_lidar_matrix(3,Eigen::all) = lidar_matrix(9, Eigen::all);
          // RCLCPP_INFO_STREAM(this->get_logger(), "LIDAR MATRIX\n" << transformed_lidar_matrix(3,Eigen::all) << "\n");
          // transformed_lidar_matrix.bottomLeftCorner(6, numLidarPts) = lidar_matrix(Eigen::seq(4,Eigen::last),Eigen::all);

          Eigen::MatrixXd transformed_radar_matrix = Eigen::MatrixXd(7, numRadarPts);
          transformed_radar_matrix.topLeftCorner(3, numRadarPts) = intrinsic_matrix_ * radar_matrix_camera_frame(Eigen::seq(0,2),Eigen::all);
          transformed_radar_matrix(Eigen::seq(3,Eigen::last-1), Eigen::all) = radar_matrix(Eigen::seq(4,Eigen::last),Eigen::all);

          for(int i=0; i < numRadarPts; i++)
          {
            Eigen::Matrix<double, 3,1> radarVec = radar_matrix_camera_frame(Eigen::all, i);
            transformed_radar_matrix(6, i) = radarVec.norm();
          }

          for(int i=0; i < numLidarPts; i++)
          {
            Eigen::Matrix<double, 3,1> lidarVec = lidar_matrix_camera_frame(Eigen::all, i);
            transformed_lidar_matrix(3, i) = lidarVec.norm();
          }


          for (auto col : transformed_radar_matrix.colwise())
          {
            col(0) /= col(2);
            col(1) /= col(2);
            col(2) = 1;
          }

          for (auto col : transformed_lidar_matrix.colwise())
          {
            col(0) /= col(2);
            col(1) /= col(2);
            col(2) = 1;
          }

          cv_bridge::CvImagePtr cv_ptr;
          try
          {
            cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
          }
          catch (cv_bridge::Exception& e)
          {
            RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
          }

          if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
          {
            for(auto col : transformed_radar_matrix.colwise())
            {
              RCLCPP_INFO_STREAM(this->get_logger(), "radar: " << col(0) << ", " << col(1) << endl);
              if(col(0) > 0 && col(0) < 1920 && col(1) > 0 && col(1) < 1080 && col(6) > 0.5)
              {
                double H = col(6) * 100;
                double S = 1; // col(4) > 0 ? col(4) * 100 : 100;
                double V = 1; // col(5) > 0 ? col(5) * 100 : 100;
                HSVtoRGB(col(3), col(4), col(5), H, S, V);
                // RCLCPP_INFO(this->get_logger(), "RADAR (%f, %f, %f)", H, S, V);
                cv::circle(cv_ptr->image, cv::Point(col(0), col(1)), 4/*15 * pow(col(6),-0.33)*/, CV_RGB(col(5),col(4),col(3)),-1);
                // RCLCPP_INFO_STREAM(this->get_logger(), "radar: " << col(0) << ", " << col(1) << endl);
              }
            }
            for(auto col : transformed_lidar_matrix.colwise())
            {
              if(col(0) > 0 && col(0) < 1920 && col(1) > 0 && col(1) < 1080 && col(3) > 0.5)
              {
                double R = 0.0;
                double G = 0.0;
                double B = 0.0;
                double H = col(3)*2;
                double S = 1;
                double V = 1;
                HSVtoRGB(R, G, B, H, S, V);

                // RCLCPP_INFO_STREAM(this->get_logger(), "lidar: " << col(0) << ", " << col(1) << endl);

                // RCLCPP_INFO(this->get_logger(), "LIDAR (%f, %f, %f) -> (%f, %f, %f)", H, S, V, R, G, B);
                cv::circle(cv_ptr->image, cv::Point(col(0), col(1)), 4/*20 * pow(col(3),-0.33)*/, CV_RGB(B, G, R),-1);
              }
            }
          }
          sensor_msgs::msg::Image::SharedPtr overlaid_img = cv_ptr->toImageMsg();
          overlay_pub_->publish(*overlaid_img.get());
        }
      }
    }

    Eigen::MatrixXd radarPointCloud2ToEigen(const sensor_msgs::msg::PointCloud2 & cloud_msg) const
    {
      int num_points = cloud_msg.width * cloud_msg.height;
      Eigen::MatrixXf eigen_cloud(7, num_points);

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_h(cloud_msg, "h");
      sensor_msgs::PointCloud2ConstIterator<float> iter_s(cloud_msg, "s");
      sensor_msgs::PointCloud2ConstIterator<float> iter_v(cloud_msg, "v");
      int cnt=0;
      int emptycnt=0;

      for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_h, ++iter_s, ++iter_v) 
      {
        if(*iter_x > 0.0001)
        {
          eigen_cloud(0, i-emptycnt) = *iter_x;
          eigen_cloud(1, i-emptycnt) = *iter_y;
          eigen_cloud(2, i-emptycnt) = *iter_z;
          eigen_cloud(3, i-emptycnt) = 1;
          eigen_cloud(4, i-emptycnt) = *iter_h;
          eigen_cloud(5, i-emptycnt) = *iter_s;
          eigen_cloud(6, i-emptycnt) = *iter_v;
          cnt++;
        } else {
          emptycnt++;
        }
      }
      Eigen::MatrixXd out_cloud(7,cnt);
      out_cloud = eigen_cloud(Eigen::all, Eigen::seq(0,cnt-1)).cast<double>();
      return out_cloud;
    }

    Eigen::MatrixXd lidarPointCloud2ToEigen(const sensor_msgs::msg::PointCloud2 & cloud_msg) const
    {
      int num_points = cloud_msg.width * cloud_msg.height;
      Eigen::MatrixXf eigen_cloud(10, num_points);

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud_msg, "intensity");
      sensor_msgs::PointCloud2ConstIterator<float> iter_lidart(cloud_msg, "t");
      sensor_msgs::PointCloud2ConstIterator<float> iter_reflectivity(cloud_msg, "reflectivity");
      sensor_msgs::PointCloud2ConstIterator<float> iter_ring(cloud_msg, "ring");
      sensor_msgs::PointCloud2ConstIterator<float> iter_ambient(cloud_msg, "ambient");
      sensor_msgs::PointCloud2ConstIterator<float> iter_range(cloud_msg, "range");
      int cnt=0;
      int emptycnt=0;

      for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_lidart, ++iter_reflectivity, ++iter_ring, ++iter_ambient, ++iter_range) 
      {
        if(*iter_y > 0.0001)
        {
          eigen_cloud(0, i-emptycnt) = *iter_x;
          eigen_cloud(1, i-emptycnt) = *iter_y;
          eigen_cloud(2, i-emptycnt) = *iter_z;
          eigen_cloud(3, i-emptycnt) = 1;
          eigen_cloud(4, i-emptycnt) = *iter_intensity;
          eigen_cloud(5, i-emptycnt) = *iter_lidart;
          eigen_cloud(6, i-emptycnt) = *iter_reflectivity;
          eigen_cloud(7, i-emptycnt) = *iter_ring;
          eigen_cloud(8, i-emptycnt) = *iter_ambient;
          eigen_cloud(9, i-emptycnt) = *iter_range;

          cnt++;
        } else {
          emptycnt++;
        }
      }
      Eigen::MatrixXd out_cloud(10,cnt);
      out_cloud = eigen_cloud(Eigen::all, Eigen::seq(0,cnt-1)).cast<double>();
      return out_cloud;
    }

    sensor_msgs::msg::PointCloud2 projectPointCloud2(sensor_msgs::msg::PointCloud2 & pc2, Eigen::Affine3d & projection_matrix)
    {
      sensor_msgs::msg::PointCloud2 transformed_pc2;
      geometry_msgs::msg::TransformStamped projection_tf;

      projection_tf = tf2::eigenToTransform(projection_matrix);
      tf2::doTransform(pc2, transformed_pc2, projection_tf);

      sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_pc2, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(transformed_pc2, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(transformed_pc2, "z");

      for (int i = 0; i < static_cast<int>(pc2.width*pc2.height); ++i, ++iter_x, ++iter_y, ++iter_z) 
      {
        if(!(*iter_x == 0 && *iter_x == *iter_y && *iter_y == *iter_z))
        {
          *iter_x = std::numeric_limits<float>::quiet_NaN();
          *iter_y = std::numeric_limits<float>::quiet_NaN();
          *iter_z = std::numeric_limits<float>::quiet_NaN();
        } else {
          *iter_x /= *iter_z;
          *iter_y /= *iter_z;
          *iter_z = 1;
        }
      }
      return transformed_pc2;
    }

    sensor_msgs::msg::PointCloud2 pclTransformPointCloud2(sensor_msgs::msg::PointCloud2 & pc2, Eigen::Affine3d & projection_matrix)
    {
      sensor_msgs::msg::PointCloud2 transformed_pc2;
      pcl::PointCloud<pcl::PointXYZHSV> transformed_pcl;
      pcl::PointCloud<pcl::PointXYZHSV> pcl_mtx;

      pcl::fromROSMsg(pc2, pcl_mtx);

      transformed_pcl = pcl::PointCloud<pcl::PointXYZHSV>();
      pcl::transformPointCloud(pcl_mtx, transformed_pcl, projection_matrix);
      for (pcl::PointCloud<pcl::PointXYZHSV>::iterator it = transformed_pcl.begin(); it != transformed_pcl.end(); it++) {
        if (it->x == 0 && it->y == it->x && it->y == it->z) {
          transformed_pcl.erase(it);
        } else {
          it->x=it->x / it->z;
          it->y=it->y / it->z;
          it->z=1;
        }
      }
      // for (auto & p : transformed_radar_pcl.points) 
      // {
        
      // }

      pcl::toROSMsg(transformed_pcl, transformed_pc2);
      return transformed_pc2;      
    }

    void radar_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      radar_exists_ = true;
      latest_radar_ = msg;
    }
    
    void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) 
    {
      lidar_exists_ = true;
      latest_lidar_ = msg;
    }

    void HSVtoRGB(double& fR, double& fG, double& fB, double& fH, double& fS, double& fV)
    {
      double fC = fV * fS;
      double fHPrime = fmod(fH / 60.0, 6);
      double fX = fC * (1-fabs(fmod(fHPrime, 2) - 1));
      double fM = fV - fC;

      if(0 <= fHPrime && fHPrime < 1)
      {
        fR = fC;
        fG = fX;
        fB = 0;
      } else if(1 <= fHPrime && fHPrime < 2) {
        fR = fX;
        fG = fC;
        fB = 0;
      } else if(2 <= fHPrime && fHPrime < 3) {
        fR = 0;
        fG = fC;
        fB = fX;
      } else if(3 <= fHPrime && fHPrime < 4) {
        fR = 0;
        fG = fX;
        fB = fC;
      } else if(4 <= fHPrime && fHPrime < 5) {
        fR = fX;
        fG = 0;
        fB = fC;
      } else if(5 <= fHPrime && fHPrime < 6) {
        fR = fC;
        fG = 0;
        fB = fX;
      } else {
        fR = 0;
        fG = 0;
        fB = 0;
      }
      fR += fM;
      fG += fM;
      fB += fM;
      fR *= 255;
      fG *= 255;
      fB *= 255;
    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr imu_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    sensor_msgs::msg::PointCloud2 latest_lidar_;
    sensor_msgs::msg::PointCloud2 latest_radar_;

    bool radar_exists_;
    bool lidar_exists_;

    Eigen::Matrix3d intrinsic_matrix_;
    Eigen::Matrix<double,3,4> world2camera_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Perception>());
  rclcpp::shutdown();
  return 0;
}
