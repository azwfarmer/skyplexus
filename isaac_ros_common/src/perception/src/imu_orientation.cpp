#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <string>
#include "tf2_eigen/tf2_eigen.hpp"

#define g 9.80665
#define pi 3.1415926535897931

struct IMUData {
  double roll_acc_prev;
  double pitch_acc_prev;
  double alpha_lpf;

  IMUData() :
      roll_acc_prev(0.0),
      pitch_acc_prev(0.0),
      alpha_lpf(0.1)
      {}
};

class ImuOrient : public rclcpp::Node
{
public:
  ImuOrient()
  : Node("turtle_tf2_frame_publisher")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    compass_sub_ = this->create_subscription<std_msgs::msg::Float64>("mavros/global_position/compass_hdg", qos, std::bind(&ImuOrient::handle_compass, this, std::placeholders::_1));
    mavros_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data_raw", qos, std::bind(&ImuOrient::handle_imu, this, std::placeholders::_1));

    yaw_ = 0.0;
    hdg_difference_ = pi * 183.11 / 180.0;
    
    mavros_imu_ = IMUData();

    mavros_to_os_extrinsic_ = Eigen::Matrix3d();
    mavros_to_os_extrinsic_ << 0.801, 0.061, -0.595,
                              0.023, 0.997, -0.072,
                              -0.598, 0.045, 0.801;
  }

private:
  double timeToSeconds(const builtin_interfaces::msg::Time & time_msg) {
      return time_msg.sec + time_msg.nanosec * 1e-9;
  }

  double normalize_angle(const double & ang)
  {
    double out = ang;
    while (out > 2 * pi) {
      out -= 2 * pi;
    } 
    while (out < 0) {
      out += 2 * pi;
    }
    return out;
  }

  void publish_tf(const double & roll, const double & pitch, const double & yaw, builtin_interfaces::msg::Time & stamp, const char* frame_id)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;

    t.header.frame_id = "base_link";
    t.child_frame_id = frame_id;

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = -0.5;

    tf2::Quaternion q;

    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(t);
  }

  void handle_compass(const std::shared_ptr<std_msgs::msg::Float64> msg)
  {
    yaw_ = normalize_angle((pi * msg->data / 180) - hdg_difference_);
  }
  
  void handle_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    double roll_acc = normalize_angle(atan2(ay, az));
    double pitch_acc = atan2(-ax, sqrt(pow(ay,2)+pow(az,2)));

    IMUData& imu_data = mavros_imu_;

    double roll_lpf = normalize_angle(imu_data.alpha_lpf * roll_acc + (1 - imu_data.alpha_lpf) * imu_data.roll_acc_prev);
    double pitch_lpf = normalize_angle(0.63704517697 + (imu_data.alpha_lpf * pitch_acc + (1 - imu_data.alpha_lpf) *  imu_data.pitch_acc_prev));
    
    // double pitch_lpf = normalize_angle((imu_data.alpha_lpf * pitch_acc + (1 - imu_data.alpha_lpf) *  imu_data.pitch_acc_prev));

    // Eigen::Matrix3d x_matrix;
    // x_matrix << 1, 0, 0,
    //             0, cos(roll_lpf), -sin(roll_lpf),
    //             0, sin(roll_lpf), cos(roll_lpf);

    // Eigen::Matrix3d y_matrix;
    // y_matrix << cos(pitch_lpf), 0, sin(pitch_lpf),
    //             0, 1, 0,
    //             -sin(pitch_lpf), 0, cos(pitch_lpf);

    // Eigen::Matrix3d z_matrix;
    // z_matrix << cos(yaw_), -sin(yaw_), 0,
    //             sin(yaw_), cos(yaw_), 0,
    //             0, 0, 1;

    // Eigen::Matrix3d rot_matrix = mavros_to_os_extrinsic_ * x_matrix * y_matrix * z_matrix;

    // double roll_out = atan2(rot_matrix(2,1), rot_matrix(2,2));
    // double pitch_out = -asin(rot_matrix(2,0));
    // double yaw_out = atan2(rot_matrix(1,0), rot_matrix(0,0));

    imu_data.roll_acc_prev = roll_acc;
    imu_data.pitch_acc_prev = pitch_acc;

    publish_tf(0.0, 0.63704517697, yaw_, msg->header.stamp, "payload");
    // publish_tf(roll_out, pitch_out, yaw_out, msg->header.stamp, "payload");

  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mavros_imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  double yaw_;
  double hdg_difference_;

  IMUData mavros_imu_;

  Eigen::Matrix3d mavros_to_os_extrinsic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOrient>());
  rclcpp::shutdown();
  return 0;
}