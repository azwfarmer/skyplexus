#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class IMUToTFNode : public rclcpp::Node
{
public:
  IMUToTFNode()
  : Node("imu_tf_node"), 
    complementary_filter_(0.98) // alpha value for complementary filter
  {
    // Create the IMU subscriber
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::IMU>(
      "imu/data", 
      10, 
      std::bind(&IMUToTFNode::imu_callback, this, std::placeholders::_1)
    );

    // Create the TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize orientation
    orientation_ = Eigen::Quaterniond::Identity();
  }

private:
  void imu_callback(const sensor_msgs::msg::IMU::SharedPtr msg)
  {
    // Extract orientation from IMU message
    Eigen::Quaterniond imu_orientation(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z
    );

    // Complementary filter for orientation
    Eigen::Quaterniond filtered_orientation = complementary_filter_.apply(imu_orientation);
    orientation_ = filtered_orientation;

    // Create and publish the TF transform
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_link";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = orientation_.x();
    transformStamped.transform.rotation.y = orientation_.y();
    transformStamped.transform.rotation.z = orientation_.z();
    transformStamped.transform.rotation.w = orientation_.w();

    tf_broadcaster_->sendTransform(transformStamped);
  }

  class ComplementaryFilter
  {
  public:
    ComplementaryFilter(double alpha) : alpha_(alpha), orientation_(Eigen::Quaterniond::Identity()) {}

    Eigen::Quaterniond apply(const Eigen::Quaterniond& imu_orientation)
    {
      // Apply complementary filter
      Eigen::Quaterniond gyro_orientation = orientation_ * imu_orientation;
      orientation_ = alpha_ * gyro_orientation + (1 - alpha_) * imu_orientation;
      return orientation_;
    }

  private:
    double alpha_;
    Eigen::Quaterniond orientation_;
  };

  rclcpp::Subscription<sensor_msgs::msg::IMU>::SharedPtr imu_subscriber_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::Quaterniond orientation_;
  ComplementaryFilter complementary_filter_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUToTFNode>());
  rclcpp::shutdown();
  return 0;
}
