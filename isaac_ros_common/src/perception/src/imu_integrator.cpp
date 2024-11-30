#include <memory>
#include <functional>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#define g 9.80665
#define pi 3.1415926535897931

using std::placeholders::_1;

class ImuIntegrator : public rclcpp::Node
{
public:
ImuIntegrator() : Node("imu_integrator")
  {
    name_ = this->declare_parameter<std::string>("name", "os_imu");
    t_ = geometry_msgs::msg::TransformStamped();
    t_.header.frame_id = "map";
    t_.child_frame_id = name_.c_str();
    t_.header.stamp = rclcpp::Time(0);

    twist_ = geometry_msgs::msg::TwistStamped();
    twist_.header.frame_id = "map";
    twist_.header.stamp = rclcpp::Time(0);

    prev_linear_accel_ = geometry_msgs::msg::Vector3();
    prev_linear_accel_.x = 0.0;
    prev_linear_accel_.y = 0.0;
    prev_linear_accel_.z = 0.0;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    // sub_ = this->create_subscription<sensor_msgs::msg::Imu>("mavros/imu/data", qos, std::bind(&ImuIntegrator::callback, this, _1));
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>("ouster/imu", qos, std::bind(&ImuIntegrator::callback, this, _1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);

    first_ = true;
  }

private:
  void callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    rclcpp::Time msg_time(msg->header.stamp);
    rclcpp::Time prev_time(t_.header.stamp);
    double dt;
    if(prev_time.seconds() == 0)
    {
      dt = 0;
    } else {
      dt = msg_time.seconds() - prev_time.seconds();
    }
     
    t_.header.stamp = msg->header.stamp;
    
    t_.transform.translation.x += 0.5*(msg->linear_acceleration.x+prev_linear_accel_.x+4.906675*2)*pow(dt,2);
    t_.transform.translation.y += 0.5*(msg->linear_acceleration.y + prev_linear_accel_.y-0.658901*2)*pow(dt,2);
    t_.transform.translation.z += 0.5*(msg->linear_acceleration.z+prev_linear_accel_.z+8.343534*2)*pow(dt,2);

    t_.transform.rotation = msg->orientation;

    tf_broadcaster_->sendTransform(t_);
    prev_linear_accel_ = msg->linear_acceleration;

    twist_.header.stamp = msg->header.stamp;
    twist_.twist.linear.x += dt * (msg->linear_acceleration.x+4.906675);
    twist_.twist.linear.y += dt * (msg->linear_acceleration.y-0.658901);
    twist_.twist.linear.z += dt * (msg->linear_acceleration.z+8.343534);
    twist_.twist.angular = msg->angular_velocity;

    RCLCPP_INFO(this->get_logger(), "Current accel X %f", msg->linear_acceleration.x + 4.906675);
    RCLCPP_INFO(this->get_logger(), "X VEL (integrated) %f", twist_.twist.linear.x);
    RCLCPP_INFO(this->get_logger(), "GUESS %f", dt * (msg->linear_acceleration.x+4.906675));


    twist_pub_->publish(twist_);

    if (first_) {
      RCLCPP_INFO(this->get_logger(), "Pitch: %f, Roll: %f", 180.0/pi*atan2(msg->linear_acceleration.x,g), 180.0/pi*atan2(msg->linear_acceleration.y,msg->linear_acceleration.z));
      first_ = false;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  geometry_msgs::msg::Vector3 prev_linear_accel_;
  geometry_msgs::msg::TransformStamped t_;
  geometry_msgs::msg::TwistStamped twist_;
  std::string name_;
  bool first_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuIntegrator>());
  rclcpp::shutdown();
  return 0;
}