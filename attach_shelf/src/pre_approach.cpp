#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    // Declare parameters with their default values
    this->declare_parameter<double>("obstacle", 0.5);
    this->declare_parameter<int>("degrees", -90);

    // Get parameter values
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();

    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::Twist vel_msg;
    if (first_scan_) {
      front_index_ =
          static_cast<int>(std::round(-msg->angle_min / msg->angle_increment));
      first_scan_ = false;
    }

    float front_distance = msg->ranges[front_index_];
    if (front_distance > obstacle_ && !stop_criteria_) {
      vel_msg.linear.x = 0.5;
      publisher_vel_->publish(vel_msg);
      return;

    } else {
      vel_msg.linear.x = 0.0;
      publisher_vel_->publish(vel_msg);
      stop_criteria_ = true;
    }
    double error_yaw = target_yaw_ - current_yaw_;
    while (error_yaw > M_PI)
      error_yaw -= 2.0 * M_PI;

    while (error_yaw < -M_PI)
      error_yaw += 2.0 * M_PI;

    if (std::abs(error_yaw) > 0.05) {
      vel_msg.angular.z = error_yaw / 2;
      publisher_vel_->publish(vel_msg);
      return;
    }

    if (!completed_) {
      completed_ = true;
      // Target reached
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      publisher_vel_->publish(vel_msg);

      RCLCPP_INFO(this->get_logger(), "Target reached. Shutting down.");

      rclcpp::shutdown();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

      if (stop_criteria_ && first_stop_) {
        starting_yaw_ = current_yaw_;
        first_stop_ = false;
        target_yaw_ = starting_yaw_ + M_PI * degrees_ / 180;
      }
      // yaw is your heading in radians
    }
  }

private:
  // Declared variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;

  bool first_scan_{true};
  bool stop_criteria_{false};
  bool first_stop_{true};
  bool completed_{false};
  int front_index_;
  double obstacle_;
  int degrees_;
  double current_yaw_{0.0};
  double starting_yaw_{0.0};
  double target_yaw_{0.0};
};

int main(int argc, char **argv) {
  // initialize the ROS2 communication
  rclcpp::init(argc, argv);
  // declare the node constructor
  auto node = std::make_shared<PreApproach>();
  // keeps the node alive, waits for a request to kill the node (ctrl+c)
  rclcpp::spin(node);
  return 0;
}