#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "custom_interfaces/srv/go_to_loading.hpp"

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    // Declare parameters with their default values
    this->declare_parameter<double>("obstacle", 0.5);
    this->declare_parameter<int>("degrees", -90);
    this->declare_parameter<bool>("final_approach", false);

    // Get parameter values
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    final_approach_client_ =
        this->create_client<custom_interfaces::srv::GoToLoading>(
            "/approach_shelf");

    // Wait for the service to be available (checks every second)
    while (!final_approach_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service /approach_shelf not available, waiting again...");
    }
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::Twist vel_msg;

    // If first scan determine index at front of robot
    if (first_scan_) {
      front_index_ =
          static_cast<int>(std::round(-msg->angle_min / msg->angle_increment));
      first_scan_ = false;
    }

    // Initial movement forward, check distance at front compare it to obstacle
    // distance. If not within parameter drive forward
    float front_distance = msg->ranges[front_index_];
    if (front_distance > obstacle_ && !stop_criteria_) {
      vel_msg.linear.x = 0.5;
      publisher_vel_->publish(vel_msg);
      return;

    } else {
      // If within distance parameter stop moving forward and change
      // stop_criteria_ to true preventing going into drive logic again.
      vel_msg.linear.x = 0.0;
      publisher_vel_->publish(vel_msg);
      stop_criteria_ = true;
    }

    // This should only occur IF stop criteria is true
    // Find error yaw
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

    if (!pre_approach_completed_) {
      pre_approach_completed_ = true;
      // Target reached
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      publisher_vel_->publish(vel_msg);
    }

    if (pre_approach_completed_ && !final_approach_completed_) {
      final_approach_completed_ = true;
      auto req =
          std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
      req->attach_to_shelf = final_approach_;

      auto future = final_approach_client_->async_send_request(
          req, std::bind(&PreApproach::response_callback, this,
                         std::placeholders::_1));
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    {
      // Use odom to calculate current_yaw_
      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

      // Once stop_criteria_ is allowed i.e. we have completed driving forward
      // section AND this is the first time we've stopped we find
      // starting_yaw_ for our turn We also use that to calculate our
      // target_yaw_ Set first_stop_ to false preventing recalculating of
      // target/starting yaw
      if (stop_criteria_ && first_stop_) {
        starting_yaw_ = current_yaw_;
        first_stop_ = false;
        target_yaw_ = starting_yaw_ + M_PI * degrees_ / 180;
      }
    }
  }

  void response_callback(
      rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture
          future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
                "/approach_shelf Service Response Received: %d",
                response->complete);
  }

private:
  // Declared variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr
      final_approach_client_;

  bool first_scan_{true};
  bool stop_criteria_{false};
  bool first_stop_{true};
  bool pre_approach_completed_{false};
  bool final_approach_completed_{false};
  bool final_approach_;
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

  rclcpp::shutdown();
  return 0;
}