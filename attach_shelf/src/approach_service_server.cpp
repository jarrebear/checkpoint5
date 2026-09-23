#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "custom_interfaces/srv/go_to_loading.hpp"

class ApproachService : public rclcpp::Node {
public:
  using Approach = custom_interfaces::srv::GoToLoading;

  ApproachService()
      : Node("approach_service_node"), tf_buffer_(this->get_clock()) {

    approach_service_ = this->create_service<Approach>(
        "/approach_shelf",
        std::bind(&ApproachService::approach_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachService::scan_callback, this,
                  std::placeholders::_1));

    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    publisher_elevator_ =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    timer_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    cart_frame_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ApproachService::republish_cart_frame, this), timer_group_);

    RCLCPP_INFO(this->get_logger(), "Approach Service Ready");
  }

private:
  void approach_callback(const std::shared_ptr<Approach::Request> request,
                         std::shared_ptr<Approach::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Service Requested");

    auto [is_valid, indices] = find_shelf_legs();

    if (!is_valid) {
      RCLCPP_WARN(this->get_logger(), "Failed to detect both cart frame legs.");

      response->complete = false;
      return;
    }

    // Find the center index for each leg
    const int leg_1_index = center_index(indices[0], indices[1]);

    const int leg_2_index = center_index(indices[2], indices[3]);

    // Find the x,y coordinate of each leg
    const auto [leg_1_x, leg_1_y] = scan_point(leg_1_index);

    const auto [leg_2_x, leg_2_y] = scan_point(leg_2_index);

    // Find the midpoint of the two leg coordinates
    const double midpoint_x = (leg_1_x + leg_2_x) / 2.0;

    const double midpoint_y = (leg_1_y + leg_2_y) / 2.0;

    RCLCPP_INFO(this->get_logger(), "Shelf midpoint: x=%.2f, y=%.2f",
                midpoint_x, midpoint_y);

    if (!publish_cart_frame(midpoint_x, midpoint_y)) {
      response->complete = false;
      return;
    }

    if (request->attach_to_shelf) {
      if (!approach_cart_frame()) {
        response->complete = false;
        return;
      }

      if (!move_forward_30cm()) {
        response->complete = false;
        return;
      }

      std_msgs::msg::String elevator_up;
      elevator_up.data = "";
      publisher_elevator_->publish(elevator_up);

      response->complete = true;
    } else {
      response->complete = true;
    }
  }

  std::pair<bool, std::vector<int>> find_shelf_legs() {
    std::vector<int> max_indices;
    std::vector<float> local_intensities;

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);

      if (!current_scan_) {
        return {false, {}};
      }

      local_intensities = current_scan_->intensities;
    }

    for (size_t i = 0; i < local_intensities.size(); ++i) {
      // It's the very first element and it's active
      if (i == 0 && local_intensities[i] > 0) {
        max_indices.push_back(i);
      }
      // A increase in intensity is starting (current is high, previous was low)
      else if (i > 0 && local_intensities[i] > 0 &&
               approx_equal_absolute(local_intensities[i - 1], 0.0)) {
        max_indices.push_back(i);
      }
      // A increase in intensity just ended (current is low, previous was high)
      // We record (i - 1) to get the final index
      else if (i > 0 && approx_equal_absolute(local_intensities[i], 0.0) &&
               local_intensities[i - 1] > 0) {
        max_indices.push_back(i - 1);
      }
    }

    bool success = (max_indices.size() == 4);

    return {success, max_indices};
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(scan_mutex_);
    current_scan_ = msg;
  }

  bool approx_equal_absolute(float a, float b, float epsilon = 0.00001f) {

    return std::abs(a - b) <= epsilon;
  }

  int center_index(int a, int b) { return (a + b) / 2; }

  std::pair<double, double> scan_point(int index) {
    double angle;
    double distance;

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);

      angle = current_scan_->angle_min + index * current_scan_->angle_increment;

      distance = current_scan_->ranges[index];
    }

    return {std::cos(angle) * distance, std::sin(angle) * distance};
  }

  bool publish_cart_frame(double x, double y) {
    std::string scan_frame;

    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan_frame = current_scan_->header.frame_id;
    }

    geometry_msgs::msg::TransformStamped scan_to_odom;

    try {
      scan_to_odom =
          tf_buffer_.lookupTransform("odom", scan_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not transform shelf midpoint to odom: %s", ex.what());

      return false;
    }

    tf2::Transform odom_to_scan;

    tf2::Quaternion q(
        scan_to_odom.transform.rotation.x, scan_to_odom.transform.rotation.y,
        scan_to_odom.transform.rotation.z, scan_to_odom.transform.rotation.w);

    odom_to_scan.setOrigin(tf2::Vector3(scan_to_odom.transform.translation.x,
                                        scan_to_odom.transform.translation.y,
                                        scan_to_odom.transform.translation.z));

    odom_to_scan.setRotation(q);

    tf2::Vector3 midpoint_in_scan(x, y, 0.0);

    tf2::Vector3 midpoint_in_odom = odom_to_scan * midpoint_in_scan;

    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "cart_frame";

    transform.transform.translation.x = midpoint_in_odom.x();

    transform.transform.translation.y = midpoint_in_odom.y();

    transform.transform.translation.z = midpoint_in_odom.z();

    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    {
      std::lock_guard<std::mutex> lock(tf_mutex_);
      cart_frame_tf_ = transform;
      cart_frame_ready_ = true;
    }

    return true;
  }

  void republish_cart_frame() {
    std::lock_guard<std::mutex> lock(tf_mutex_);

    if (!cart_frame_ready_) {
      return;
    }

    cart_frame_tf_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(cart_frame_tf_);
  }

  bool approach_cart_frame() {
    const double distance_tolerance = 0.05;

    while (rclcpp::ok()) {
      try {
        auto transform = tf_buffer_.lookupTransform(
            "robot_base_link", "cart_frame", tf2::TimePointZero);

        const double x = transform.transform.translation.x;

        const double y = transform.transform.translation.y;

        const double error_distance =
            calculate_distance(transform.transform.translation);

        const double error_yaw = std::atan2(y, x);

        RCLCPP_INFO(this->get_logger(), "Distance: %.2f m, Yaw: %.2f rad",
                    error_distance, error_yaw);

        // Stop once we are close enough to cart_frame.
        if (error_distance <= distance_tolerance) {
          break;
        }

        // Calculate velocity.
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x =
            std::clamp(error_distance * kp_distance_, 0.0, max_linear_speed_);

        msg.angular.z = std::clamp(error_yaw * kp_yaw_, -max_angular_speed_,
                                   max_angular_speed_);

        publisher_vel_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing: linear.x=%.2f, angular.z=%.2f", msg.linear.x,
                    msg.angular.z);

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not get transform: %s", ex.what());
      }

      // Wait 100 ms before checking TF again.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Make absolutely sure the robot stops.
    stop_robot();

    RCLCPP_INFO(this->get_logger(), "Reached cart_frame.");

    return true;
  }

  bool move_forward_30cm() {
    const double target_distance = 0.50;
    const double linear_speed = 0.10;

    geometry_msgs::msg::TransformStamped start_transform;

    try {
      start_transform = tf_buffer_.lookupTransform("odom", "robot_base_link",
                                                   tf2::TimePointZero);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not get starting odom transform: %s", ex.what());

      stop_robot();
      return false;
    }

    const double start_x = start_transform.transform.translation.x;

    const double start_y = start_transform.transform.translation.y;

    while (rclcpp::ok()) {
      geometry_msgs::msg::TransformStamped current_transform;

      try {
        current_transform = tf_buffer_.lookupTransform(
            "odom", "robot_base_link", tf2::TimePointZero);

      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not get odom transform: %s", ex.what());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        continue;
      }

      const double current_x = current_transform.transform.translation.x;

      const double current_y = current_transform.transform.translation.y;

      const double dx = current_x - start_x;

      const double dy = current_y - start_y;

      const double distance_traveled = std::sqrt(dx * dx + dy * dy);

      RCLCPP_INFO(this->get_logger(), "Moved %.2f / %.2f meters",
                  distance_traveled, target_distance);

      // Stop once we have traveled 30 cm.
      if (distance_traveled >= target_distance) {
        break;
      }

      geometry_msgs::msg::Twist cmd;

      cmd.linear.x = linear_speed;
      cmd.angular.z = 0.0;

      publisher_vel_->publish(cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the robot.
    stop_robot();

    RCLCPP_INFO(this->get_logger(), "Moved forward 30 cm.");

    return true;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop_cmd;

    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;

    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;

    publisher_vel_->publish(stop_cmd);
  }

  double calculate_distance(const geometry_msgs::msg::Vector3 &translation) {

    return std::sqrt(translation.x * translation.x +
                     translation.y * translation.y);
  }

private:
  rclcpp::Service<Approach>::SharedPtr approach_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_elevator_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  std::mutex scan_mutex_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::TimerBase::SharedPtr cart_frame_timer_;

  geometry_msgs::msg::TransformStamped cart_frame_tf_;
  bool cart_frame_ready_ = false;
  std::mutex tf_mutex_;

  double kp_distance_ = 0.5;
  double kp_yaw_ = 1.0;

  double max_linear_speed_ = 0.25;
  double max_angular_speed_ = 0.5;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ApproachService>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
