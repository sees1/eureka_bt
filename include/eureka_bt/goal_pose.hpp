#pragma once
#include <cmath>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>

#include "utils/arrow_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rcppmath/rolling_mean_accumulator.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav2_simple_commander/basic_nav.hpp"


class Goalpose : public BT::StatefulActionNode {
public:
  using ResultCode = rclcpp_action::ResultCode;
  using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
public:
  Goalpose(const std::string& name,
           const BT::NodeConfiguration& config,
           rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus onStart() override;
  virtual BT::NodeStatus onRunning() override;
  virtual void onHalted() override;
  
private:
  double computeRobotYaw();
  void publishGoalPose(double length, double angle);

private:
  // before been in cv node
  bool processValues();

private:
  // ros parameters
  bool full_info_;
  double length_error_delta_;
  double navigation_time_limit_;
  double enough_close_to_republish_;
  size_t buffer_size_;
  double too_far_length_;
  std::string odometry_topic_name_;

  // rover constraints
  float base_length = 1.0f;
  float body_width = 0.5f;

private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<eureka_bt::BasicNavigator> navigator_;
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::WrappedResult> go_to_pose_res_;
  TimePoint start_navigation_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub_;

  bool already_published_;
  double current_goal_x_;
  double current_goal_y_;

  bool republish_once_ = false;

  Arrow current_arrow_;
  std::shared_ptr<ArrowFilter> arrow_acc_;
  
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;

  std::mutex mut_;
};
