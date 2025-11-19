#pragma once
#include <cmath>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
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
  Goalpose(const std::string& name,
           const BT::NodeConfiguration& config,
           rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus onStart() override;
  virtual BT::NodeStatus onRunning() override;
  virtual void onHalted() override;
  
private:
  bool isRobotNearGoal();
  void publishGoalPose(double length, double angle);

private:
  // before been in cv node
  void processValues();
  double calculateAverage(const std::deque<double>& values);

private:
  // ros parameters
  bool full_info_;
  bool dry_run_;
  double length_error_delta_;
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub_;

  bool already_published_;
  double current_goal_x_;
  double current_goal_y_;

  std::string narrow_ = "none";
  double length_ = 0.0;
  double angle_ = 0.0;
  double coef_ = 0.0;

  std::deque<std::string> names_;
  std::deque<double>      positions_;
  std::deque<double>      velocities_;
  std::deque<double>      efforts_;
  
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;

  std::mutex mut_;
};
