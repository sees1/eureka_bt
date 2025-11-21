#pragma once
#include <future>
#include <cmath>
#include <chrono>
#include <deque>

#include "utils/arrow_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Turn_inside : public BT::StatefulActionNode {
public:
  Turn_inside(const std::string& name,
              const BT::NodeConfiguration& config,
              rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus onStart() override;
  virtual BT::NodeStatus onRunning() override;
  virtual void onHalted() override;

private:
  // before been in cv node
  bool processValues();

private:
  bool stopRobot();
  bool rotateWheels();
  void updateRotation(std::string& turn_arrow);

private:
  static double computeRobotYaw(tf2::Quaternion& pose_q);

private:
  // ros parameter's
  size_t buffer_size_;
  double dummy_rotation_dur_;
  double too_far_length_;
  double too_big_angle_;
  std::string odometry_topic_name_;

private:
  rclcpp::Node::SharedPtr node_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turn_pub_;
  
  // processing flag's
  bool turning_task_finished_;

  // rotate logic parameter's
  bool rotate_fire_once_;
  std::chrono::time_point<std::chrono::steady_clock> rotate_time_point_;
  
  // arrow logic substracted from cv node
  std::shared_ptr<ArrowFilter> arrow_acc_;
  Arrow current_arrow_;
  std::string turn_direction_;

  tf2::Quaternion pose_quat_;

  // controll take data from topic and publish goal process
  std::mutex mut_;
};

