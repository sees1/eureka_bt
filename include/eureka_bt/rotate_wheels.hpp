#pragma once
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/msg/twist.hpp>

class RotateWheels : public BT::StatefulActionNode {
public:
  RotateWheels(const std::string& name,
               const BT::NodeConfiguration& config,
               rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts() { return BT::PortsList(); };

  virtual BT::NodeStatus onStart() override;
  virtual BT::NodeStatus onRunning() override;
  virtual void onHalted() override;
  
private:
  // ros parameters
  double rotate_duration_;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotate_pub_;
  
  std::chrono::time_point<std::chrono::steady_clock> rotate_time_point_;
  geometry_msgs::msg::Twist twist_msg_;
};
