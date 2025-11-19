#include "eureka_bt/rotate_wheels.hpp"

RotateWheels::RotateWheels(const std::string& name,
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  if (node_->has_parameter("rotate_duration"))
    rotate_duration_ = node_->get_parameter("rotate_duration").as_double();
  else
    rotate_duration_ = node_->declare_parameter("rotate_duration", 5.0);

  rotate_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::NodeStatus RotateWheels::onStart()
{
  rotate_time_point_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateWheels::onRunning()
{
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 100.0;

  rotate_pub_->publish(twist_msg_);

  RCLCPP_INFO(node_->get_logger(), "(RotateWheels) Robot in rotate wheels process!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rotate_time_point_).count() / 1000.0;

  if (dt > rotate_duration_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::RUNNING;
}

void RotateWheels::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(RotateWheels) current node is halted!");
}