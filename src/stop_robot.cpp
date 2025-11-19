#include "eureka_bt/stop_robot.hpp"

StopRobot::StopRobot(const std::string& name,
                     const BT::NodeConfiguration& config,
                     rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  if (node_->has_parameter("stop_duration"))
    stop_duration_ = node_->get_parameter("stop_duration").as_double();
  else
    stop_duration_ = node_->declare_parameter("stop_duration", 5.0);

  stop_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::NodeStatus StopRobot::onStart()
{
  stop_time_point_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopRobot::onRunning()
{
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;

  stop_pub_->publish(twist_msg_);

  RCLCPP_INFO(node_->get_logger(), "(StopRobot) Robot in stop process!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - stop_time_point_).count() / 1000.0;

  if (dt > stop_duration_)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::RUNNING;
}

void StopRobot::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(StopRobot) current node is halted!");
}