#pragma once
#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <sensor_msgs/msg/joint_state.hpp>

class CV_detection : public BT::StatefulActionNode {
public:
  CV_detection(const std::string& name,
                const BT::NodeConfiguration& config,
                rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts();

  virtual BT::NodeStatus onStart() override;
  virtual BT::NodeStatus onRunning() override;
  virtual void onHalted() override;

private:
  void processValues();
  double calculateAverage(const std::vector<double>& values);
  void clearData();

private:
  // ros parameter's
  bool full_info_;
  bool dry_run_;

private:
  rclcpp::Node::SharedPtr node_;

  std::vector<std::string> names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  std::string narrow = "No_detection";
  double length = 0.0;
  double angle = 0.0;
  double coef = 0.0;

  std::mutex mut_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
  rclcpp::Node::SharedPtr node;
};
