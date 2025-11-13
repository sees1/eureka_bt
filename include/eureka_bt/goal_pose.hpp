#pragma once
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


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
  // before been in cv node
  void processValues();
  double calculateAverage(const std::vector<double>& values);
  void clearData();

  bool fullClearing();

  void publishGoalPose(double length, double angle);
  bool isRobotNearGoal();

private:
  // ros parameters
  bool full_info_;
  bool dry_run_;
  double lenght_error_;
  std::string odometry_topic_name_;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub;

  bool already_published;
  double current_goal_x;
  double current_goal_y;

  std::vector<std::string> names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  std::chrono::time_point<std::chrono::steady_clock> last_time_point_;

  std::string narrow = "No_detection";
  double length = 0.0;
  double angle = 0.0;
  double coef = 0.0;

  bool full_clear = false;
  bool clearing_fire_once_ = false;
  
  std::mutex mut_;
  double posex;
  double posey;
  double orientationw;
  double orientationx;
  double orientationy;
  double orientationz;
};
