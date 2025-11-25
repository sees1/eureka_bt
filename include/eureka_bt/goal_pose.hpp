#pragma once
#include <cmath>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <tuple>
#include <exception>

#include "utils/false_positive_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rcppmath/rolling_mean_accumulator.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

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
  std::tuple<double, double, double> computeRobotAngles();
  double normalizeAngle(double angle);
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

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub_;

  bool already_published_;
  double current_goal_x_;
  double current_goal_y_;
  double transform_tolerance_;

  bool republish_once_ = false;
  bool first_pub;

  Object current_arrow_;
  Object current_cone_;
  std::shared_ptr<FalsePositiveFilter> arrow_acc_;
  std::shared_ptr<FalsePositiveFilter> cone_acc_;
  
  geometry_msgs::msg::TransformStamped current_robot_transform_;
  geometry_msgs::msg::PoseStamped current_goal_;

  std::mutex mut_;
};
