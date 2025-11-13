#include "eureka_bt/turn_inside.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

Turn_inside::Turn_inside(const std::string& name,
                         const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  turning_task_finished_(true),
  is_robot_stop_(false),
  before_turning_yaw_(0.0),
  turn_angle_(0.0),
  stop_fire_once_(false)
{
  if (node_->has_parameter("dry_run"))
    dry_run_ = node_->get_parameter("dry_run").as_bool();
  else
    dry_run_ = node_->declare_parameter("dry_run", false);

  if (node_->has_parameter("odometry_topic_name"))
    odometry_topic_name_ = node_->get_parameter("odometry_topic_name").as_string();
  else  
    odometry_topic_name_ = node_->declare_parameter("odometry_topic_name", "/odometry");

  if (node_->has_parameter("buffer_size"))
    buffer_size_ = static_cast<size_t>(node_->get_parameter("buffer_size").as_int());
  else
    buffer_size_ = static_cast<size_t>(node_->declare_parameter("buffer_size", 10));

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      pose_quat_ = tf2::Quaternion(msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y,
                                   msg->pose.pose.orientation.z,
                                   msg->pose.pose.orientation.w);
    }
  );

  arrow_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lc(mut_);

      if (!dry_run_)
      {
        double max = 0;
        size_t max_idx = 0;
        for(size_t idx = 0; idx < msg->effort.size(); ++idx)
        {
          if (msg->effort[idx] > max)
          {
            max = msg->effort[idx];
            max_idx = idx;
          }
        }

        if (names_.size() == buffer_size_)
          names_.pop_front();

        names_.push_back(msg->name[max_idx]);
      }
    }
  );

  turn_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::PortsList Turn_inside::providedPorts() 
{
  return BT::PortsList();
}

BT::NodeStatus Turn_inside::onStart()
{
  turning_task_finished_ = false;
  is_robot_stop_ = false;

  return BT::NodeStatus::RUNNING;
} 

BT::NodeStatus Turn_inside::onRunning() 
{
  std::lock_guard<std::mutex> lc(mut_);

  if (dry_run_)
  {
    if (!turning_task_finished_)
    {
      if (!is_robot_stop_)
      {
        is_robot_stop_ = stopRobot();

        before_turning_yaw_ = computeRobotYaw(pose_quat_);
        turn_angle_ = 90.0;
      }
      else
        updateRotation(turn_angle_);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot sucessfully rotated on place!");
      return BT::NodeStatus::SUCCESS;
    }
  }
  else
  {
    if (!turning_task_finished_)
    {
      if (!is_robot_stop_)
      {
        is_robot_stop_ = stopRobot();

        processValues();
        before_turning_yaw_ = computeRobotYaw(pose_quat_);
        turn_angle_ = (narrow_ == "left") ? 90.0 : -90.0;
      }
      else
        updateRotation(turn_angle_);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot sucessfully rotated on place!");
      return BT::NodeStatus::SUCCESS;
    }
  }
    
  return BT::NodeStatus::RUNNING;
}

void Turn_inside::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(Turn_inside) Current node is halted!");
}

void Turn_inside::processValues()
{
  bool has_no_detection = std::any_of(names_.begin(), names_.end(),
    [](const std::string& name)
    {
      return name == "No_detection";
    }
  );

  narrow_ = has_no_detection ? "No_detection" : names_.back();
}

bool Turn_inside::stopRobot()
{
  if (!stop_fire_once_)
  {
    last_time_point_ = std::chrono::steady_clock::now();
    stop_fire_once_ = true;
  }

  geometry_msgs::msg::Twist twist_msg;

  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
  turn_pub_->publish(twist_msg);

  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot in stop process!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_time_point_).count() / 1000.0;

  if (dt > 4.0)
    return true;
  else
    return false;
}

void Turn_inside::updateRotation(double turn_angle)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 100.0;

  if (turn_angle > 0)
  {
    twist_msg.linear.x = 0.2;
  }
  else
  {
    twist_msg.linear.x = -0.2;
  }

  double current_robot_yaw = computeRobotYaw(pose_quat_);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Current robot yaw = %f", current_robot_yaw);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Befor turning robot yaw = %f", before_turning_yaw_);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Turning yaw = %f deg", turn_angle);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) After turning robot yaw = %f", before_turning_yaw_ + (turn_angle * M_PI) / 180);

  if ((turn_angle > 0 && before_turning_yaw_ + (turn_angle * M_PI) / 180 > current_robot_yaw) ||
      (turn_angle < 0 && before_turning_yaw_ + (turn_angle * M_PI) / 180 < current_robot_yaw))
  {
    turn_pub_->publish(twist_msg);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Succesfully turned robot!");
    is_robot_stop_ = false;
    turning_task_finished_ = true;
    stop_fire_once_ = false;
  }
}

// static member's

double Turn_inside::computeRobotYaw(tf2::Quaternion& pose_q)
{
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);

  return pose_yaw;
}
