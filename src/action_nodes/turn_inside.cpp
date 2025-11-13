#include "eureka_bt/turn_inside.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

Turn_inside::Turn_inside(const std::string& name,
                         const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  collect_success_(false),
  turning_task_finished_(true),
  is_robot_stop_(false),
  stop_fire_once_(false),
  before_turning_yaw_(0.0),
  turn_angle_(0.0)
{
  if (node_->has_parameter("dry_run"))
    dry_run_ = node_->get_parameter("dry_run").as_bool();
  else
    dry_run_ = node_->declare_parameter("dry_run", false);

  if (node_->has_parameter("odometry_topic_name"))
    odometry_topic_name_ = node_->get_parameter("odometry_topic_name").as_string();
  else  
    odometry_topic_name_ = node_->declare_parameter("odometry_topic_name", "/odometry");

  pose_sub = node_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      pose_x_ = msg->pose.pose.position.x;
      pose_y_ = msg->pose.pose.position.y;
      orientationw = msg->pose.pose.orientation.w;
      orientationx = msg->pose.pose.orientation.x;
      orientationy = msg->pose.pose.orientation.y;
      orientationz = msg->pose.pose.orientation.z;
    }
  );

  arrow_sub = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // RCLCPP_INFO(node_->get_logger(), "(Turn_inside) arrow compute loop called!");

      if (!msg->name.empty() && !dry_run_ && names_.size() != 5)
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

        names_.emplace_back(msg->name[max_idx]);
        positions_.emplace_back(msg->position[max_idx]);
        velocities_.emplace_back(msg->velocity[max_idx]);
        efforts_.emplace_back(msg->effort[max_idx]);
      }
    }
  );

  publisher_turn = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
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

bool Turn_inside::tryToCollectData()
{
  if (dry_run_)
    return true;

  if (names_.size() == 5)
  {
    processValues();

    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Collect enough data to process!");

    return true;
  }
  else
    return false;
}

BT::NodeStatus Turn_inside::onRunning() 
{
  if (!collect_success_)
    collect_success_ = tryToCollectData();

  if (!collect_success_)
    return BT::NodeStatus::RUNNING;

  if (dry_run_)
  {
    if (!turning_task_finished_)
    {
      if (!is_robot_stop_)
      {
        stopRobot();
        is_robot_stop_ = true;

        before_turning_yaw_ = computeRobotYaw();
        turn_angle_ = 90.0;
      }

      updateGoalPose(turn_angle_);
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

        before_turning_yaw_ = computeRobotYaw();
        turn_angle_ = (narrow == "left") ? 90.0 : -90.0;
      }
      else
        updateGoalPose(turn_angle_);
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

  narrow = has_no_detection ? "No_detection" : names_.back();

  length = calculateAverage(positions_);
  angle = calculateAverage(velocities_);
  coef = calculateAverage(efforts_);
}

double Turn_inside::calculateAverage(const std::vector<double>& values)
{
  double sum = 0.0;

  for (const auto& value : values)
    sum += value;
  
  return sum / values.size();  
}

void Turn_inside::clearData() {
  narrow = "No_detection";
  length = 0.0;
  angle = 0.0;
  coef = 0.0;
  names_.clear();
  positions_.clear();
  velocities_.clear();
  efforts_.clear();
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
  publisher_turn->publish(twist_msg);

  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot in stop process!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_time_point_).count() / 1000.0;

  if (dt > 4.0)
    return true;
  else
    return false;
}

double Turn_inside::computeRobotYaw()
{
  tf2::Quaternion pose_q(orientationx, orientationy, orientationz, orientationw);
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);

  return pose_yaw;
}

void Turn_inside::updateGoalPose(double turn_angle)
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

  double current_robot_yaw = computeRobotYaw();
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Current robot yaw = %f", current_robot_yaw);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Befor turning robot yaw = %f", before_turning_yaw_);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Turning yaw = %f deg", turn_angle);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) After turning robot yaw = %f", before_turning_yaw_ + (turn_angle * M_PI) / 180);

  if ((turn_angle > 0 && before_turning_yaw_ + (turn_angle * M_PI) / 180 > current_robot_yaw) ||
      (turn_angle < 0 && before_turning_yaw_ + (turn_angle * M_PI) / 180 < current_robot_yaw))
  {
    publisher_turn->publish(twist_msg);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Succesfully turned robot!");
    is_robot_stop_ = false;
    turning_task_finished_ = true;
    stop_fire_once_ = false;
  }
}