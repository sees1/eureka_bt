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
  stop_fire_once_(false),
  rotate_fire_once_(false)
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

  if (node_->has_parameter("dummy_rotate_duration"))
    dummy_rotation_dur_ = node_->get_parameter("dummy_rotate_duration").as_double();
  else
    dummy_rotation_dur_ = node_->declare_parameter("dummy_rotate_duration", 4.0);

  if (node_->has_parameter("too_far_distance"))
    too_far_length_ = node_->get_parameter("too_far_distance").as_double();
  else
    too_far_length_ = node_->declare_parameter("too_far_distance", 4.0);

  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Set dummy_rotate_duration = %f", dummy_rotation_dur_);

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

      // collect only none and arrow:left/arrow:right
      if (!dry_run_)
      {
        std::vector<int> v_idx;
        for(size_t idx = 0; idx < msg->name.size(); ++idx)
        {
          if (msg->name[idx] != "cone:none")
            v_idx.push_back(idx);
        }

        double max = 0;
        size_t max_idx = 0;
        for(auto idx : v_idx)
        {
          if (msg->effort[idx] > max)
          {
            max = msg->effort[idx];
            max_idx = idx;
          }
        }

        if (names_.size() == buffer_size_)
        {
          names_.pop_front();
          positions_.pop_front();
          velocities_.pop_front();
          efforts_.pop_front();
        }

        // if v_idx.size == 0 than we can't find any none or arrow detection, 
        // only cone so don't add it
        if (v_idx.size())
        {
          names_.push_back(msg->name[max_idx]);
          positions_.push_back(msg->position[max_idx]);
          velocities_.push_back(msg->velocity[max_idx]);
          efforts_.push_back(msg->effort[max_idx]);
        }
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

        turn_narrow_ = "arrow:left";
        return BT::NodeStatus::RUNNING;
      }

      if (!rotate_fire_once_)
      {
        rotate_time_point_ = std::chrono::steady_clock::now();
        rotate_fire_once_ = true;
      }

      // dummy rotate 4 seconds -_-
      if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rotate_time_point_).count() / 1000.0 < dummy_rotation_dur_)
        updateRotation(turn_narrow_);
      else
      {
        turn_narrow_ = "none";
        is_robot_stop_ = false;
        turning_task_finished_ = true;
        stop_fire_once_ = false;
        rotate_fire_once_ = false;
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot sucessfully rotated on place!");
      return BT::NodeStatus::SUCCESS;
    }
  }
  else
  {
    if (!turning_task_finished_ && names_.size() == buffer_size_)
    {
      processValues();

      if (!is_robot_stop_)
      {
        is_robot_stop_ = stopRobot();

        turn_narrow_ = narrow_;

        return BT::NodeStatus::RUNNING;
      }

      if (!rotate_fire_once_)
      {
        rotate_time_point_ = std::chrono::steady_clock::now();
        rotate_fire_once_ = true;
      }

      if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rotate_time_point_).count() / 1000.0 < dummy_rotation_dur_ ||
          narrow_ == "none" ||
          length_ > too_far_length_)
        updateRotation(turn_narrow_);
      else
      {
        RCLCPP_INFO(node_->get_logger(), "(Turn_inside)(Temp) narrow_ = %s, lenght_ = %f!", narrow_.c_str(), length_);
        turn_narrow_ = "none";
        is_robot_stop_ = false;
        turning_task_finished_ = true;
        stop_fire_once_ = false;
        rotate_fire_once_ = false;
      }
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
      return name == "none";
    }
  );

  narrow_ = has_no_detection ? "none" : names_.back();

  length_ = calculateAverage(positions_);
  angle_ = calculateAverage(velocities_);
  coef_ = calculateAverage(efforts_);
}

double Turn_inside::calculateAverage(const std::deque<double>& values)
{
  double sum = 0.0;

  for (const auto& value : values)
    sum += value;
  
  return sum / values.size();  
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

void Turn_inside::updateRotation(std::string& turn_arrow)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 100.0;

  if (turn_arrow == "arrow:left")
  {
    twist_msg.linear.x = 0.2;
  }
  else if (turn_arrow == "arrow:right")
  {
    twist_msg.linear.x = -0.2;
  }
  else if (turn_arrow == "none")
  {
    twist_msg.linear.x = -0.2;
    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) unknow direction but turn right!");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) unknow direction and unmove direction!");
  }

  turn_pub_->publish(twist_msg);
}

// static member's

double Turn_inside::computeRobotYaw(tf2::Quaternion& pose_q)
{
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);

  return pose_yaw;
}
