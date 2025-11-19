#include "eureka_bt/goal_pose.hpp"

Goalpose::Goalpose(const std::string& name,
                   const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  already_published_(false),
  pose_x_(0.0),
  pose_y_(0.0),
  quat_w_(0.0),
  quat_x_(0.0),
  quat_y_(0.0),
  quat_z_(0.0)
{
  if (node_->has_parameter("full_info"))
    full_info_ = node_->get_parameter("full_info").as_bool();
  else
    full_info_ = node_->declare_parameter("full_info", true);
  
  if (node_->has_parameter("lenght_error"))
    lenght_error_ = node_->get_parameter("lenght_error").as_double();
  else
    lenght_error_ = node_->declare_parameter("lenght_error", -1.6);

  if (node_->has_parameter("buffer_size"))
    buffer_size_ = static_cast<size_t>(node_->get_parameter("buffer_size").as_int());
  else
    buffer_size_ = static_cast<size_t>(node_->declare_parameter("buffer_size", 10));
  
  if (node_->has_parameter("dry_run"))
    dry_run_ = node_->get_parameter("dry_run").as_bool();
  else
    dry_run_ = node_->declare_parameter("dry_run", false);

  if (node_->has_parameter("odometry_topic_name"))
    odometry_topic_name_ = node_->get_parameter("odometry_topic_name").as_string();
  else
    odometry_topic_name_ = node_->declare_parameter("odometry_topic_name", "/odometry");

  if (node_->has_parameter("too_far_distance"))
    too_far_length_ = node_->get_parameter("too_far_distance").as_double();
  else
    too_far_length_ = node_->declare_parameter("too_far_distance", 4.0);

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set too_far_length = %f", too_far_length_);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set length_error = %f", lenght_error_);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set buffer_size = %d", static_cast<int>(buffer_size_));
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set odometry_topic_name = %s", odometry_topic_name_.c_str());
  
  navigator_.waitUntilNav2Active();

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      pose_x_ = msg->pose.pose.position.x;
      pose_y_ = msg->pose.pose.position.y;
      quat_w_ = msg->pose.pose.orientation.w;
      quat_x_ = msg->pose.pose.orientation.x;
      quat_y_ = msg->pose.pose.orientation.y;
      quat_z_ = msg->pose.pose.orientation.z;
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
        // only cone so add none
        if (v_idx.size())
        {
          names_.push_back(msg->name[max_idx]);
          positions_.push_back(msg->position[max_idx]);
          velocities_.push_back(msg->velocity[max_idx]);
          efforts_.push_back(msg->effort[max_idx]);
        }
        else
        {
          names_.push_back("none");
          positions_.push_back(0.0);
          velocities_.push_back(0.0);
          efforts_.push_back(0.0);
        }
      }
    }
  );

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  turn_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::PortsList Goalpose::providedPorts() 
{
  return BT::PortsList();
}

BT::NodeStatus Goalpose::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Goalpose::onRunning()
{
  std::lock_guard<std::mutex> lc(mut_);

  if (!dry_run_)
  {
    if (names_.size() == buffer_size_)
    {
      processValues();

      RCLCPP_INFO(node_->get_logger(), "(Goalpose) Collecting length = %f, arrow direction = %s, coef = %f, angle = %f, after process values!", length_, narrow_.c_str(), coef_, angle_);

      if (!is_robot_stop_)
      {
        is_robot_stop_ = stopRobot();

        return BT::NodeStatus::RUNNING;
      }

      if (already_published_)
      {
        if (isRobotNearGoal())
        {
          if (waitNav())
          {
            wait_nav_fire_once_ = false;
            is_robot_stop_ = false;
            stop_fire_once_ = false;
            already_published_ = false; // manual swap because current type of bt node didn't create after returning SUCCESS or FAILUREc
            return BT::NodeStatus::SUCCESS;
          }
        }
      }
      
      if (length_ < too_far_length_ && length_ > 1.8 && narrow_ != "none" && already_published_ == false)
      {
        publishGoalPose(length_, angle_);
        already_published_ = true;
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "(Goalpose) Can't collect enough data to start. Current candidate set size = %d", (int)names_.size());
      return BT::NodeStatus::RUNNING;
    }
  }
  else
  {
    if (already_published_)
    {
      if (isRobotNearGoal())
        return BT::NodeStatus::SUCCESS;
    }

    if (already_published_ == false)
    {
      publishGoalPose(2.0, 0.4);
      already_published_ = true;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void Goalpose::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(Goalpose) current node is halted!");
}

void Goalpose::processValues()
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

double Goalpose::calculateAverage(const std::deque<double>& values)
{
  double sum = 0.0;

  for (const auto& value : values)
    sum += value;
  
  return sum / values.size();  
}

void Goalpose::publishGoalPose(double length, double angle) 
{
  geometry_msgs::msg::PoseStamped goal;

  goal.header.stamp = node_->now();
  goal.header.frame_id = "map"; 

  double yaw = atan2(2.0 * (quat_w_ * quat_z_ + quat_x_ * quat_y_), 1.0 - 2.0 * (quat_y_ * quat_y_ + quat_z_ * quat_z_));
  // lenght in robot frame
  double local_goal_x = (length + lenght_error_) * cos((-angle * M_PI) / 180);
  double local_goal_y = (length + lenght_error_) * sin((-angle * M_PI) / 180) + body_width / 2.0f; // because lenght is dist between cam and arrow
  
  // goal = lenght in global_frame + robot_pose(in global_frame)
  current_goal_x_ = pose_x_ + (local_goal_x * cos(yaw) - local_goal_y * sin(yaw));
  current_goal_y_ = pose_y_ + (local_goal_x * sin(yaw) + local_goal_y * cos(yaw));

  goal.pose.position.x = current_goal_x_;
  goal.pose.position.y = current_goal_y_;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.x = quat_x_;
  goal.pose.orientation.y = quat_y_;
  goal.pose.orientation.z = quat_z_;
  goal.pose.orientation.w = quat_w_;

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Create and publish goal (x = %f, y = %f, path_yaw = %f)!", current_goal_x_, current_goal_y_, yaw);

  navigator_.goToPose(goal);
}

bool Goalpose::isRobotNearGoal()
{
  // bool fl = (std::hypot(std::abs(pose_x_ - current_goal_x_), std::abs(pose_y_ - current_goal_y_)) < 0.5);

  // if (fl)
  //   RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is sucessfully achive goal! Move to next stage!");

  // if (!fl && full_info_)
  // {
  //   RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot try to move to goal!");
  //   RCLCPP_INFO(node_->get_logger(), "(Goalpose) Current robot pose (x = %f, y = %f)", pose_x_, pose_y_);
  // }

  

  return fl;
}

bool Goalpose::waitNav()
{
  if (!wait_nav_fire_once_)
  {
    wait_nav_point_ = std::chrono::steady_clock::now();
    wait_nav_fire_once_ = true;
  }

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) wait until nav2 finish path");

  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wait_nav_point_).count() / 1000.0;

  if (dt > 3.0)
    return true;
  else
    return false;
}

bool Goalpose::stopRobot()
{
  if (!stop_fire_once_)
  {
    stop_time_point_ = std::chrono::steady_clock::now();
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

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot in stop process!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - stop_time_point_).count() / 1000.0;

  if (dt > 4.0)
    return true;
  else
    return false;
}
