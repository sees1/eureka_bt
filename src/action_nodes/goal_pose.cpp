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
    lenght_error_ = node_->declare_parameter("lenght_error", -0.2);

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
        {
          names_.pop_front();
          positions_.pop_front();
          velocities_.pop_front();
          efforts_.pop_front();
        }

        names_.push_back(msg->name[max_idx]);
        positions_.push_back(msg->position[max_idx]);
        velocities_.push_back(msg->velocity[max_idx]);
        efforts_.push_back(msg->effort[max_idx]);
      }
    }
  );

  goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
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
    if (names_.size() == 5)
    {
      processValues();

      RCLCPP_INFO(node_->get_logger(), "(Goalpose) Collecting length = %f, arrow direction = %s, coef = %f, after process values!", length_, narrow_.c_str(), coef_);

      if (already_published_)
      {
        if (isRobotNearGoal())
        {
          already_published_ = false; // manual swap because current type of bt node didn't create after returning SUCCESS or FAILURE
          return BT::NodeStatus::SUCCESS;
        }
      }
      
      if (length_ > 1.8 && narrow_ != "No_detection" && already_published_ == false)
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
      return name == "No_detection";
    }
  );

  narrow_ = has_no_detection ? "No_detection" : names_.back();

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
  double local_goal_x = (length + lenght_error_);
  double local_goal_y = (length + lenght_error_) * sin((-angle * M_PI) / 180); 
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

  goal_pub_->publish(goal);
}

bool Goalpose::isRobotNearGoal()
{
  bool fl = (std::hypot(std::abs(pose_x_ - current_goal_x_), std::abs(pose_y_ - current_goal_y_)) < 0.5);

  if (fl)
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is sucessfully achive goal! Move to next stage!");

  if (!fl && full_info_)
  {
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot try to move to goal!");
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Current robot pose (x = %f, y = %f)", pose_x_, pose_y_);
  }

  return fl;
}