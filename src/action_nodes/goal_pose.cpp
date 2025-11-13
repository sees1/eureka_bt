#include "eureka_bt/goal_pose.hpp"

Goalpose::Goalpose(const std::string& name,
                   const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  already_published(false),
  posex(0.0),
  posey(0.0),
  orientationw(0.0),
  orientationx(0.0),
  orientationy(0.0),
  orientationz(0.0)
{
  if (node_->has_parameter("full_info"))
    full_info_ = node_->get_parameter("full_info").as_bool();
  else
    full_info_ = node_->declare_parameter("full_info", true);
  
  if (node_->has_parameter("lenght_error"))
    lenght_error_ = node_->get_parameter("lenght_error").as_double();
  else
    lenght_error_ = node_->declare_parameter("lenght_error", -0.2);
  
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
      posex = msg->pose.pose.position.x;
      posey = msg->pose.pose.position.y;
      orientationw = msg->pose.pose.orientation.w;
      orientationx = msg->pose.pose.orientation.x;
      orientationy = msg->pose.pose.orientation.y;
      orientationz = msg->pose.pose.orientation.z;
    }
  );

  arrow_sub = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lc(mut_);
      // if (full_info_)
      // RCLCPP_INFO(node_->get_logger(), "(Goalpose) arrow compute loop called! Current candidate set size = %d", (int)names_.size());

      if (strcmp(msg->name, "none") != 0 && !dry_run_ && names_.size() != 5)
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

  publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
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
    // if (!full_clear)
    // {
    //   full_clear = fullClearing();
    //   return BT::NodeStatus::RUNNING;
    // }

    if (names_.size() == 5)
    {
      processValues();

      RCLCPP_INFO(node_->get_logger(), "(Goalpose) Collecting length = %f, arrow direction = %s, coef = %f, after process values!", length, narrow.c_str(), coef);

      if (already_published)
      {
        if (isRobotNearGoal())
        {
          clearData();
          already_published = false;
          clearing_fire_once_ = false;
          full_clear = false;
          return BT::NodeStatus::SUCCESS;
        }
      }
      
      if (length > 1.8 && narrow != "No_detection" && already_published == false)
      {
        publishGoalPose(length, angle);
        already_published = true;
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
    if (already_published)
    {
      if (isRobotNearGoal())
        return BT::NodeStatus::SUCCESS;
    }

    if (already_published == false)
    {
      publishGoalPose(2.0, 0.4);
      already_published = true;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void Goalpose::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(Goalpose) current node is halted!");
}

bool Goalpose::fullClearing()
{
  if (!clearing_fire_once_)
  {
    last_time_point_ = std::chrono::steady_clock::now();
    clearing_fire_once_ = true;
  }

  clearData();


  RCLCPP_INFO(node_->get_logger(), "(GoalPose) Robot buffer clearing!");
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_time_point_).count() / 1000.0;

  if (dt > 4.0)
    return true;
  else
    return false;
}

void Goalpose::processValues()
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

double Goalpose::calculateAverage(const std::vector<double>& values)
{
  double sum = 0.0;

  for (const auto& value : values)
    sum += value;
  
  return sum / values.size();  
}

void Goalpose::clearData() {
  narrow = "No_detection";
  length = 0.0;
  angle = 0.0;
  coef = 0.0;
  names_.clear();
  positions_.clear();
  velocities_.clear();
  efforts_.clear();
}

void Goalpose::publishGoalPose(double length, double angle) 
{
  geometry_msgs::msg::PoseStamped goalposemsg;

  goalposemsg.header.stamp = node_->now();
  goalposemsg.header.frame_id = "map"; 

  double yaw = atan2(2.0 * (orientationw * orientationz + orientationx * orientationy),
                     1.0 - 2.0 * (orientationy * orientationy + orientationz * orientationz));
  double local_goal_x = (length + lenght_error_);
  double local_goal_y = (length + lenght_error_) * sin((-angle * M_PI) / 180); 
  current_goal_x = posex + (local_goal_x * cos(yaw) - local_goal_y * sin(yaw));
  current_goal_y = posey + (local_goal_x * sin(yaw) + local_goal_y * cos(yaw));

  goalposemsg.pose.position.x = current_goal_x;
  goalposemsg.pose.position.y = current_goal_y;
  goalposemsg.pose.position.z = 0.0;
  goalposemsg.pose.orientation.x = orientationx;
  goalposemsg.pose.orientation.y = orientationy;
  goalposemsg.pose.orientation.z = orientationz;
  goalposemsg.pose.orientation.w = orientationw;

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Create and publish goal (x = %f, y = %f, path_yaw = %f)!", current_goal_x, current_goal_y, yaw);

  publisher->publish(goalposemsg);
}

bool Goalpose::isRobotNearGoal()
{
  bool fl = (std::hypot(std::abs(posex - current_goal_x), std::abs(posey - current_goal_y)) < 0.5);

  if (fl)
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is sucessfully achive goal! Move to next stage!");

  if (!fl && full_info_)
  {
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot try to move to goal!");
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Current robot pose (x = %f, y = %f)", posex, posey);
  }

  return fl;
}