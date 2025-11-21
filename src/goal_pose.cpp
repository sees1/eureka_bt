#include "eureka_bt/goal_pose.hpp"

Goalpose::Goalpose(const std::string& name,
                   const BT::NodeConfiguration& config,
                   rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  navigator_(std::make_shared<eureka_bt::BasicNavigator>()),
  already_published_(false)
{
  if (node_->has_parameter("full_info"))
    full_info_ = node_->get_parameter("full_info").as_bool();
  else
    full_info_ = node_->declare_parameter("full_info", true);
  
  if (node_->has_parameter("length_error_delta"))
    length_error_delta_ = node_->get_parameter("length_error_delta").as_double();
  else
    length_error_delta_ = node_->declare_parameter("length_error_delta", -0.3);

  if (length_error_delta_ > 0.0)
    RCLCPP_WARN(node_->get_logger(), "(Goalpose) Set length_error_delta greater than zero is unusual! Please read code one more time");

  if (node_->has_parameter("buffer_size"))
    buffer_size_ = static_cast<size_t>(node_->get_parameter("buffer_size").as_int());
  else
    buffer_size_ = static_cast<size_t>(node_->declare_parameter("buffer_size", 10));

  if (node_->has_parameter("navigation_time_limit"))
    navigation_time_limit_ = node_->get_parameter("navigation_time_limit").as_double();
  else
    navigation_time_limit_ = node_->declare_parameter("navigation_time_limit", 60.0);

  if (node_->has_parameter("enough_close_to_republish"))
    enough_close_to_republish_ = node_->get_parameter("enough_close_to_republish").as_double();
  else
    enough_close_to_republish_ = node_->declare_parameter("enough_close_to_republish", 3.0);

  if (node_->has_parameter("odometry_topic_name"))
    odometry_topic_name_ = node_->get_parameter("odometry_topic_name").as_string();
  else
    odometry_topic_name_ = node_->declare_parameter("odometry_topic_name", "/odometry");

  if (node_->has_parameter("too_far_distance"))
    too_far_length_ = node_->get_parameter("too_far_distance").as_double();
  else
    too_far_length_ = node_->declare_parameter("too_far_distance", 4.0);

  double allow_length_error, allow_angle_error;

  if (node_->has_parameter("allow_length_error"))
    allow_length_error = node_->get_parameter("allow_length_error").as_double();
  else
    allow_length_error = node_->declare_parameter("allow_length_error", 2.0);

  if (node_->has_parameter("allow_angle_error"))
    allow_angle_error = node_->get_parameter("allow_angle_error").as_double();
  else
    allow_angle_error = node_->declare_parameter("allow_angle_error", 4.0);

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set too_far_length = %f", too_far_length_);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set length_error_delta = %f", length_error_delta_);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set buffer_size = %d", static_cast<int>(buffer_size_));
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Set odometry_topic_name = %s", odometry_topic_name_.c_str());

  arrow_acc_ = std::make_shared<ArrowFilter>(buffer_size_, allow_length_error, allow_angle_error);
  
  navigator_->waitUntilNav2Active();

  pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_name_, 10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      current_robot_pose_.pose.position.x = msg->pose.pose.position.x;
      current_robot_pose_.pose.position.y = msg->pose.pose.position.y;
      current_robot_pose_.pose.position.z = msg->pose.pose.position.z;
      current_robot_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
      current_robot_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
      current_robot_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
      current_robot_pose_.pose.orientation.w = msg->pose.pose.orientation.w;
    }
  );

  arrow_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lc(mut_);

      // collect only none and arrow:left/arrow:right
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

      // if v_idx.size == 0 than we can't find any none or arrow detection, 
      // only cone so add none
      if (v_idx.size())
        arrow_acc_->addArrow(Arrow{msg->name[max_idx], msg->position[max_idx], msg->velocity[max_idx]});
      else
        arrow_acc_->addArrow(Arrow{"none", 0.0, 0.0});
    }
  );
}

BT::PortsList Goalpose::providedPorts() 
{
  return BT::PortsList();
}

BT::NodeStatus Goalpose::onStart()
{
  republish_once_ = false;
  already_published_ = false; // manual swap because current type of bt node didn't create after returning SUCCESS or FAILUREc
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Goalpose::onRunning()
{
  std::lock_guard<std::mutex> lc(mut_);

  if (processValues())
  {
    // RCLCPP_INFO(node_->get_logger(), "(Goalpose) Collecting length = %f, arrow direction = %s, coef = %f, angle = %f, after process values!", length_, narrow_.c_str(), coef_, angle_);

    if (already_published_)
    {
      rclcpp_action::ResultCode fl = navigator_->isTaskComplete<typename nav2_msgs::action::NavigateToPose>(go_to_pose_res_);

      switch (fl)
      {
        case ResultCode::UNKNOWN:
        {
          float navigation_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_navigation_time_).count();

          if (navigation_elapsed > navigation_time_limit_)
          {
            RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Navigation mission duration is long than required! Quit");
            return BT::NodeStatus::FAILURE;
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot try to move to goal!");

            if (!republish_once_)
            {
              if (std::hypot(std::abs(current_goal_.pose.position.x - current_robot_pose_.pose.position.x),
                              std::abs(current_goal_.pose.position.y - current_robot_pose_.pose.position.y)) < enough_close_to_republish_)
              {
                republish_once_ = true;
                navigator_->cancelTask<typename nav2_msgs::action::NavigateToPose>();

                RCLCPP_INFO(node_->get_logger(), "Republishing while come to arrow closer than %f m", enough_close_to_republish_);
                processValues();
                already_published_ = false;
                break;
              }
            }

            return BT::NodeStatus::RUNNING;
          }
        }
        case ResultCode::ABORTED:
        {
          RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Navigation failed while running! Try to create new goal!");
          already_published_ = false;
          break;
        }
        case ResultCode::SUCCEEDED:
        {
          RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is sucessfully achive goal! Move to next stage!");
          return BT::NodeStatus::SUCCESS;
        }
        case ResultCode::CANCELED:
        {
          RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Navigation is broken! FAILURE will return!");
          return BT::NodeStatus::FAILURE;
        }
      }
    }
    
    if (current_arrow_.distance < too_far_length_ &&
        current_arrow_.distance > 1.8             &&
        current_arrow_.direction != "none"        &&
        already_published_ == false)
    {
      publishGoalPose(current_arrow_.distance, current_arrow_.angle);
      already_published_ = true;
    }
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "(Goalpose) Can't collect enough data to start!");
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void Goalpose::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(Goalpose) current node is halted! Stop nav stack!");

  if (already_published_)
  {
    rclcpp_action::ResultCode fl = navigator_->isTaskComplete<typename nav2_msgs::action::NavigateToPose>(go_to_pose_res_);

    switch (fl)
    {
      case ResultCode::UNKNOWN:
      {
        navigator_->cancelTask<typename nav2_msgs::action::NavigateToPose>();
        break;
      }
      case ResultCode::ABORTED:
      {
        RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Navigation failed while node is halted! Doesn't matter");
        break;
      }
      case ResultCode::SUCCEEDED:
      {
        RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is sucessfully achive goal while node is halted! Happy");
        break;
      }
      case ResultCode::CANCELED:
      {
        RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Navigation is broken while node is halted! Doesn't matter");
        break;
      }
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "(Goalpose) Quit!");
}

bool Goalpose::processValues()
{
  if (!arrow_acc_->isBufferFull())
    return false;

  current_arrow_ = arrow_acc_->getActualArrow();

  return true;
}

void Goalpose::publishGoalPose(double length, double angle) 
{
  current_goal_.header.stamp = node_->now();
  current_goal_.header.frame_id = "map"; 

  double robot_yaw = computeRobotYaw();

  double before_length = length;

  length += length_error_delta_;

  // lenght in robot frame
  double local_goal_x = (length) * cos((-angle * M_PI) / 180.0);
  double local_goal_y = (length) * sin((-angle * M_PI) / 180.0);

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) angle = %f!", angle);
  
  // goal = lenght in global_frame + robot_pose(in global_frame)
  current_goal_.pose.position.x = current_robot_pose_.pose.position.x + (local_goal_x * cos(robot_yaw) - local_goal_y * sin(robot_yaw));
  current_goal_.pose.position.y = current_robot_pose_.pose.position.y + (local_goal_x * sin(robot_yaw) + local_goal_y * cos(robot_yaw)); // because lenght is dist between cam and arrow
  current_goal_.pose.position.z = 0.0;
  current_goal_.pose.orientation.x = current_robot_pose_.pose.orientation.x;
  current_goal_.pose.orientation.y = current_robot_pose_.pose.orientation.y;
  current_goal_.pose.orientation.z = current_robot_pose_.pose.orientation.z;
  current_goal_.pose.orientation.w = current_robot_pose_.pose.orientation.w;

  nav_msgs::msg::Path current_path = navigator_->getPath(current_robot_pose_, current_goal_);
  
  while (current_path.poses.empty())
  {
    if (length < 0.0)
    {
      length = before_length;
      length_error_delta_ += 0.1;
    }
    
    if (length_error_delta_ > 0.0)
      throw std::runtime_error("wtf");

    length += length_error_delta_;

    local_goal_x = (length) * cos((-angle * M_PI) / 180.0);
    local_goal_y = (length) * sin((-angle * M_PI) / 180.0);
  
    current_goal_.pose.position.x = current_robot_pose_.pose.position.x + (local_goal_x * cos(robot_yaw) - local_goal_y * sin(robot_yaw));
    current_goal_.pose.position.y = current_robot_pose_.pose.position.y + (local_goal_x * sin(robot_yaw) + local_goal_y * cos(robot_yaw)); // because lenght is dist between cam and arrow
    current_goal_.pose.position.z = 0.0;
    current_goal_.pose.orientation.x = current_robot_pose_.pose.orientation.x;
    current_goal_.pose.orientation.y = current_robot_pose_.pose.orientation.y;
    current_goal_.pose.orientation.z = current_robot_pose_.pose.orientation.z;
    current_goal_.pose.orientation.w = current_robot_pose_.pose.orientation.w;

    current_path = navigator_->getPath(current_robot_pose_, current_goal_);
  }

  // we shoudn't go too close to arrow, so cut a little
  if (length > 3.0)
    length -= 1.0;
  else if (length > 0.5 && length <= 3.0)
    length -= 0.6;

  local_goal_x = (length) * cos((-angle * M_PI) / 180.0);
  local_goal_y = (length) * sin((-angle * M_PI) / 180.0);

  current_goal_.pose.position.x = current_robot_pose_.pose.position.x + (local_goal_x * cos(robot_yaw) - local_goal_y * sin(robot_yaw));
  current_goal_.pose.position.y = current_robot_pose_.pose.position.y + (local_goal_x * sin(robot_yaw) + local_goal_y * cos(robot_yaw)); // because lenght is dist between cam and arrow
  current_goal_.pose.position.z = 0.0;
  current_goal_.pose.orientation.x = current_robot_pose_.pose.orientation.x;
  current_goal_.pose.orientation.y = current_robot_pose_.pose.orientation.y;
  current_goal_.pose.orientation.z = current_robot_pose_.pose.orientation.z;
  current_goal_.pose.orientation.w = current_robot_pose_.pose.orientation.w;

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Create and publish goal (x = %f, y = %f, end_yaw = %f)!", current_goal_.pose.position.x, current_goal_.pose.position.y, robot_yaw);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Length of path cropped from %f to %f!", before_length, length);

  go_to_pose_res_ = navigator_->goToPose(current_goal_);
  start_navigation_time_ = std::chrono::steady_clock::now();
}

double Goalpose::computeRobotYaw()
{
  tf2::Quaternion pose_q(current_robot_pose_.pose.orientation.x, 
                         current_robot_pose_.pose.orientation.y,
                         current_robot_pose_.pose.orientation.z,
                         current_robot_pose_.pose.orientation.w);
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);

  return pose_yaw;
}