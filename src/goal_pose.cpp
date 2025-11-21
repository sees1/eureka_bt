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

  arrow_acc_ = std::make_shared<FalsePositiveFilter<Arrow>>(buffer_size_, allow_length_error, allow_angle_error);
  cone_acc_ = std::make_shared<FalsePositiveFilter<Cone>>(buffer_size_, allow_length_error, allow_angle_error);

  navigator_->waitUntilNav2Active();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  arrow_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lc(mut_);

      // collect only none and arrow:left/arrow:right
      std::vector<int> arrow_idx;
      std::vector<int> cone_idx;
      for(size_t idx = 0; idx < msg->name.size(); ++idx)
      {
        if (msg->name[idx] != "cone:none")
          arrow_idx.push_back(idx);

        if (msg->name[idx] != "arrow:right" &&
            msg->name[idx] != "arrow:left")
          cone_idx.push_back(idx);
      }

      double max = 0.0;
      size_t arrow_max_idx = 0;
      for(auto idx : arrow_idx)
      {
        if (msg->effort[idx] > max)
        {
          max = msg->effort[idx];
          arrow_max_idx = idx;
        }
      }

      max = 0.0;
      size_t cone_max_idx = 0;
      for(auto idx : cone_idx)
      {
        if (msg->effort[idx] > max)
        {
          max = msg->effort[idx];
          cone_max_idx = idx;
        }
      }

      // if v_idx.size == 0 than we can't find any none or arrow detection, 
      // only cone so add none
      if (arrow_idx.size())
        arrow_acc_->addObject(Arrow{msg->name[arrow_max_idx], msg->position[arrow_max_idx], msg->velocity[arrow_max_idx]});
      else
        arrow_acc_->addObject(Arrow{"none", 0.0, 0.0});

      if (cone_idx.size())
        cone_acc_->addObject(Cone{msg->name[cone_max_idx], msg->position[cone_max_idx], msg->velocity[cone_max_idx]});
      else
        cone_acc_->addObject(Cone{"none", 0.0, 0.0});
    }
  );

  pose_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), 
    [this]()
    {
      try
      {
        current_robot_transform_ = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      } 
      catch (tf2::TransformException &ex)
      {
        RCLCPP_WARN(node_->get_logger(), "tf lookup from %s to %s failed: %s", "base_link", "map", ex.what());
      }
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
              if (std::hypot(std::abs(current_goal_.pose.position.x - current_robot_transform_.transform.translation.x),
                              std::abs(current_goal_.pose.position.y - current_robot_transform_.transform.translation.y)) < enough_close_to_republish_)
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
          if (current_cone_.direction != "none")
          {
            RCLCPP_INFO(node_->get_logger(), "(Goalpose) Robot is stoped aroun cone!");
            std::exit(1);
          }
          else
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

    if (current_cone_.distance < too_far_length_ &&
        current_cone_.distance > 1.8             &&
        current_cone_.direction != "none"        &&
        already_published_ == false)
    {
      publishGoalPose(current_cone_.distance, current_cone_.angle);
      already_published_ = true;
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
  if (!arrow_acc_->isBufferFull() && !cone_acc_->isBufferFull())
    return false;

  current_arrow_ = arrow_acc_->getActualObject();
  current_cone_ = cone_acc_->getActualObject();

  return true;
}

void Goalpose::publishGoalPose(double length, double angle) 
{
  current_goal_.header.stamp = node_->now();
  current_goal_.header.frame_id = "map"; 
  
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.stamp = current_goal_.header.stamp;
  robot_pose.header.frame_id = "map";
  robot_pose.pose.position.x = current_robot_transform_.transform.translation.x;
  robot_pose.pose.position.y = current_robot_transform_.transform.translation.y;
  robot_pose.pose.position.z = current_robot_transform_.transform.translation.z;
  robot_pose.pose.orientation.x = current_robot_transform_.transform.rotation.x;
  robot_pose.pose.orientation.y = current_robot_transform_.transform.rotation.y;
  robot_pose.pose.orientation.z = current_robot_transform_.transform.rotation.z;
  robot_pose.pose.orientation.w = current_robot_transform_.transform.rotation.w;
  
  double robot_roll, robot_pitch, robot_yaw;
  std::tie(robot_roll, robot_pitch, robot_yaw) = computeRobotAngles();
  
  double robot_yaw_sin = std::sin(robot_yaw);
  double robot_yaw_cos = std::cos(robot_yaw);
  
  tf2::Quaternion robot_goal_pose;
  robot_goal_pose.setRPY(robot_roll, robot_pitch, normalizeAngle(robot_yaw + ((angle * M_PI) / 180.0)));
  
  double before_length = length;
  
  length += length_error_delta_;
  
  // lenght in robot frame
  double local_goal_x = (length) * cos((-angle * M_PI) / 180.0);
  double local_goal_y = (length) * sin((-angle * M_PI) / 180.0);
  
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) robot x = %f, robot y = %f, robot_angle = %f!",current_robot_transform_.transform.translation.x, current_robot_transform_.transform.translation.y, robot_yaw);
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) angle = %f!", angle);
  
  // goal = lenght in global_frame + robot_pose(in global_frame)
  current_goal_.pose.position.x = current_robot_transform_.transform.translation.x + (local_goal_x * robot_yaw_cos - local_goal_y * robot_yaw_sin);
  current_goal_.pose.position.y = current_robot_transform_.transform.translation.y + (local_goal_x * robot_yaw_sin + local_goal_y * robot_yaw_cos); // because lenght is dist between cam and arrow
  current_goal_.pose.position.z = 0.0;
  current_goal_.pose.orientation.x = robot_goal_pose.x();
  current_goal_.pose.orientation.y = robot_goal_pose.y();
  current_goal_.pose.orientation.z = robot_goal_pose.z();
  current_goal_.pose.orientation.w = robot_goal_pose.w();

  nav_msgs::msg::Path current_path = navigator_->getPath(robot_pose, current_goal_);
  
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
  
    current_goal_.pose.position.x = current_robot_transform_.transform.translation.x + (local_goal_x * robot_yaw_cos - local_goal_y * robot_yaw_sin);
    current_goal_.pose.position.y = current_robot_transform_.transform.translation.y + (local_goal_x * robot_yaw_sin + local_goal_y * robot_yaw_cos); // because lenght is dist between cam and arrow
    current_goal_.pose.position.z = 0.0;
    current_goal_.pose.orientation.x = robot_goal_pose.x();
    current_goal_.pose.orientation.y = robot_goal_pose.y();
    current_goal_.pose.orientation.z = robot_goal_pose.z();
    current_goal_.pose.orientation.w = robot_goal_pose.w();

    current_path = navigator_->getPath(robot_pose, current_goal_);
  }

  // we shoudn't go too close to arrow, so cut a little
  if (length > 3.0)
    length -= 1.0;
  else if (length > 0.5 && length <= 3.0)
    length -= 0.45;

  local_goal_x = (length) * cos((-angle * M_PI) / 180.0);
  local_goal_y = (length) * sin((-angle * M_PI) / 180.0);

  current_goal_.pose.position.x = current_robot_transform_.transform.translation.x + (local_goal_x * robot_yaw_cos - local_goal_y * robot_yaw_sin);
  current_goal_.pose.position.y = current_robot_transform_.transform.translation.y + (local_goal_x * robot_yaw_sin + local_goal_y * robot_yaw_cos); // because lenght is dist between cam and arrow
  current_goal_.pose.position.z = 0.0;
  current_goal_.pose.orientation.x = robot_goal_pose.x();
  current_goal_.pose.orientation.y = robot_goal_pose.y();
  current_goal_.pose.orientation.z = robot_goal_pose.z();
  current_goal_.pose.orientation.w = robot_goal_pose.w();

  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Create and publish goal (x = %f, y = %f, end_yaw = %f)!", current_goal_.pose.position.x, current_goal_.pose.position.y, robot_yaw + ((angle * M_PI) / 180.0));
  RCLCPP_INFO(node_->get_logger(), "(Goalpose) Length of path cropped from %f to %f!", before_length, length);

  go_to_pose_res_ = navigator_->goToPose(current_goal_);
  start_navigation_time_ = std::chrono::steady_clock::now();
}

std::tuple<double, double, double> Goalpose::computeRobotAngles()
{
  tf2::Quaternion pose_q(current_robot_transform_.transform.rotation.x, 
                         current_robot_transform_.transform.rotation.y,
                         current_robot_transform_.transform.rotation.z,
                         current_robot_transform_.transform.rotation.w);
  tf2::Matrix3x3 pose_m(pose_q);
  double pose_roll, pose_pitch, pose_yaw;
  pose_m.getRPY(pose_roll, pose_pitch, pose_yaw);

  return {pose_roll, pose_pitch, pose_yaw};
}

double Goalpose::normalizeAngle(double angle)
{
  // 1.2 M_PI -> -0.8 M_PI like in navigation 
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
      angle += 2.0 * M_PI;
  return angle - M_PI;
}