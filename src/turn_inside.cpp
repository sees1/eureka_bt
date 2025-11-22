#include "eureka_bt/turn_inside.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

Turn_inside::Turn_inside(const std::string& name,
                         const BT::NodeConfiguration& config,
                         rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  turning_task_finished_(true),
  rotate_fire_once_(false)
{
  if (node_->has_parameter("odometry_topic_name"))
    odometry_topic_name_ = node_->get_parameter("odometry_topic_name").as_string();
  else  
    odometry_topic_name_ = node_->declare_parameter("odometry_topic_name", "/odometry");

  if (node_->has_parameter("buffer_size"))
    buffer_size_ = static_cast<size_t>(node_->get_parameter("buffer_size").as_int());
  else
    buffer_size_ = static_cast<size_t>(node_->declare_parameter("buffer_size", 10));

  if (node_->has_parameter("dummy_rotate_duration"))
    dummy_rotate_duration_temp_ = node_->get_parameter("dummy_rotate_duration").as_double();
  else
    dummy_rotate_duration_temp_ = node_->declare_parameter("dummy_rotate_duration", 4.0);

  if (node_->has_parameter("too_far_distance"))
    too_far_length_ = node_->get_parameter("too_far_distance").as_double();
  else
    too_far_length_ = node_->declare_parameter("too_far_distance", 4.0);

  if (node_->has_parameter("too_big_angle"))
    too_big_angle_ = node_->get_parameter("too_big_angle").as_double();
  else
    too_big_angle_ = node_->declare_parameter("too_big_angle", 5.0);

  double allow_length_error, allow_angle_error;

  if (node_->has_parameter("allow_length_error"))
    allow_length_error = node_->get_parameter("allow_length_error").as_double();
  else
    allow_length_error = node_->declare_parameter("allow_length_error", 2.0);

  if (node_->has_parameter("allow_angle_error"))
    allow_angle_error = node_->get_parameter("allow_angle_error").as_double();
  else
    allow_angle_error = node_->declare_parameter("allow_angle_error", 4.0);

  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Set dummy_rotate_duration = %f", dummy_rotate_duration_temp_);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Set too_big_angle = %f", too_big_angle_);
  RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Set too_far_distance = %f", too_far_length_);

  arrow_acc_ = std::make_shared<FalsePositiveFilter>(buffer_size_, allow_length_error, allow_angle_error);
  cone_acc_ = std::make_shared<FalsePositiveFilter>(buffer_size_, allow_length_error, allow_angle_error);

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
        arrow_acc_->addObject(Object{msg->name[arrow_max_idx], msg->position[arrow_max_idx], msg->velocity[arrow_max_idx]});
      else
        arrow_acc_->addObject(Object{"none", 0.0, 0.0});

      if (cone_idx.size())
        cone_acc_->addObject(Object{msg->name[cone_max_idx], msg->position[cone_max_idx], msg->velocity[cone_max_idx]});
      else
        cone_acc_->addObject(Object{"none", 0.0, 0.0});
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
  turn_direction_ = "none";
  turning_task_finished_ = false;
  rotate_fire_once_ = false;
  dummy_rotation_dur_ = dummy_rotate_duration_temp_;

  return BT::NodeStatus::RUNNING;
} 

BT::NodeStatus Turn_inside::onRunning() 
{
  if (!turning_task_finished_)
  {
    if (processValues())
    {
      if (!rotate_fire_once_)
      {
        turn_direction_ = current_arrow_.direction;
        
        // if we stay left or right than arrow
        if ((turn_direction_ == "arrow:right" && current_arrow_.angle > too_big_angle_) ||
            (turn_direction_ == "arrow:left"  && current_arrow_.angle < -too_big_angle_))
          dummy_rotation_dur_ += 3.0;
        
        rotate_time_point_ = std::chrono::steady_clock::now();
        rotate_fire_once_ = true;
      }

      if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rotate_time_point_).count() / 1000.0 < dummy_rotation_dur_)
      {
        RCLCPP_INFO(node_->get_logger(), "(Turn_inside) dummy rotation!");
        updateRotation(turn_direction_);
        return BT::NodeStatus::RUNNING;
      }

      if (current_cone_.direction == "none")
      {
        if (current_arrow_.direction == "none")
        {
          RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Cone and arrow don't detect, rotate!");
          updateRotation(turn_direction_);
        }
        else if (current_arrow_.direction  != "none" && (current_arrow_.distance > too_far_length_ || std::abs(current_arrow_.angle) > too_big_angle_))
        {
          updateRotation(turn_direction_);
          RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Current arrow bent, rotate! Arrow narrow_ = %s, lenght_ = %f, angle = %f!", current_arrow_.direction.c_str(), current_arrow_.distance, current_arrow_.angle);
        }
        else {
          turning_task_finished_ = true;
          return BT::NodeStatus::RUNNING;
        }
      }
      else if (current_cone_.direction != "none" && (current_cone_.distance > too_far_length_ || std::abs(current_cone_.angle) > too_big_angle_))
      {
        updateRotation(turn_direction_);
        RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Current cone bent, rotate! Cone narrow_ = %s, lenght_ = %f, angle = %f!", current_cone_.direction.c_str(), current_cone_.distance, current_cone_.angle);
      }
      else
      {
        turning_task_finished_ = true;
      }
    }
    else
      return BT::NodeStatus::RUNNING;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "(Turn_inside) Robot sucessfully rotated on place!");
    return BT::NodeStatus::SUCCESS;
  }
    
  return BT::NodeStatus::RUNNING;
}

void Turn_inside::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(Turn_inside) Current node is halted!");
}

bool Turn_inside::processValues()
{
  std::lock_guard<std::mutex> lc(mut_);

  if (!arrow_acc_->isBufferFull() && !cone_acc_->isBufferFull())
    return false;

  current_arrow_ = arrow_acc_->getActualObject();
  current_cone_ = cone_acc_->getActualObject();

  return true;
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
    twist_msg.linear.x = 0.15;
  }
  else if (turn_arrow == "arrow:right")
  {
    twist_msg.linear.x = -0.15;
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
