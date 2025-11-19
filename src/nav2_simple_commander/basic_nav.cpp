#include "nav2_simple_commander/basic_nav.hpp"

#include <thread>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace eureka_bt
{

BasicNavigator::BasicNavigator(const std::string & node_name, const std::string & namespace_) :
  Node(node_name, namespace_)
{
  // Action clients
  nav_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
    this, "navigate_through_poses");
  nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");
  follow_waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    this, "follow_waypoints");
  follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
    this, "follow_path");
  compute_path_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    this, "compute_path_to_pose");
  compute_path_through_poses_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathThroughPoses>(
    this, "compute_path_through_poses");
  smoother_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(
    this, "smooth_path");
  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "spin");
  backup_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(this, "backup");
  assisted_teleop_client_ = rclcpp_action::create_client<nav2_msgs::action::AssistedTeleop>(this, "assisted_teleop");

  // Service clients
  change_maps_srv_ = this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
  clear_costmap_global_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "global_costmap/clear_entirely_global_costmap");
  clear_costmap_local_srv_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "local_costmap/clear_entirely_local_costmap");
  get_costmap_global_srv_ = this->create_client<nav2_msgs::srv::GetCostmap>("global_costmap/get_costmap");
  get_costmap_local_srv_ = this->create_client<nav2_msgs::srv::GetCostmap>("local_costmap/get_costmap");
}

// ----------------------- Pose / nav API -----------------------

std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::WrappedResult> BasicNavigator::goThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> & poses,
                                    const std::string & behavior_tree)
{
  waitForActionServer<nav2_msgs::action::NavigateThroughPoses>(nav_through_poses_client_, "NavigateThroughPoses");

  nav2_msgs::action::NavigateThroughPoses::Goal goal_msg;
  goal_msg.poses = poses;
  goal_msg.behavior_tree = behavior_tree;

  RCLCPP_INFO(this->get_logger(), "Navigating with %zu goals....", poses.size());

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::NavigateThroughPoses>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = nav_through_poses_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  nav_through_poses_goal_handle_ = send_goal_future.get();
  if (!nav_through_poses_goal_handle_ || 
       nav_through_poses_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
    return {};
  }

  // store result future
  return nav_through_poses_client_->async_get_result(nav_through_poses_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::WrappedResult> BasicNavigator::goToPose(const geometry_msgs::msg::PoseStamped & pose, const std::string & behavior_tree)
{
  waitForActionServer<nav2_msgs::action::NavigateToPose>(nav_to_pose_client_, "NavigateToPose");

  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose = pose;
  goal_msg.behavior_tree = behavior_tree;

  RCLCPP_INFO(this->get_logger(), "Navigating to goal: %.2f %.2f...",
              pose.pose.position.x, pose.pose.position.y);

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::NavigateToPose>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  nav_to_pose_goal_handle_ = send_goal_future.get();
  if (!nav_to_pose_goal_handle_ ||
       nav_to_pose_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
    return {};
  }

  return nav_to_pose_client_->async_get_result(nav_to_pose_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::WrappedResult> BasicNavigator::followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  waitForActionServer<nav2_msgs::action::FollowWaypoints>(follow_waypoints_client_, "FollowWaypoints");

  nav2_msgs::action::FollowWaypoints::Goal goal_msg;
  goal_msg.poses = poses;

  RCLCPP_INFO(this->get_logger(), "Following %zu waypoints....", poses.size());

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::FollowWaypoints>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = follow_waypoints_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  follow_waypoints_goal_handle_ = send_goal_future.get();
  if (!follow_waypoints_goal_handle_ || 
       follow_waypoints_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected!");
    return {};
  }

  return follow_waypoints_client_->async_get_result(follow_waypoints_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::FollowPath>::WrappedResult> BasicNavigator::followPath(const nav_msgs::msg::Path& path,
                                const std::string& controller_id,
                                const std::string& goal_checker_id)
{
  waitForActionServer<nav2_msgs::action::FollowPath>(follow_path_client_, "FollowPath");

  nav2_msgs::action::FollowPath::Goal goal_msg;
  goal_msg.path = path;
  goal_msg.controller_id = controller_id;
  goal_msg.goal_checker_id = goal_checker_id;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::FollowPath>, this, std::placeholders::_1, std::placeholders::_2);


  auto future_handle = follow_path_client_->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(shared_from_this(), future_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_ERROR(get_logger(), "Failed to send FollowPath goal");
      return {};
  }

  follow_path_goal_handle_ = future_handle.get();
  if (!follow_path_goal_handle_ || 
       follow_path_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "FollowPath goal rejected!");
      return {};
  }

  return follow_path_client_->async_get_result(follow_path_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::Spin>::WrappedResult> BasicNavigator::spin(double spin_dist, int time_allowance)
{
  waitForActionServer<nav2_msgs::action::Spin>(spin_client_, "Spin");

  nav2_msgs::action::Spin::Goal goal_msg;
  goal_msg.target_yaw = spin_dist;
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  RCLCPP_INFO(this->get_logger(), "Spinning to angle %.2f...", spin_dist);

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::Spin>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = spin_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  spin_goal_handle_ = send_goal_future.get();
  if (!spin_goal_handle_ || 
       spin_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Spin request was rejected!");
    return {};
  }

  return spin_client_->async_get_result(spin_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::BackUp>::WrappedResult> BasicNavigator::backup(double backup_dist, double backup_speed, int time_allowance)
{
  waitForActionServer<nav2_msgs::action::BackUp>(backup_client_, "BackUp");

  nav2_msgs::action::BackUp::Goal goal_msg;
  goal_msg.target.x = static_cast<float>(backup_dist);
  goal_msg.speed = backup_speed;
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  RCLCPP_INFO(this->get_logger(), "Backing up %.2f m at %.3f m/s....", backup_dist, backup_speed);

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::BackUp>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = backup_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  backup_goal_handle_ = send_goal_future.get();
  if (!backup_goal_handle_ || 
       backup_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Backup request was rejected!");
    return {};
  }

  return backup_client_->async_get_result(backup_goal_handle_);
}

std::shared_future<rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::WrappedResult> BasicNavigator::assistedTeleop(int time_allowance)
{
  waitForActionServer<nav2_msgs::action::AssistedTeleop>(assisted_teleop_client_, "AssistedTeleop");

  nav2_msgs::action::AssistedTeleop::Goal goal_msg;
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(time_allowance);

  RCLCPP_INFO(this->get_logger(), "Running assisted teleop....");

  auto send_goal_opts = rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::SendGoalOptions();
  send_goal_opts.feedback_callback =
    std::bind(&BasicNavigator::_feedbackCallback<nav2_msgs::action::AssistedTeleop>, this, std::placeholders::_1, std::placeholders::_2);

  auto send_goal_future = assisted_teleop_client_->async_send_goal(goal_msg, send_goal_opts);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  assisted_teleop_goal_handle_ = send_goal_future.get();
  if (!assisted_teleop_goal_handle_ || 
       assisted_teleop_goal_handle_->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Assisted teleop request was rejected!");
    return {};
  }

  return assisted_teleop_client_->async_get_result(assisted_teleop_goal_handle_);
}

// ----------------------- Path utilities -----------------------

nav_msgs::msg::Path BasicNavigator::getPath(const geometry_msgs::msg::PoseStamped & start,
                                            const geometry_msgs::msg::PoseStamped & goal,
                                            const std::string & planner_id,
                                            bool use_start)
{
  waitForActionServer<nav2_msgs::action::ComputePathToPose>(compute_path_to_pose_client_, "ComputePathToPose");

  nav2_msgs::action::ComputePathToPose::Goal goal_msg;
  goal_msg.start = start;
  goal_msg.goal = goal;
  goal_msg.planner_id = planner_id;
  goal_msg.use_start = use_start;

  RCLCPP_INFO(this->get_logger(), "Getting path...");
  auto send_goal_future = compute_path_to_pose_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle || 
       goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Get path was rejected!");
    return {};
  }

  auto result_future = compute_path_to_pose_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "get_result failed");
    return {};
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "Getting path failed with result code: %d", static_cast<int>(result.code));
    return {};
  }
  return result.result->path;
}

nav_msgs::msg::Path BasicNavigator::getPathThroughPoses(const geometry_msgs::msg::PoseStamped & start,
                                                        const std::vector<geometry_msgs::msg::PoseStamped> & goals,
                                                        const std::string & planner_id,
                                                        bool use_start)
{
  waitForActionServer<nav2_msgs::action::ComputePathThroughPoses>(compute_path_through_poses_client_, "ComputePathThroughPoses");

  nav2_msgs::action::ComputePathThroughPoses::Goal goal_msg;
  goal_msg.start = start;
  goal_msg.goals = goals;
  goal_msg.planner_id = planner_id;
  goal_msg.use_start = use_start;

  RCLCPP_INFO(this->get_logger(), "Getting path through poses...");
  auto send_goal_future = compute_path_through_poses_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle || 
       goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Get path was rejected!");
    return {};
  }

  auto result_future = compute_path_through_poses_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "get_result failed");
    return {};
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(this->get_logger(), "Getting path failed with result code: %d", static_cast<int>(result.code));
    return {};
  }
  return result.result->path;
}

nav2_msgs::action::SmoothPath::Result BasicNavigator::smoothPath(const nav_msgs::msg::Path & path,
                                                                 const std::string & smoother_id,
                                                                 double max_duration,
                                                                 bool check_for_collision)
{
  waitForActionServer<nav2_msgs::action::SmoothPath>(smoother_client_, "SmoothPath");

  nav2_msgs::action::SmoothPath::Goal goal_msg;
  goal_msg.path = path;
  goal_msg.smoother_id = smoother_id;
  goal_msg.max_smoothing_duration = rclcpp::Duration::from_seconds(max_duration);
  goal_msg.check_for_collisions = check_for_collision;

  RCLCPP_INFO(this->get_logger(), "Smoothing path...");
  auto send_goal_future = smoother_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "send_goal failed");
    return {};
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle ||
       goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR(this->get_logger(), "Smooth path was rejected!");
    return {};
  }

  auto result_future = smoother_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "get_result failed");
    return {};
  }

  auto result = result_future.get();
  return *result.result;
}

// ----------------------- Misc ------------------------------------------

// template<typename ActionT>
// void BasicNavigator::cancelTask(rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::SharedFuture)
// {
//     RCLCPP_INFO(this->get_logger(), "Canceling current task.");

//     if (goal_handle_) {
//         auto cancel_future = goal_handle_->async_cancel_goal();

//         // Ждём завершения
//         if (rclcpp::spin_until_future_complete(
//                 shared_from_this(), cancel_future) 
//             != rclcpp::FutureReturnCode::SUCCESS)
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to cancel goal");
//         }
//     }
// }

// TaskResult BasicNavigator::getResult()
// {
//     switch (status_) {
//         case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
//             return TaskResult::SUCCEEDED;

//         case action_msgs::msg::GoalStatus::STATUS_ABORTED:
//             return TaskResult::FAILED;

//         case action_msgs::msg::GoalStatus::STATUS_CANCELED:
//             return TaskResult::CANCELED;

//         default:
//             return TaskResult::UNKNOWN;
//     }
// }

// geometry_msgs::msg::PoseWithCovarianceStamped BasicNavigator::getFeedback()
// {
//     return feedback_;
// }

// ----------------------- Costmap / map utilities -----------------------

void BasicNavigator::clearLocalCostmap()
{
  waitForService<nav2_msgs::srv::ClearEntireCostmap>(clear_costmap_local_srv_, "clear local costmap");
  auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto f = clear_costmap_local_srv_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
}

void BasicNavigator::clearGlobalCostmap()
{
  waitForService<nav2_msgs::srv::ClearEntireCostmap>(clear_costmap_global_srv_, "clear global costmap");
  auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto f = clear_costmap_global_srv_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
}

void BasicNavigator::clearAllCostmaps()
{
  clearLocalCostmap();
  clearGlobalCostmap();
}

nav2_msgs::srv::GetCostmap::Response BasicNavigator::getGlobalCostmap()
{
  waitForService<nav2_msgs::srv::GetCostmap>(get_costmap_global_srv_, "get global costmap");
  auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto f = get_costmap_global_srv_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
  return *f.get();
}

nav2_msgs::srv::GetCostmap::Response BasicNavigator::getLocalCostmap()
{
  waitForService<nav2_msgs::srv::GetCostmap>(get_costmap_local_srv_, "get local costmap");
  auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto f = get_costmap_local_srv_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
  return *f.get();
}

void BasicNavigator::changeMap(const std::string & map_url)
{
  waitForService<nav2_msgs::srv::LoadMap>(change_maps_srv_, "load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_url;
  auto f = change_maps_srv_->async_send_request(req);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
  auto res = f.get();
  if (res->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Change map request failed!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Change map request was successful!");
  }
}

// ----------------------- Lifecycle -----------------------

void BasicNavigator::lifecycleStartup()
{
  RCLCPP_INFO(this->get_logger(), "Starting up lifecycle nodes based on lifecycle_manager.");
  for (auto & srv : this->get_service_names_and_types()) {
    if (!srv.second.empty() && srv.second[0] == "nav2_msgs/srv/ManageLifecycleNodes") {
      RCLCPP_INFO(this->get_logger(), "Starting %s", srv.first.c_str());
      auto mgr_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(srv.first);
      waitForService<nav2_msgs::srv::ManageLifecycleNodes>(mgr_client, srv.first);
      auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
      auto f = mgr_client->async_send_request(req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Nav2 is ready for use!");
}

void BasicNavigator::lifecycleShutdown()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down lifecycle nodes based on lifecycle_manager.");
  for (auto & srv : this->get_service_names_and_types()) {
    if (!srv.second.empty() && srv.second[0] == "nav2_msgs/srv/ManageLifecycleNodes") {
      RCLCPP_INFO(this->get_logger(), "Shutting down %s", srv.first.c_str());
      auto mgr_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(srv.first);
      waitForService<nav2_msgs::srv::ManageLifecycleNodes>(mgr_client, srv.first);
      auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
      req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN;
      auto f = mgr_client->async_send_request(req);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
    }
  }
}

void BasicNavigator::waitUntilNav2Active(const std::string & navigator)
{
  _waitForNodeToActivate(navigator);
  RCLCPP_INFO(this->get_logger(), "Nav2 is ready for use!");
}

// ----------------------- Internal helpers -----------------------

template<class ActionT>
void BasicNavigator::_feedbackCallback(typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                                       const std::shared_ptr<const typename ActionT::Feedback> feedback)
{
  (void)feedback;
  RCLCPP_DEBUG(this->get_logger(), "Received action feedback message");
}

void BasicNavigator::_waitForNodeToActivate(const std::string & node_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Waiting for %s to become active..", node_name.c_str());
  std::string node_service = node_name + "/get_state";
  auto state_client = this->create_client<lifecycle_msgs::srv::GetState>(node_service);
  while (!state_client->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting...", node_service.c_str());
  }

  auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  std::string state = "unknown";
  while (state != "active") {
    auto f = state_client->async_send_request(req);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), f);
    if (f.valid() && f.get()) {
      state = f.get()->current_state.label;
      RCLCPP_DEBUG(this->get_logger(), "Result of get_state: %s", state.c_str());
    }
    std::this_thread::sleep_for(2s);
  }
}

template<typename ActionT>
void BasicNavigator::waitForActionServer(const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
                                         const std::string & name)
{
  while (!client->wait_for_action_server(1s)) {
    RCLCPP_INFO(this->get_logger(), "%s action server not available, waiting...", name.c_str());
  }
}

template<typename SrvT>
void BasicNavigator::waitForService(const typename rclcpp::Client<SrvT>::SharedPtr & client, const std::string & name)
{
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(this->get_logger(), "%s service not available, waiting...", name.c_str());
  }
}

} // namespace eureka_bt