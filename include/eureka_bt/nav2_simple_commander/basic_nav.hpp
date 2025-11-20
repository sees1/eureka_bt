#pragma once

#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "action_msgs/msg/goal_status.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace eureka_bt
{

enum class TaskResult { UNKNOWN = 0, SUCCEEDED = 1, CANCELED = 2, FAILED = 3 };

class BasicNavigator : public rclcpp::Node
{
public:
  explicit BasicNavigator(const std::string & node_name = "basic_navigator",
                          const std::string & namespace_ = "");
  ~BasicNavigator() override = default;

  // Pose / navigation API
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::WrappedResult> goToPose(const geometry_msgs::msg::PoseStamped & pose,
                                                                                  const std::string & behavior_tree = "");
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::WrappedResult> goThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> & poses,
                                                                                              const std::string & behavior_tree = "");
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::WrappedResult> followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> & poses);
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::FollowPath>::WrappedResult> followPath(const nav_msgs::msg::Path & path,
                                                                                const std::string & controller_id = "",
                                                                                const std::string & goal_checker_id = "");
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::Spin>::WrappedResult> spin(double spin_dist = 1.57, int time_allowance = 10);
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::BackUp>::WrappedResult> backup(double backup_dist = 0.15, double backup_speed = 0.025, int time_allowance = 10);
  std::shared_future<rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::WrappedResult> assistedTeleop(int time_allowance = 30);

  // Path utilities
  nav_msgs::msg::Path getPath(const geometry_msgs::msg::PoseStamped & start,
                              const geometry_msgs::msg::PoseStamped & goal,
                              const std::string & planner_id = "",
                              bool use_start = false);
  nav_msgs::msg::Path getPathThroughPoses(const geometry_msgs::msg::PoseStamped & start,
                                          const std::vector<geometry_msgs::msg::PoseStamped> & goals,
                                          const std::string & planner_id = "",
                                          bool use_start = false);
  nav2_msgs::action::SmoothPath::Result smoothPath(const nav_msgs::msg::Path & path,
                                                   const std::string & smoother_id = "",
                                                   double max_duration = 2.0,
                                                   bool check_for_collision = false);

  // Misc

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr getGoalByType()
  {
    if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateToPose>)
      return nav_to_pose_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateThroughPoses>)
      return nav_through_poses_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::FollowWaypoints>)
      return follow_waypoints_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::FollowPath>)
      return follow_path_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::Spin>) 
      return spin_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::BackUp>) 
      return backup_goal_handle_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::AssistedTeleop>) 
      return assisted_teleop_goal_handle_;
    else
      static_assert(!sizeof(ActionT), "Unsupported type");
  }

  template<typename ActionT>
  typename rclcpp_action::Client<ActionT>::SharedPtr getClientByType()
  {
    if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateToPose>)
      return nav_to_pose_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::NavigateThroughPoses>)
      return nav_through_poses_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::FollowWaypoints>)
      return follow_waypoints_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::FollowPath>)
      return follow_path_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::Spin>) 
      return spin_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::BackUp>) 
      return backup_client_;
    else if constexpr (std::is_same_v<ActionT, nav2_msgs::action::AssistedTeleop>) 
      return assisted_teleop_client_;
    else
      static_assert(!sizeof(ActionT), "Unsupported type");
  }

  template<typename ActionT>
  void cancelTask()
  {
    RCLCPP_INFO(this->get_logger(), "Canceling current task.");

    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = 
      getGoalByType<ActionT>();

    typename rclcpp_action::Client<ActionT>::SharedPtr client = 
      getClientByType<ActionT>();

    if (goal_handle && client) {
        auto cancel_future = client->async_cancel_goal(goal_handle);

        // Ждём завершения
        if (rclcpp::spin_until_future_complete(shared_from_this(), cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to cancel goal");
        }
    }
  }

  template<class ActionT>
  rclcpp_action::ResultCode isTaskComplete(std::shared_future<typename rclcpp_action::Client<ActionT>::WrappedResult> result_future)
  {
    using ResultCode = rclcpp_action::ResultCode;

    if (!result_future.valid()) {
      return ResultCode::SUCCEEDED;   // задача уже завершена или отменена
    }

    auto ret = rclcpp::spin_until_future_complete(
      shared_from_this(), result_future, std::chrono::milliseconds(100));

    if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
      return ResultCode::UNKNOWN;  // всё ещё выполняется
    }

    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      status_ = result_future.get().code;

      if ((int8_t)status_ != action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        RCLCPP_DEBUG(this->get_logger(), "Task failed!");
        return ResultCode::ABORTED;
      }

      RCLCPP_DEBUG(this->get_logger(), "Task succeeded!");
      return ResultCode::SUCCEEDED;
    }

    return ResultCode::SUCCEEDED;  // в случае UNKNOWN считаем завершённым
  }
  // TaskResult getResult();
  // geometry_msgs::msg::PoseWithCovarianceStamped getFeedback();

  void clearLocalCostmap();
  void clearGlobalCostmap();
  void clearAllCostmaps();
  nav2_msgs::srv::GetCostmap::Response getGlobalCostmap();
  nav2_msgs::srv::GetCostmap::Response getLocalCostmap();
  void changeMap(const std::string & map_url);

  // Lifecycle
  void lifecycleStartup();
  void lifecycleShutdown();
  void waitUntilNav2Active(const std::string & navigator = "bt_navigator");

private:
  // callbacks
  template<class ActionT>
  void _feedbackCallback(typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                         const std::shared_ptr<const typename ActionT::Feedback> feedback);
  void _waitForNodeToActivate(const std::string & node_name);

  template<typename ActionT>
  void waitForActionServer(const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
                           const std::string & name);
  template<typename SrvT>
  void waitForService(const typename rclcpp::Client<SrvT>::SharedPtr & client, const std::string & name);

private:
  // Members
  geometry_msgs::msg::PoseWithCovarianceStamped last_feedback_;

  // Action clients
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr compute_path_to_pose_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr compute_path_through_poses_client_;
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smoother_client_;
  rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<nav2_msgs::action::AssistedTeleop>::SharedPtr assisted_teleop_client_;

  // Goal handles and result futures (per-action)
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr follow_path_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr spin_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::BackUp>::SharedPtr backup_goal_handle_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::AssistedTeleop>::SharedPtr assisted_teleop_goal_handle_;

  // result futures
  rclcpp::FutureReturnCode last_result_ready_ = rclcpp::FutureReturnCode::TIMEOUT;
  // We'll keep type-erased last_result via shared_ptr to void; users can get typed results per-action

  // Service clients
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_maps_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_global_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_local_srv_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_global_srv_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_local_srv_;

  rclcpp_action::ResultCode status_;
};

} // namespace eureka_bt
