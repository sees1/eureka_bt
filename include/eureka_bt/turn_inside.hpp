#pragma once
#include <future>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Turn_inside : public BT::StatefulActionNode {
public:
    Turn_inside(const std::string& name,
                const BT::NodeConfiguration& config,
                rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    virtual BT::NodeStatus onStart() override;
    virtual BT::NodeStatus onRunning() override;
    virtual void onHalted() override;

protected:
    // before been in cv node
    void processValues();
    double calculateAverage(const std::vector<double>& values);
    void clearData();

    void updateGoalPose(double turn_angle);
    bool stopRobot();
    bool tryToCollectData();
    double computeRobotYaw();

private:
    // ros parameter's
    std::string odometry_topic_name_;
    bool dry_run_;

private:
    std::mutex mut;
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arrow_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_turn;

    bool collect_success_;
    bool turning_task_finished_;
    bool is_robot_stop_;
    bool stop_fire_once_;

    std::chrono::time_point<std::chrono::steady_clock> last_time_point_;

    double before_turning_yaw_;
    double turn_angle_;

    std::vector<std::string> names_;
    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<double> efforts_;

    std::string narrow = "No_detection";
    double length = 0.0;
    double angle = 0.0;
    double coef = 0.0;

    double pose_x_;
    double pose_y_;
    double orientationw;
    double orientationx;
    double orientationy;
    double orientationz;
};

