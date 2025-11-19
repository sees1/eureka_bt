#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <eureka_bt/goal_pose.hpp>
#include <eureka_bt/turn_inside.hpp>
#include <eureka_bt/rotate_wheels.hpp>
#include <eureka_bt/stop_robot.hpp>
#include <behaviortree_cpp_v3/bt_factory.h> 
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <eureka_bt/bt_action_node.hpp>

template <typename ActionType>
void register_action(
    std::string&& node_name,
    BT::BehaviorTreeFactory& factory,
    rclcpp::Node::SharedPtr& ros_node)
{
    BT::NodeBuilder builder = [&](auto name, auto config)
    {
      std::string msg = "Building " + node_name + " Action";
      RCLCPP_INFO(ros_node->get_logger(), msg.c_str());
      return std::make_unique<ActionType>(name, config, ros_node);
    };
    factory.registerBuilder<ActionType>(std::move(node_name), builder);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("main_bt");

  BT::BehaviorTreeFactory factory;

  register_action<StopRobot>("Stop_robot", factory, nh);
  register_action<RotateWheels>("Rotate_wheels", factory, nh);
  register_action<Goalpose>("Goal_pose", factory, nh);
  register_action<Turn_inside>("Turn_inside", factory, nh);

  auto tree = factory.createTreeFromFile("./src/eureka_bt/xml_tree/tree.xml");

  auto timer = nh->create_wall_timer(std::chrono::milliseconds(250), 
    [&tree]()
    {
      tree.tickRoot();
    }
  );

  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}

