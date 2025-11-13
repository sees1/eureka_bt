cmake_minimum_required(VERSION 3.16)
project(eureka_bt LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  sensor_msgs
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  rclcpp
  rcpputils
  std_msgs
  tf2
  rclcpp_action
  nav2_msgs
  # behaviortree_ros2
)

foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(${PROJECT_NAME}
  src/action_nodes/turn_inside.cpp
  src/action_nodes/goal_pose.cpp
  src/action_nodes/cv.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  geometry_msgs
  nav_msgs
  behaviortree_cpp_v3
  rclcpp
  rcpputils
  std_msgs
  tf2
  rclcpp_action
  nav2_msgs
  # behaviortree_ros2
)

# INSTALL
install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
