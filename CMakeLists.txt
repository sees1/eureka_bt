cmake_minimum_required(VERSION 3.16)
project(eureka_bt LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rcpputils
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  lifecycle_msgs
  behaviortree_cpp_v3
  tf2
)

foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(simple_commander SHARED
  src/nav2_simple_commander/basic_nav.cpp)

target_include_directories(simple_commander PUBLIC include/eureka_bt)

ament_target_dependencies(simple_commander
  rclcpp
  rcpputils
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  lifecycle_msgs
  behaviortree_cpp_v3
  tf2
)

add_executable(${PROJECT_NAME}
  src/turn_inside.cpp
  src/goal_pose.cpp
  src/stop_robot.cpp
  src/rotate_wheels.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME} simple_commander)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcpputils
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  lifecycle_msgs
  behaviortree_cpp_v3
  tf2
)

# INSTALL
install(TARGETS ${PROJECT_NAME} simple_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
