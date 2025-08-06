cmake_minimum_required(VERSION 3.8)
project(cagopa_tiago_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ur10_velocity_controller_node src/ur10_velocity_controller_node.cpp)
ament_target_dependencies(ur10_velocity_controller_node rclcpp sensor_msgs std_msgs)

add_executable(test_joint_state_publisher src/test_joint_state_publisher.cpp)
ament_target_dependencies(test_joint_state_publisher rclcpp sensor_msgs)

install(TARGETS
  ur10_velocity_controller_node
  test_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
