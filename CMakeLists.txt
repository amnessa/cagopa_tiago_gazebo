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

add_executable(tiago_controller_node src/tiago_controller_node.cpp)
ament_target_dependencies(tiago_controller_node rclcpp rclcpp_action control_msgs trajectory_msgs)

install(TARGETS
  tiago_controller_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
