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
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(ur10_velocity_controller_node src/ur10_velocity_controller_node.cpp)
ament_target_dependencies(ur10_velocity_controller_node rclcpp sensor_msgs std_msgs)

add_executable(test_joint_state_publisher src/test_joint_state_publisher.cpp)
ament_target_dependencies(test_joint_state_publisher rclcpp sensor_msgs)

add_executable(ball_tracker_node src/ball_tracker_node.cpp)
ament_target_dependencies(ball_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_geometry
)
target_link_libraries(ball_tracker_node ${OpenCV_LIBRARIES})

install(TARGETS
  ur10_velocity_controller_node
  test_joint_state_publisher
  ball_tracker_node
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
