cmake_minimum_required(VERSION 3.8)
project(swiftpro)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SwiftproState.msg"
  "msg/position.msg"
  "msg/angle4th.msg"
  "msg/status.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

add_executable(swiftpro_write_node src/swiftpro_write_node.cpp)
add_executable(swiftpro_moveit_node src/swiftpro_moveit_node.cpp)
add_executable(swiftpro_rviz_node  src/swiftpro_rviz_node.cpp)
add_executable(swiftpro_read_node  src/swiftpro_read_node.cpp)

ament_target_dependencies(swiftpro_write_node  rclcpp std_msgs sensor_msgs tf2 tf2_ros)
ament_target_dependencies(swiftpro_moveit_node rclcpp std_msgs sensor_msgs tf2 tf2_ros)
ament_target_dependencies(swiftpro_rviz_node   rclcpp std_msgs sensor_msgs tf2 tf2_ros)
ament_target_dependencies(swiftpro_read_node   rclcpp std_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS
  swiftpro_write_node
  swiftpro_moveit_node
  swiftpro_rviz_node
  swiftpro_read_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
