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
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SwiftproState.msg"
  "msg/Position.msg"
  "msg/Angle4th.msg"
  "msg/Status.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

## NOTE: The C++ nodes in this package are written for ROS1 (they include <ros/ros.h>)
## and will not compile in a ROS2 workspace without porting. To allow this package to
## provide only the message interfaces (so rosidl can generate them) we do not build
## the ROS1 node sources here. If you want to port the nodes to ROS2, replace the
## ROS1 APIs with rclcpp equivalents and re-add the executables below.

ament_export_dependencies(rosidl_default_runtime)

add_executable(swiftpro_read_node_ros2 src/swiftpro_read_node_ros2.cpp)
ament_target_dependencies(swiftpro_read_node_ros2 rclcpp)
target_include_directories(swiftpro_read_node_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(typesupport_target_read ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swiftpro_read_node_ros2 ${typesupport_target_read})

add_executable(swiftpro_write_node_ros2 src/swiftpro_write_node_ros2.cpp)
ament_target_dependencies(swiftpro_write_node_ros2 rclcpp)
target_include_directories(swiftpro_write_node_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(typesupport_target_write ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swiftpro_write_node_ros2 ${typesupport_target_write})

add_executable(swiftpro_moveit_node_ros2 src/swiftpro_moveit_node_ros2.cpp)
ament_target_dependencies(swiftpro_moveit_node_ros2 rclcpp sensor_msgs)
target_include_directories(swiftpro_moveit_node_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(typesupport_target_moveit ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swiftpro_moveit_node_ros2 ${typesupport_target_moveit})

add_executable(swiftpro_rviz_node_ros2 src/swiftpro_rviz_node_ros2.cpp)
ament_target_dependencies(swiftpro_rviz_node_ros2 rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(swiftpro_rviz_node_ros2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(typesupport_target_rviz ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swiftpro_rviz_node_ros2 ${typesupport_target_rviz})

install(TARGETS
  swiftpro_read_node_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  swiftpro_write_node_ros2
  swiftpro_moveit_node_ros2
  swiftpro_rviz_node_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# Install the simulation publisher script
install(PROGRAMS
  scripts/sim_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install launch files and other resources
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
