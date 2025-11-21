"""Launch file to start real robot control nodes and rviz WITHOUT MoveIt or Xacro.

Usage:
  ros2 launch swiftpro minimal.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    # swiftpro_write_node_ros2: communicates with the robot via serial
    write_node = Node(
        package='swiftpro',
        executable='swiftpro_write_node_ros2',
        name='swiftpro_write_node',
        output='screen',
    )

    # swiftpro_rviz_node_ros2: publishes joint_states for visualization
    # It reads from the robot and publishes to /joint_states
    rviz_node = Node(
        package='swiftpro',
        executable='swiftpro_rviz_node_ros2',
        name='swiftpro_rviz_node',
        output='screen',
    )

    # robot_state_publisher: use the raw URDF file (no xacro needed)
    urdf_file = PathJoinSubstitution([FindPackageShare('swiftpro'), 'urdf', 'pro_model.urdf'])
    
    # We need to read the file content to pass it as a parameter, 
    # but robot_state_publisher also accepts a file path in 'arguments' or 'parameters' depending on version.
    # In Humble, it usually takes 'robot_description' parameter as string.
    # Since we can't run 'cat' easily in Command if we want to be safe, 
    # we can use Command 'cat' which is standard.
    
    robot_description_content = Command(['cat ', urdf_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # start rviz2 with default config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('swiftpro'), 'rviz', 'swiftpro_default.rviz'])]
    )

    ld.add_action(write_node)
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz2_node)

    return ld
