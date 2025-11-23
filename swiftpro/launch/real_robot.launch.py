"""Launch file to start real robot control nodes and rviz.

Usage:
  ros2 launch swiftpro real_robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # swiftpro_write_node_ros2: communicates with the robot via serial
    write_node = Node(
        package='swiftpro',
        executable='swiftpro_write_node_ros2',
        name='swiftpro_write_node',
        output='screen',
        parameters=[{'enable_writes': True}]  # Enable actual hardware communication
    )

    # swiftpro_read_node_ros2: reads robot state from serial and publishes SwiftproState_topic
    read_node = Node(
        package='swiftpro',
        executable='swiftpro_read_node_ros2',
        name='swiftpro_read_node',
        output='screen'
    )

    # swiftpro_moveit_node_ros2: converts MoveIt fake controller states to robot commands
    moveit_node = Node(
        package='swiftpro',
        executable='swiftpro_moveit_node_ros2',
        name='swiftpro_moveit_node',
        output='screen',
    )

    # swiftpro_rviz_node_ros2: publishes joint_states for visualization
    rviz_node = Node(
        package='swiftpro',
        executable='swiftpro_rviz_node_ros2',
        name='swiftpro_rviz_node',
        output='screen',
    )

    # robot_state_publisher: use xacro to generate robot_description
    # Using pro_model.xacro as default for Swift Pro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('swiftpro'), 'urdf', 'pro_model.xacro'])
    ])

    # Load SRDF for MoveIt semantic description
    robot_description_semantic_content = Command([
        'cat ',
        PathJoinSubstitution([FindPackageShare('pro_moveit_config'), 'config', 'swiftpro.srdf'])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Define params before rviz2_node to ensure it is initialized before use
    params = {
        'robot_description': robot_description_content,
        'planning_pipelines': ['ompl']
    }

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('pro_moveit_config'), 'launch', 'moveit.rviz'])],
        parameters=[{
            'robot_description': robot_description_content,
            'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str),
            'planning_pipelines': ['ompl']
        }]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str),
            'planning_pipelines': ['ompl'],
            'planner_configs': {
                'SBLkConfigDefault': {
                    'type': 'geometric::SBL',
                    'range': 0.0
                },
                'ESTkConfigDefault': {
                    'type': 'geometric::EST',
                    'range': 0.0,
                    'goal_bias': 0.05
                }
            }
        }]
    )

    # Static transform publisher from world to Base
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'Base']
    )

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'odom']
    )

    ld.add_action(write_node)
    ld.add_action(read_node)
    ld.add_action(moveit_node)
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(move_group_node)
    ld.add_action(static_tf_node)
    ld.add_action(rviz2_node)
    ld.add_action(static_transform_publisher)

    return ld
