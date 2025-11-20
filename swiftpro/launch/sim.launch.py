"""Launch file to start simulation publisher, rviz node and optionally rviz2.

Usage:
  ros2 launch swiftpro sim.launch.py
  ros2 launch swiftpro sim.launch.py start_rviz:=false  # skip launching rviz2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz', default='true')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('start_rviz', default_value='true', description='Start rviz2'))

    # simulation publisher (installed script)
    sim_pub_node = Node(
        package='swiftpro',
        executable='sim_publisher.py',
        name='swiftpro_sim_publisher',
        output='screen',
        emulate_tty=True,
    )

    # the rviz node built in the package (publishes joint_states)
    rviz_node = Node(
        package='swiftpro',
        executable='swiftpro_rviz_node_ros2',
        name='swiftpro_rviz_node',
        output='screen',
    )

    # robot_state_publisher: use xacro to generate robot_description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('swiftpro'), 'urdf', 'swift_model.xacro'])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # start rviz2 (user can close it if not needed)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('swiftpro'), 'rviz', 'swiftpro_default.rviz'])]
    )

    ld.add_action(sim_pub_node)
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz2_node)

    return ld
