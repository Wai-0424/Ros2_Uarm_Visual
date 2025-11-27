from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    declare_config = DeclareLaunchArgument('config', default_value='false')
    rviz_config_arg = LaunchConfiguration('config')

    rviz_config_path = PathJoinSubstitution([pkg_cfg, 'launch', 'moveit.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([declare_config, rviz_node])
