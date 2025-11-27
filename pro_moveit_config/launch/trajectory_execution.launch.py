from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    trajectory_params = PathJoinSubstitution([pkg_cfg, 'config', 'ompl_planning.yaml'])

    node = Node(
        package='moveit_controller_manager',
        executable='trajectory_execution',
        name='trajectory_execution',
        parameters=[trajectory_params],
        output='screen'
    )

    return LaunchDescription([node])
