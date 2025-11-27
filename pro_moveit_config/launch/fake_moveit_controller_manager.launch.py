from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    fake_controllers = PathJoinSubstitution([pkg_cfg, 'config', 'fake_controllers.yaml'])

    # In ROS2 we typically pass controller configs as parameters to the controller manager node.
    node = Node(
        package='moveit_controller_manager',
        executable='moveit_controller_manager',
        name='moveit_controller_manager',
        parameters=[fake_controllers],
        output='screen'
    )

    return LaunchDescription([node])
