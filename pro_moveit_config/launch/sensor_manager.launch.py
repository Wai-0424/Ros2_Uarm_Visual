from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    # Expose sensor manager config (octomap params, etc.) for ROS2 usage
    # For now we provide a node stub that would be replaced by the actual sensor manager.
    sensor_params = PathJoinSubstitution([pkg_cfg, 'config', 'kinematics.yaml'])

    node = Node(
        package='sensor_manager',
        executable='sensor_manager',
        name='sensor_manager',
        parameters=[sensor_params],
        output='screen'
    )

    return LaunchDescription([node])
