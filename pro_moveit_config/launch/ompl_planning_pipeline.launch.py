from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    planning_plugin = LaunchConfiguration('planning_plugin', default='ompl_interface/OMPLPlanner')
    planning_adapters = LaunchConfiguration('planning_adapters')

    ompl_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'ompl_planning.yaml'])

    # This ROS2 launch provides the OMPL yaml as a parameter file to any node that includes it.
    ld = LaunchDescription()
    # Users can include this launch and reuse 'ompl_yaml' PathJoinSubstitution as needed.
    return ld
