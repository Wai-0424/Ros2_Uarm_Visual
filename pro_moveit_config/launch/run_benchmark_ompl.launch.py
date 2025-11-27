from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    cfg_arg = DeclareLaunchArgument('cfg', description='Benchmark config file(s)')
    cfg = LaunchConfiguration('cfg')

    kinematics_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'kinematics.yaml'])
    ompl_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'ompl_planning.yaml'])

    # This minimal ROS2 wrapper will launch the benchmark node when needed.
    benchmark_node = Node(
        package='moveit_ros_benchmarks',
        executable='moveit_run_benchmark',
        name='moveit_run_benchmark',
        output='screen',
        arguments=[LaunchConfiguration('cfg'), '--benchmark-planners'],
        parameters=[kinematics_yaml, ompl_yaml]
    )

    return LaunchDescription([cfg_arg, benchmark_node])
