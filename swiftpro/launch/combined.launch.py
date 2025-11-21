"""Launch file to start both real robot control and simulation nodes.

Usage:
  ros2 launch swiftpro combined.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(SetEnvironmentVariable('AMENT_PREFIX_PATH', '/media/wai4424/Data/Ros2_Uarm_Visual-main/install:/opt/ros/humble'))
    ld.add_action(SetEnvironmentVariable('LD_LIBRARY_PATH', '/media/wai4424/Data/Ros2_Uarm_Visual-main/install/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib'))
    ld.add_action(SetEnvironmentVariable('PYTHONPATH', '/media/wai4424/Data/Ros2_Uarm_Visual-main/install/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages'))

    # Include real robot launch file
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('swiftpro'),
                'launch',
                'real_robot.launch.py'
            ])
        )
    )

    # Include simulation launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('swiftpro'),
                'launch',
                'sim.launch.py'
            ])
        )
    )

    # Add both launch files to the launch description
    ld.add_action(real_robot_launch)
    ld.add_action(sim_launch)

    return ld