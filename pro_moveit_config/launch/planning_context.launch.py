from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg = FindPackageShare('pro_moveit_config')

    declare_load_robot = DeclareLaunchArgument('load_robot_description', default_value='false')
    load_robot = LaunchConfiguration('load_robot_description')

    # Build xacro command to generate robot_description if requested
    robot_description_cmd = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift, 'urdf', 'pro_model.xacro'])
    ])

    # Provide the SRDF path (move_group will read it as needed)
    robot_description_semantic = PathJoinSubstitution([pkg_cfg, 'config', 'swiftpro.srdf'])

    # Note: in ROS2 we generally pass parameter files into the nodes that need them
    # (e.g., move_group). This launch provides helpers and does not directly load
    # parameters into a global server as ROS1 did.

    ld = LaunchDescription()
    ld.add_action(declare_load_robot)

    # If other systems need to read these substitutions, they can import this launch
    # and reuse 'robot_description_cmd' and 'robot_description_semantic'.
    return ld
