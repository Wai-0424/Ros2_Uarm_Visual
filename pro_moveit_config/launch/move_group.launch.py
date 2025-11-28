from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg_share = FindPackageShare('pro_moveit_config')

    # Launch arguments
    allow_trajectory_execution = LaunchConfiguration('allow_trajectory_execution', default='true')
    fake_execution = LaunchConfiguration('fake_execution', default='false')
    debug = LaunchConfiguration('debug', default='false')
    info = LaunchConfiguration('info', default='false')

    # Include planning context (python launch) if present
    planning_context = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'planning_context.launch.py'])
        )
    )

    # Include planning pipeline (ompl)
    planning_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'ompl_planning_pipeline.launch.py'])
        )
    )

    # Include trajectory execution if trajectory execution is allowed
    trajectory_execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'trajectory_execution.launch.py'])
        )
    )

    # Include controller manager: use fake controller manager when fake_execution is true,
    # otherwise include the swiftpro controller manager (which may be empty placeholder).
    fake_controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'fake_moveit_controller_manager.launch.py'])
        ),
        condition=IfCondition(fake_execution)
    )

    swiftpro_controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'swiftpro_moveit_controller_manager.launch.py'])
        ),
        condition=UnlessCondition(fake_execution)
    )

    # Include sensor manager (optional)
    sensor_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'sensor_manager.launch.py'])
        )
    )

    # move_group node (parameters expected to be provided by included launches or top-level)
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        # Load consolidated ROS2 parameters for move_group (ompl, kinematics, joint_limits)
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'move_group_params.yaml'])],
    )

    ld = LaunchDescription()
    # Declare arguments
    ld.add_action(DeclareLaunchArgument('allow_trajectory_execution', default_value='true'))
    ld.add_action(DeclareLaunchArgument('fake_execution', default_value='false'))
    ld.add_action(DeclareLaunchArgument('debug', default_value='false'))
    ld.add_action(DeclareLaunchArgument('info', default_value='false'))

    # Add included launch files and the move_group node
    ld.add_action(planning_context)
    ld.add_action(planning_pipeline)
    ld.add_action(trajectory_execution)
    ld.add_action(fake_controller_manager)
    ld.add_action(swiftpro_controller_manager)
    ld.add_action(sensor_manager)
    ld.add_action(move_group)

    return ld
