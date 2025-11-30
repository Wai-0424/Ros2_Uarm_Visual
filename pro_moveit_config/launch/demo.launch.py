from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg = FindPackageShare('pro_moveit_config')

    # Produce robot_description from xacro
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift, 'urdf', 'pro_model.xacro'])
    ])

    # Read SRDF as string (some move_group nodes expect content)
    robot_description_semantic = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        PathJoinSubstitution([pkg_cfg, 'config', 'swiftpro.srdf'])
    ])
    # Ensure the SRDF path/content is passed as a string parameter (avoid yaml parsing)
    robot_description_semantic = ParameterValue(robot_description_semantic, value_type=str)

    # Base parameters passed to move_group and rviz
    params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'planning_pipelines': ['ompl']
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[params]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_cfg, 'launch', 'moveit.rviz'])],
        parameters=[params]
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'Base']
    )

    # Publish fake joint states
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([rsp, move_group, rviz, static_tf, jsp])
