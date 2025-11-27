from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg = FindPackageShare('pro_moveit_config')

    # Produce robot_description from xacro
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift, 'urdf', 'swift_model.xacro'])
    ])

    # Read SRDF as string (some move_group nodes expect content)
    robot_description_semantic = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        PathJoinSubstitution([pkg_cfg, 'config', 'swiftpro.srdf'])
    ])

    # Base parameters passed to move_group and rviz
    params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'planning_pipelines': ['ompl']
    }

    # Include additional parameter files (OMPL, kinematics, joint limits)
    ompl_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'ompl_planning.yaml'])
    kinematics_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'kinematics.yaml'])
    joint_limits_yaml = PathJoinSubstitution([pkg_cfg, 'config', 'joint_limits.yaml'])

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[params, ompl_yaml, kinematics_yaml, joint_limits_yaml]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_cfg, 'launch', 'moveit.rviz'])],
        parameters=[params]
    )

    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher')
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([jsp, rsp, move_group, rviz])
