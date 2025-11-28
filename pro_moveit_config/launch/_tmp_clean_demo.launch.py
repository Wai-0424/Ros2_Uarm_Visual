from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg = FindPackageShare('pro_moveit_config')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift, 'urdf', 'swift_model.xacro'])
    ])

    robot_description_semantic = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        PathJoinSubstitution([pkg_cfg, 'config', 'swiftpro.srdf'])
    ])
    robot_description_semantic = ParameterValue(robot_description_semantic, value_type=str)

    params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'planning_pipelines': ['ompl']
    }

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

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

    return LaunchDescription([rsp, move_group, rviz])
