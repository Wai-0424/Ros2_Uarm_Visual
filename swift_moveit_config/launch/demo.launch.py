from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg  = FindPackageShare('swift_moveit_config')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift,'urdf','swift_model.xacro'])
    ])

    params = {
        'robot_description': robot_description,
        'robot_description_semantic':
            PathJoinSubstitution([pkg_cfg,'config','swiftpro.srdf']),
        'planning_pipelines': ['ompl'],
        'use_sim_time': False
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[params,
                    PathJoinSubstitution([pkg_cfg,'config','ompl_planning.yaml'])]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_cfg,'launch','moveit.rviz'])],
        parameters=[params]
    )

    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description}])

    return LaunchDescription([jsp, rsp, move_group, rviz])
