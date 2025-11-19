from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_swift = FindPackageShare('swiftpro')
    pkg_cfg  = FindPackageShare('pro_moveit_config')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_swift,'urdf','swift_model.xacro'])
    ])

    params = {
        'robot_description': robot_description,
        'robot_description_semantic':
            PathJoinSubstitution([pkg_cfg,'config','swiftpro.srdf']),
        'planning_pipelines': ['ompl']
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[params,
                    PathJoinSubstitution([pkg_cfg,'config','ompl_planning.yaml'])]
    )

    return LaunchDescription([move_group])
