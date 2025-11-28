from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, ConcatSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_cfg = FindPackageShare('pro_moveit_config')

    pipeline_arg = DeclareLaunchArgument(
        'pipeline', default_value='ompl', description='Which planning pipeline to use (eg. ompl)'
    )

    pipeline_name = ConcatSubstitution([
        LaunchConfiguration('pipeline'),
        TextSubstitution(text='_planning_pipeline.launch.py')
    ])

    pipeline_launch_path = PathJoinSubstitution([pkg_cfg, 'launch', pipeline_name])

    include_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch_path)
    )

    ld = LaunchDescription()
    ld.add_action(pipeline_arg)
    ld.add_action(include_pipeline)

    return ld
