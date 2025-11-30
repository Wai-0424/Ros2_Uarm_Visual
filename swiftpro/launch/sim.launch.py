"""Launch file to start simulation publisher, rviz node and optionally rviz2.

Usage:
  ros2 launch swiftpro sim.launch.py
  ros2 launch swiftpro sim.launch.py start_rviz:=false  # skip launching rviz2
  ros2 launch swiftpro sim.launch.py model_type:=swift  # use swift model instead of pro
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    model_type = LaunchConfiguration('model_type')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('start_rviz', default_value='true', description='Start rviz2'))
    ld.add_action(DeclareLaunchArgument('model_type', default_value='pro', description='Model type: pro or swift'))

    # simulation publisher (installed script)
    # Using the script name directly as it is installed to lib/swiftpro/
    sim_pub_node = Node(
        package='swiftpro',
        executable='sim_publisher.py',
        name='swiftpro_sim_publisher',
        output='screen',
        emulate_tty=True,
    )

    # the rviz node built in the package (publishes joint_states)
    rviz_node = Node(
        package='swiftpro',
        executable='swiftpro_rviz_node_ros2',
        name='swiftpro_rviz_node',
        output='screen',
    )

    # Determine xacro file based on model_type
    # We use a PythonExpression to select the file name
    xacro_file_name = PythonExpression([
        "'pro_model.xacro' if '", model_type, "' == 'pro' else 'swift_model.xacro'"
    ])

    # robot_state_publisher: use xacro to generate robot_description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('swiftpro'), 'urdf', xacro_file_name])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # start rviz2 (user can close it if not needed)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(start_rviz),
        arguments=['-d', PathJoinSubstitution([FindPackageShare('swiftpro'), 'rviz', 'swiftpro_default.rviz'])]
    )

    # Static transform publisher from world to Base
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    ld.add_action(sim_pub_node)
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz2_node)
    ld.add_action(static_tf_node)

    return ld
