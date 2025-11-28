from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Converted from legacy XML that provided warehouse settings.
    moveit_warehouse_port_arg = DeclareLaunchArgument(
        'moveit_warehouse_port', default_value='33829', description='MoveIt warehouse DB port'
    )

    moveit_warehouse_host_arg = DeclareLaunchArgument(
        'moveit_warehouse_host', default_value='localhost', description='MoveIt warehouse DB host'
    )

    # Other legacy params (warehouse_exec, warehouse_plugin) were static values; if needed
    # they can be exposed as launch arguments or passed as ros__parameters to specific nodes.

    ld = LaunchDescription()
    ld.add_action(moveit_warehouse_port_arg)
    ld.add_action(moveit_warehouse_host_arg)

    return ld
