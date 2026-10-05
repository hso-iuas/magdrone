import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    py_magnet_node = Node(
        package='py_magnet',
        executable='magnetService',
        name='py_magnet_node',
        output='screen'
    )

    return LaunchDescription([
        py_magnet_node
    ])