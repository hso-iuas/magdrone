import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    cpp_commander_node = Node(
        package='cpp_commander',
        executable='mavros_commander',
        name='cpp_commander_node',
        output='screen',
        #prefix=['nice -n -17 '],

    )

    return LaunchDescription([
        cpp_commander_node
    ])