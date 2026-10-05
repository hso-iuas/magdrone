import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    camera_processor_node = Node(
        package='py_camera_processor',
        executable='camProcessor',
        name='camera_processor_node',
        output='screen',
        parameters=[{
            'aruco_marker_side_length': 0.0376
        }],
    )

    # set python venv path
    pkg_prefix = get_package_prefix('py_camera_processor')
    workspace_root = os.path.dirname(os.path.dirname(pkg_prefix))
    venv_site_packages = os.path.join(
        workspace_root,
        'src',
        'py_camera_processor',
        '.venv',
        'lib',
        'python3.12',
        'site-packages'
    )
    set_python_path = AppendEnvironmentVariable(
        name='PYTHONPATH',
        value=venv_site_packages,
        separator=':'
    )

    return LaunchDescription([
        set_python_path,
        sim_mode_arg,
        camera_processor_node
    ])