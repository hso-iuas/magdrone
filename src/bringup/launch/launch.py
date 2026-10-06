import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    fcu_url = 'udp://:14540@'

    common_group = GroupAction(
        actions=[
            # camera processor
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('py_camera_processor'),
                                 'launch', 'launch.py')
                )
            ),
            # commander
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('cpp_commander'),
                                 'launch', 'launch.py')
                )
            ),
            # mavros
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('mavros'),
                        'launch', 'px4.launch'
                    )
                ),
                launch_arguments={
                    'fcu_url': fcu_url,
                }.items()
            ),
            # camera optical frame
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_camera_optical_frame',
                arguments=[
                    '0', '0', '0',
                    '-1.57', '0', '-1.57',
                    'camera_hardware',
                    'camera_optical_frame'
                ]
            ),
        ]
    )

    real_group = GroupAction(
        actions=[
            # magnet
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('py_magnet'),
                                 'launch', 'launch.py')
                )
            ),
            # camera
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch', 'rs_launch.py'
                    )
                ),
                launch_arguments={
                    'enable_sync': 'false',

                    'enable_color': 'false',
                    'enable_infra1': 'true',
                    'enable_depth': 'true',
                    'depth_module.infra_profile': '848x480x30',
                    'depth_module.profile': '848x480x30',
                    'depth_module.enable_auto_exposure': 'true',
                    'initial_reset': 'true'
                }.items()
            ),

            # 0 0 0 (x y z) is the middle of the top plate of the drone
            # magnet pose
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_magnet',
                arguments=[
                    '0', '0', '0.14',  # x, y, z
                    '0', '0', '0',  # yaw, pitch, roll (in radians)
                    'drone_top_plate',  # parent frame
                    'magnet_tip'  # child frame
                ],
            ),
            #SCT
            #GAGA
            #HHAG
            # camera pose
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_camera_hardware',
                arguments=[
                    '0.107', '-0.02', '0.0',  # x, y, z
                    '3.141', '-1.309', '0',  # yaw, pitch, roll (in radians)
                    'drone_top_plate',  # parent frame
                    'camera_hardware'  # child frame
                ],
            ),
            # center of gravity pose
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub_magnet',
                arguments=[
                    '0', '0', '-0.05',  # x, y, z
                    '0', '0', '0',  # yaw, pitch, roll (in radians)
                    'drone_top_plate',  # parent frame
                    'drone_center'  # child frame
                ],
            ),

            TimerAction(
                period=30.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'param', 'set', '/camera/camera', 'depth_module.emitter_enabled', '0'],
                        output='screen'
                    )
                ]
            )
        ]
    )

    return LaunchDescription([
        common_group,
        real_group
    ])