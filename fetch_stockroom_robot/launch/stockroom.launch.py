import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fetch_gazebo = get_package_share_directory('fetch_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='aisle.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                fetch_gazebo, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'world_package': 'fetch_stockroom_robot',
                'world': LaunchConfiguration('world'),
                'world_name': 'aisle',
                'robot_name': 'fetch',
                'gui': LaunchConfiguration('gui'),
                'use_ros2_control': LaunchConfiguration('use_ros2_control'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
    ])
