import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stockroom_share = get_package_share_directory('fetch_stockroom_robot')

    return LaunchDescription([
        DeclareLaunchArgument('moveit_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                stockroom_share, 'launch', 'stockroom_bringup.launch.py')),
            launch_arguments={
                'moveit_rviz': LaunchConfiguration('moveit_rviz'),
                'voice_task': 'true',
            }.items(),
        ),
    ])
