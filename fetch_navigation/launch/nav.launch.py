import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fetch_navigation_share = get_package_share_directory('fetch_navigation')
    fetch_stockroom_share = get_package_share_directory('fetch_stockroom_robot')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(fetch_navigation_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(fetch_stockroom_share, 'slam', 'map.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nav_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(fetch_navigation_share, 'config', 'navigation.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('rviz', default_value='false'),
        nav2,
        rviz,
    ])
