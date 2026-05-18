import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stockroom_share = get_package_share_directory('fetch_stockroom_robot')
    moveit_share = get_package_share_directory('fetch_moveit_config')
    navigation_share = get_package_share_directory('fetch_navigation')
    robot_share = get_package_share_directory('robot')
    fastdds_profile = os.path.join(robot_share, 'config', 'fastdds_no_shm.xml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            stockroom_share, 'launch', 'stockroom.launch.py')),
        launch_arguments={
            'gui': 'true',
            'use_ros2_control': 'true',
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            moveit_share, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            moveit_share, 'launch', 'moveit_rviz.launch.py')),
        condition=IfCondition(LaunchConfiguration('moveit_rviz')),
        launch_arguments={
            'use_sim_time': 'true',
            'launch_move_group': 'false',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_share, 'launch', 'nav.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': 'true',
        }.items(),
    )

    voice_task = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            robot_share, 'launch', 'voice_nodes.launch.py')),
        condition=IfCondition(LaunchConfiguration('voice_task')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('moveit_rviz', default_value='false'),
        DeclareLaunchArgument('voice_task', default_value='true'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        simulation,
        moveit,
        moveit_rviz,
        navigation,
        voice_task,
    ])
