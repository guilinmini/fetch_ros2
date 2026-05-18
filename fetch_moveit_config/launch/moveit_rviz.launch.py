import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r') as file:
        return yaml.safe_load(file)


def load_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r') as file:
        return file.read()


def generate_launch_description():
    moveit_config_share = get_package_share_directory('fetch_moveit_config')
    fetch_gazebo_share = get_package_share_directory('fetch_gazebo')

    robot_description_file = os.path.join(
        fetch_gazebo_share, 'robots', 'fetch.gazebo.xacro')
    robot_description_config = xacro.process_file(robot_description_file)

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_config.toxml(), value_type=str)
    }
    robot_description_semantic = {
        'robot_description_semantic': load_text(
            'fetch_moveit_config', 'config/fetch.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            'fetch_moveit_config', 'config/kinematics.yaml')
    }

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            moveit_config_share, 'launch', 'move_group.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_move_group')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', os.path.join(moveit_config_share, 'launch', 'moveit.rviz')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_move_group', default_value='true'),
        move_group,
        rviz,
    ])
