import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    robot_description_planning = {
        'robot_description_planning': load_yaml(
            'fetch_moveit_config', 'config/joint_limits.yaml').get('joint_limits', {})
    }

    ompl_planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': load_yaml('fetch_moveit_config', 'config/ompl_planning.yaml'),
    }

    trajectory_execution = {
        'allow_trajectory_execution': True,
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning,
            load_yaml('fetch_moveit_config', 'config/moveit_controllers.yaml'),
            trajectory_execution,
            planning_scene_monitor,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group,
    ])
