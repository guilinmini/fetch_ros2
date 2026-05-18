import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = get_package_share_directory('robot')

    config_dir = os.path.join(robot_share, 'config')

    return LaunchDescription([
        Node(
            package='robot',
            executable='label_publisher.py',
            name='label_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'config_dir': config_dir}],
        ),
        Node(
            package='robot',
            executable='speech_asr_node.py',
            name='speech_asr_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'fallback_text': '从一号桌抓取胶水放到二号桌',
            }],
        ),
        Node(
            package='robot',
            executable='speech_nlu_node.py',
            name='speech_nlu_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='robot',
            executable='task_dispatcher.py',
            name='task_dispatcher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'config_dir': config_dir,
                'world_name': 'aisle',
                'robot_model_name': 'fetch',
            }],
        ),
    ])
