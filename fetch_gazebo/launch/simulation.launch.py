import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_x = LaunchConfiguration('x').perform(context)
    robot_y = LaunchConfiguration('y').perform(context)
    robot_z = LaunchConfiguration('z').perform(context)
    robot_yaw = LaunchConfiguration('yaw').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    world_package = LaunchConfiguration('world_package').perform(context)
    world_file = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')

    world_package_share = get_package_share_directory(world_package)
    fetch_gazebo_share = get_package_share_directory('fetch_gazebo')
    fetch_description_share = get_package_share_directory('fetch_description')
    world_path = os.path.join(world_package_share, 'worlds', world_file)
    robot_path = os.path.join(fetch_gazebo_share, 'robots', 'fetch.gazebo.xacro')
    robot_description = xacro.process_file(robot_path).toxml()

    resource_paths = [
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.path.dirname(fetch_description_share),
        os.path.join(fetch_gazebo_share, 'models'),
        os.path.join(world_package_share, 'models'),
        '/usr/share/gazebo-11/models',
    ]
    resource_path = os.pathsep.join(path for path in resource_paths if path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    laser_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_laser_static_tf',
        output='screen',
        arguments=[
            '--frame-id',
            'laser_link',
            '--child-frame-id',
            f'{robot_name}/base_link/base_laser',
        ],
        parameters=[{'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {"" if gui else "-s "}{world_path}',
            'gz_version': '6',
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world_name,
            '-name', robot_name,
            '-topic', '/robot_description',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
            '-Y', robot_yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/world/{world_name}/model/{robot_name}/joint_state@'
            'sensor_msgs/msg/JointState[gz.msgs.Model',
            '/base_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/head_camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
            '/head_camera/depth/camera_info@'
            'sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/joint_state', '/joint_states'),
        ],
    )

    controller_spawners = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration('use_ros2_control')),
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['arm_with_torso_controller', '--controller-manager', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['head_controller', '--controller-manager', '/controller_manager'],
            ),
        ],
    )
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        robot_state_publisher,
        laser_frame_tf,
        spawn,
        bridge,
        controller_spawners,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_package', default_value='fetch_gazebo'),
        DeclareLaunchArgument('world', default_value='test_zone.sdf'),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('robot_name', default_value='fetch'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
