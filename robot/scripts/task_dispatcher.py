#!/usr/bin/env python3

import time
import subprocess
import math

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from common import BOX_SIZE, load_task_config, make_pose, quaternion_from_yaw, slot_offset
from robot.srv import SpeechNLUSrv


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = type('QuaternionTuple', (), {})()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


def yaw_from_quaternion(q):
    x = float(q.get('x', 0.0))
    y = float(q.get('y', 0.0))
    z = float(q.get('z', 0.0))
    w = float(q.get('w', 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_from_ros_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate_vector_by_quaternion(x, y, z, q):
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return x, y, z
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    uvx = qy * z - qz * y
    uvy = qz * x - qx * z
    uvz = qx * y - qy * x
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx

    return (
        x + 2.0 * (qw * uvx + uuvx),
        y + 2.0 * (qw * uvy + uuvy),
        z + 2.0 * (qw * uvz + uuvz),
    )


class TaskDispatcher(Node):
    def __init__(self):
        super().__init__('task_dispatcher')
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('config_dir', '')
        self.declare_parameter('world_name', 'aisle')
        self.declare_parameter('robot_model_name', 'fetch')
        config = load_task_config(self.get_parameter('config_dir').value)

        self.locations = {
            key: value for key, value in config.items()
            if key.startswith('table_') and isinstance(value, dict) and 'yaw' in value
        }
        self.move_group_name = str(config.get('move_group_name', 'arm')).strip() or 'arm'
        self.table_surfaces = config.get('table_surfaces', {})
        self.objects = config.get('objects', {})
        self.object_states = self.build_initial_object_states()
        self.pregrasp_clearance = float(config.get('pregrasp_clearance', 0.12))
        self.place_clearance = float(config.get('place_clearance', 0.12))
        self.postgrasp_lift = float(config.get('postgrasp_lift', 0.10))
        self.gripper_target_standoff = float(config.get('gripper_target_standoff', 0.12))
        self.approach_distance = float(config.get('approach_distance', 0.10))
        self.approach_style = str(config.get('approach_style', 'top_down')).strip().lower()
        self.top_down_xy_offset = float(config.get('top_down_xy_offset', 0.08))
        self.top_down_grasp_z_offset = float(config.get('top_down_grasp_z_offset', 0.08))
        self.edge_table_x_offset = float(config.get('edge_table_x_offset', 0.12))
        self.planning_world_frame = str(config.get('planning_world_frame', 'world'))
        self.planning_scene_frame = str(config.get('planning_scene_frame', 'odom'))
        self.startup_scene_delay = float(config.get('startup_scene_delay', 8.0))
        self.post_navigation_scene_delay = float(config.get('post_navigation_scene_delay', 0.2))
        self.grasp_roll = float(config.get('grasp_roll', 0.0))
        self.grasp_pitch = float(config.get('grasp_pitch', 1.5708))
        self.grasp_yaw = float(config.get('grasp_yaw', 0.0))
        self.goal_orientation_tolerance = float(config.get('goal_orientation_tolerance', 0.08))

        self.world_name = self.get_parameter('world_name').value
        self.robot_model_name = self.get_parameter('robot_model_name').value
        self.world_pose_cache = None
        self.manipulation_robot_pose = None
        self.manipulation_world_to_base = None
        self.planning_scene_location = ''
        self.nlu_client = self.create_client(
            SpeechNLUSrv, '/speech_nlu_service', callback_group=self.callback_group)
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene',
            callback_group=self.callback_group)
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.grasp_pose_sub = self.create_subscription(
            PoseStamped, '/grasp_pose', self.handle_grasp_pose, 10,
            callback_group=self.callback_group)
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.callback_group)
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group)
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.callback_group)
        self.compute_cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self.callback_group)
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.head_client = ActionClient(
            self, FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
            callback_group=self.callback_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scene_published = False
        self.grasp_pose_busy = False

        self.service = self.create_service(
            SpeechNLUSrv, '/run_voice_task', self.handle_run_voice_task,
            callback_group=self.callback_group)
        self.startup_scene_timer = self.create_timer(
            self.startup_scene_delay, self.publish_startup_planning_scene_once,
            callback_group=self.callback_group)
        self.get_logger().info('voice task dispatcher ready: /run_voice_task')

    def handle_grasp_pose(self, msg):
        if self.grasp_pose_busy:
            self.get_logger().warn('ignore /grasp_pose because a grasp is already running')
            return
        self.grasp_pose_busy = True
        try:
            ok, message = self.grasp_published_pose(msg)
            if ok:
                self.get_logger().info(f'/grasp_pose finished: {message}')
            else:
                self.get_logger().error(f'/grasp_pose failed: {message}')
        except Exception as exc:
            self.get_logger().error(f'/grasp_pose exception: {exc}')
        finally:
            self.grasp_pose_busy = False

    def grasp_published_pose(self, msg):
        frame_id = (msg.header.frame_id or 'world').strip()
        if frame_id not in ('world', 'gazebo_world', 'gz_world', 'ign_world'):
            return False, f'unsupported frame_id for /grasp_pose: {frame_id}; use world'

        target = {
            'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'z': float(msg.pose.position.z),
        }
        self.get_logger().info(
            f'/grasp_pose received object center in {frame_id}: '
            f'x={target["x"]:.3f} y={target["y"]:.3f} z={target["z"]:.3f}')

        self.begin_manipulation_frame()
        try:
            if not self.command_head([0.0, 0.35], 1.0):
                self.get_logger().warn('head command failed, continuing')
            if not self.command_gripper(0.045):
                return False, 'open gripper failed'
            if not self.move_gripper_to_pose_point(target, 'pregrasp'):
                return False, 'pregrasp MoveIt plan failed'
            if not self.move_gripper_to_pose_point(target, 'grasp', cartesian=True):
                return False, 'grasp MoveIt plan failed'
            if not self.command_gripper(0.0):
                return False, 'close gripper failed'
            self.move_arm_to_joint_goal('stow')
            return True, 'picked published pose'
        finally:
            self.clear_manipulation_frame()

    def build_initial_object_states(self):
        states = {}
        table_counts = {}
        for object_key in sorted(self.objects.keys()):
            config = self.objects[object_key]
            table_name = config.get('table', '')
            surface = self.table_surfaces.get(table_name, {})
            slot = table_counts.get(table_name, 0)
            table_counts[table_name] = slot + 1
            dx, dy = slot_offset(slot)
            states[object_key] = {
                'display_name': config.get('display_name', object_key),
                'model_name': config.get('model_name', object_key),
                'table': table_name,
                'slot': slot,
                'held': False,
                'x': float(surface.get('x', 0.0)) + dx,
                'y': float(surface.get('y', 0.0)) + dy,
                'z': float(surface.get('z', 0.78)) + BOX_SIZE / 2.0,
            }
        return states

    def handle_run_voice_task(self, request, response):
        try:
            parsed = self.parse_voice_task(request.file_path)
            response.raw_text = parsed.raw_text
            response.target_object = parsed.target_object
            response.actions = list(parsed.actions)
            response.source_location = parsed.source_location
            response.target_location = parsed.target_location

            if not parsed.success:
                response.success = False
                response.message = parsed.message
                return response

            if 'pick' in parsed.actions and 'place' in parsed.actions:
                ok, message = self.execute_pick_place(
                    parsed.target_object, parsed.source_location, parsed.target_location)
            elif 'navigate' in parsed.actions:
                ok, message = self.navigate_to(parsed.target_location or parsed.source_location)
            else:
                ok, message = False, 'unsupported task'

            response.success = ok
            response.message = message
            return response
        except Exception as exc:
            self.get_logger().error(f'task dispatcher exception: {exc}')
            response.success = False
            response.message = str(exc)
            return response

    def parse_voice_task(self, file_path):
        if not self.nlu_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError('speech NLU service unavailable')
        req = SpeechNLUSrv.Request()
        req.file_path = file_path
        future = self.nlu_client.call_async(req)
        result = self.wait_future(future, timeout_sec=30.0)
        if result is None:
            raise RuntimeError('speech NLU timeout')
        return result

    def execute_pick_place(self, object_key, source_location, target_location):
        if object_key not in self.object_states:
            return False, f'object not found: {object_key}'
        if not source_location:
            source_location = self.object_states[object_key].get('table', '')
        if source_location not in self.locations:
            return False, f'source location invalid: {source_location}'
        if target_location not in self.locations:
            return False, f'target location invalid: {target_location}'

        self.get_logger().info(
            f'task start: pick {object_key} from {source_location}, place to {target_location}')

        ok, message = self.navigate_to(source_location)
        if not ok:
            return False, f'navigate source failed: {message}'

        ok, message = self.pick_object(object_key)
        if not ok:
            return False, message

        ok, message = self.navigate_to(target_location)
        if not ok:
            return False, f'navigate target failed: {message}'

        ok, message = self.place_object(object_key, target_location)
        if not ok:
            return False, message
        return True, 'task finished'

    def navigate_to(self, location_name, prepare_planning_scene=False):
        if location_name not in self.locations:
            return False, f'unknown location: {location_name}'
        if not self.wait_action_server(self.nav_client, '/navigate_to_pose', timeout_sec=120.0):
            return False, 'navigate_to_pose action unavailable'

        loc = self.locations[location_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = loc.get('frame_id', 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        q = quaternion_from_yaw(float(loc['yaw']))
        goal.pose.pose.orientation.x = q.x
        goal.pose.pose.orientation.y = q.y
        goal.pose.pose.orientation.z = q.z
        goal.pose.pose.orientation.w = q.w

        self.get_logger().info(
            f'navigate to {location_name}: x={loc["x"]} y={loc["y"]} yaw={loc["yaw"]}')
        send_future = self.nav_client.send_goal_async(goal)
        goal_handle = self.wait_future(send_future, timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            return False, 'navigation goal rejected'
        result = self.wait_future(goal_handle.get_result_async(), timeout_sec=120.0)
        if result and result.status == GoalStatus.STATUS_SUCCEEDED:
            if prepare_planning_scene:
                time.sleep(self.post_navigation_scene_delay)
                if not self.prepare_planning_scene_at_current_pose(location_name):
                    return False, 'planning scene preparation failed'
            return True, 'arrived'
        return False, f'navigation status={getattr(result, "status", "timeout")}'

    def prepare_planning_scene_at_current_pose(self, location_name):
        self.clear_manipulation_frame()
        self.begin_manipulation_frame()
        if not self.publish_static_planning_scene():
            self.clear_manipulation_frame()
            return False
        self.planning_scene_location = location_name
        self.get_logger().info(
            f'planning scene prepared at navigation goal: {location_name}')
        return True

    def pick_object(self, object_key):
        self.get_logger().info(f'pick start: {object_key}')
        if not self.has_locked_manipulation_frame():
            self.begin_manipulation_frame()
        else:
            self.get_logger().info(
                f'using planning scene prepared at {self.planning_scene_location or "navigation goal"}')
        try:
            if not self.command_head([0.0, 0.35], 1.0):
                self.get_logger().warn('head command failed, continuing')
            if not self.command_gripper(0.045):
                return False, 'open gripper failed'
            if not self.move_gripper_to_point(object_key, 'pregrasp'):
                return False, 'pregrasp MoveIt plan failed'
            if not self.move_gripper_to_point(object_key, 'grasp', cartesian=True):
                return False, 'grasp MoveIt plan failed'

            self.object_states[object_key]['held'] = True
            self.object_states[object_key]['table'] = ''

            if not self.command_gripper(0.0):
                return False, 'close gripper failed'
            self.move_arm_to_joint_goal('stow')
            self.get_logger().info(f'pick done: {object_key}')
            return True, 'picked'
        finally:
            self.clear_manipulation_frame()

    def place_object(self, object_key, target_location):
        self.get_logger().info(f'place start: {object_key} -> {target_location}')
        if not self.has_locked_manipulation_frame():
            self.begin_manipulation_frame()
        else:
            self.get_logger().info(
                f'using planning scene prepared at {self.planning_scene_location or "navigation goal"}')
        try:
            pose = self.get_table_place_pose(target_location, object_key)
            if pose is None:
                return False, 'target surface missing'
            if not self.move_gripper_to_pose_point(pose, 'preplace'):
                return False, 'preplace MoveIt plan failed'

            self.command_gripper(0.045)
            time.sleep(0.5)
            actual_pose = self.get_world_model_pose(self.object_states[object_key]['model_name'])
            if actual_pose is not None:
                pose.update({'x': actual_pose['x'], 'y': actual_pose['y'], 'z': actual_pose['z']})
            self.object_states[object_key].update({
                'table': target_location,
                'slot': pose.get('slot', 0),
                'held': False,
                'x': pose['x'],
                'y': pose['y'],
                'z': pose['z'],
            })
            self.move_arm_to_joint_goal('carry')
            self.move_arm_to_joint_goal('stow')
            self.get_logger().info(f'place done: {object_key}')
            return True, 'placed'
        finally:
            self.clear_manipulation_frame()

    def arm_pose(self, name):
        poses = {
            'stow': [0.00, 1.32, 0.70, 0.00, -2.00, 0.00, -0.57, 0.00],
            'carry': [0.20, 0.80, 1.00, -0.50, 1.65, 0.00, 1.25, 0.00],
            'pregrasp': [0.25, 0.35, 0.85, -0.35, 1.35, 0.00, 1.00, 0.00],
            'grasp': [0.20, 0.25, 0.92, -0.20, 1.55, 0.00, 0.85, 0.00],
            'preplace': [0.25, -0.25, 0.85, 0.35, 1.35, 0.00, 1.00, 0.00],
            'place': [0.18, -0.20, 0.95, 0.30, 1.55, 0.00, 0.85, 0.00],
        }
        return poses[name]

    def begin_manipulation_frame(self):
        if not self.uses_gazebo_world_frame():
            self.manipulation_world_to_base = self.lookup_world_to_base_transform()
            if self.manipulation_world_to_base is not None:
                transform = self.manipulation_world_to_base.transform
                yaw = yaw_from_ros_quaternion(transform.rotation)
                self.get_logger().info(
                    f'locked manipulation TF: {self.planning_world_frame}->base_link '
                    f'x={transform.translation.x:.3f} y={transform.translation.y:.3f} yaw={yaw:.3f}')
                return

        self.manipulation_robot_pose = self.get_world_model_pose(self.robot_model_name)
        if self.manipulation_robot_pose is None:
            self.get_logger().warn('using live robot pose lookups during manipulation')
            return
        yaw = yaw_from_quaternion(self.manipulation_robot_pose.get('orientation', {}))
        self.get_logger().info(
            f'locked manipulation frame: world x={self.manipulation_robot_pose["x"]:.3f} '
            f'y={self.manipulation_robot_pose["y"]:.3f} yaw={yaw:.3f}')

    def uses_gazebo_world_frame(self):
        return self.planning_world_frame in ('world', 'gazebo_world', 'gz_world', 'ign_world')

    def has_locked_manipulation_frame(self):
        return self.manipulation_world_to_base is not None or self.manipulation_robot_pose is not None

    def clear_manipulation_frame(self):
        self.manipulation_robot_pose = None
        self.manipulation_world_to_base = None
        self.planning_scene_location = ''

    def lookup_world_to_base_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                'base_link', self.planning_world_frame, Time(), timeout=Duration(seconds=1.0))
        except TransformException as exc:
            self.get_logger().warn(
                f'cannot lookup TF {self.planning_world_frame}->base_link, '
                f'falling back to Gazebo pose: {exc}')
            return None

    def publish_static_planning_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_model_name = self.get_parameter('robot_model_name').value
        self.world_pose_cache = self.get_world_poses()
        scene.world.collision_objects = self.build_static_collision_objects(frame_id=self.planning_scene_frame)
        self.world_pose_cache = None

        self.planning_scene_pub.publish(scene)
        if self.apply_planning_scene_client.wait_for_service(timeout_sec=2.0):
            req = ApplyPlanningScene.Request()
            req.scene = scene
            result = self.wait_future(
                self.apply_planning_scene_client.call_async(req), timeout_sec=5.0)
            if result and result.success:
                self.scene_published = True
                self.get_logger().info(
                    f'planning scene applied: {len(scene.world.collision_objects)} static objects')
                return True

        self.scene_published = True
        self.get_logger().warn(
            f'published planning scene diff without service ack: '
            f'{len(scene.world.collision_objects)} static objects')
        return True

    def publish_startup_planning_scene_once(self):
        if self.publish_static_planning_scene():
            self.startup_scene_timer.cancel()
            self.get_logger().info(
                f'startup planning scene loaded in {self.planning_scene_frame} from Gazebo model poses')

    def build_static_collision_objects(self, frame_id='base_link'):
        objects = []
        for table_name, surface in sorted(self.table_surfaces.items()):
            bin_index = self.table_to_bin_index(table_name)
            bin_pose = self.get_world_model_pose(f'bin_{bin_index}') if bin_index is not None else None
            if bin_pose is not None:
                objects.extend(self.build_bin_collision_objects(table_name, bin_pose, frame_id))
            else:
                objects.extend(self.build_configured_table_collision_objects(table_name, surface, frame_id))

            x = float(surface['x'])
            y = float(surface['y'])
            tag_y = y + (0.1875 if y > 0.0 else -0.1875)
            tag_yaw = 0.0 if y > 0.0 else 3.1415
            tag_name = f'{table_name}_tag'
            if table_name.startswith('table_'):
                try:
                    tag_name = f'bin_{int(table_name.split("_", 1)[1]) - 1}_tag'
                except ValueError:
                    pass
            actual_tag_pose = self.get_world_model_pose(tag_name)
            if actual_tag_pose is not None:
                tag_pose = self.world_pose_to_scene_pose(actual_tag_pose, frame_id)
            else:
                tag_pose = self.world_to_scene(x, tag_y, 0.63, tag_yaw, frame_id)
            if tag_pose is not None:
                objects.append(self.make_collision_box(
                    tag_name, frame_id,
                    tag_pose['x'], tag_pose['y'], tag_pose['z'],
                    0.20, 0.01, 0.20, tag_pose['yaw']))
        for wall_name, x, y, z, size_x, size_y, size_z in (
            ('wall_1', 2.125, -1.75, 0.35, 7.75, 0.1, 0.7),
            ('wall_2', -1.75, 0.0, 0.35, 0.1, 3.5, 0.7),
            ('wall_3', 2.125, 1.75, 0.35, 7.75, 0.1, 0.7),
            ('wall_4', 3.0, 1.25, 0.35, 0.1, 1.0, 0.7),
            ('wall_5', 3.0, -1.25, 0.35, 0.1, 1.0, 0.7),
            ('wall_6', 6.0, -1.375, 0.35, 0.1, 0.75, 0.7),
            ('wall_7', 6.0, 0.875, 0.35, 0.1, 1.75, 0.7),
            ('wall_8', 5.0, 0.0, 0.35, 0.1, 3.5, 0.7),
        ):
            wall_pose = self.world_to_scene(x, y, z, 0.0, frame_id)
            if wall_pose is None:
                continue
            objects.append(self.make_collision_box(
                wall_name, frame_id,
                wall_pose['x'], wall_pose['y'], wall_pose['z'],
                size_x, size_y, size_z, wall_pose['yaw']))
        return objects

    def table_to_bin_index(self, table_name):
        if not table_name.startswith('table_'):
            return None
        try:
            return int(table_name.split('_', 1)[1]) - 1
        except ValueError:
            return None

    def build_configured_table_collision_objects(self, table_name, surface, frame_id):
        x = float(surface['x'])
        y = float(surface['y'])
        top_z = float(surface['z'])
        surface_pose = self.world_to_scene(x, y, top_z - 0.02, 0.0, frame_id)
        wall_pose = self.world_to_scene(
            x, y + (0.20 if y > 0.0 else -0.20), top_z + 0.10, 0.0, frame_id)
        left_wall_pose = self.world_to_scene(x - 0.20, y, top_z + 0.005, 0.0, frame_id)
        right_wall_pose = self.world_to_scene(x + 0.20, y, top_z + 0.005, 0.0, frame_id)
        if surface_pose is None or wall_pose is None or left_wall_pose is None or right_wall_pose is None:
            return []
        return [
            self.make_collision_box(
                f'{table_name}_surface', frame_id,
                surface_pose['x'], surface_pose['y'], surface_pose['z'],
                0.40, 0.40, 0.02, surface_pose['yaw']),
            self.make_collision_box(
                f'{table_name}_left_wall', frame_id,
                left_wall_pose['x'], left_wall_pose['y'], left_wall_pose['z'],
                0.02, 0.40, 0.01,
                left_wall_pose['yaw']),
            self.make_collision_box(
                f'{table_name}_right_wall', frame_id,
                right_wall_pose['x'], right_wall_pose['y'], right_wall_pose['z'],
                0.02, 0.40, 0.01,
                right_wall_pose['yaw']),
            self.make_collision_box(
                f'{table_name}_back_wall', frame_id,
                wall_pose['x'], wall_pose['y'], wall_pose['z'],
                0.40, 0.02, 0.20, wall_pose['yaw']),
        ]

    def build_bin_collision_objects(self, table_name, bin_pose, frame_id):
        bin_x = float(bin_pose['x'])
        bin_y = float(bin_pose['y'])
        bin_z = float(bin_pose['z'])
        bin_yaw = yaw_from_quaternion(bin_pose.get('orientation', {}))
        parts = [
            ('surface', 0.0, 0.0, 0.0, 0.40, 0.40, 0.02),
            ('left_wall', -0.20, 0.0, 0.005, 0.02, 0.40, 0.01),
            ('right_wall', 0.20, 0.0, 0.005, 0.02, 0.40, 0.01),
            ('back_wall', 0.0, 0.20, 0.10, 0.40, 0.02, 0.20),
        ]
        objects = []
        for suffix, local_x, local_y, local_z, size_x, size_y, size_z in parts:
            world_x, world_y = self.transform_bin_local_xy(bin_x, bin_y, bin_yaw, local_x, local_y)
            pose = self.world_to_scene(world_x, world_y, bin_z + local_z, bin_yaw, frame_id)
            if pose is None:
                continue
            objects.append(self.make_collision_box(
                f'{table_name}_{suffix}', frame_id,
                pose['x'], pose['y'], pose['z'],
                size_x, size_y, size_z, pose['yaw']))
        return objects

    def transform_bin_local_xy(self, bin_x, bin_y, yaw, local_x, local_y):
        return (
            bin_x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            bin_y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
        )

    def world_pose_to_scene_pose(self, pose, frame_id):
        yaw = yaw_from_quaternion(pose.get('orientation', {}))
        return self.world_to_scene(pose['x'], pose['y'], pose['z'], yaw, frame_id)

    def world_to_scene(self, world_x, world_y, world_z, world_yaw, frame_id):
        if frame_id == 'base_link':
            return self.world_to_base(world_x, world_y, world_z, world_yaw)
        return {
            'x': float(world_x),
            'y': float(world_y),
            'z': float(world_z),
            'yaw': float(world_yaw),
        }

    def make_collision_box(self, object_id, frame_id, x, y, z, size_x, size_y, size_z, yaw=0.0):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size_x), float(size_y), float(size_z)]
        obj.primitives.append(primitive)

        obj.primitive_poses.append(make_pose(x, y, z, yaw))
        return obj

    def move_gripper_to_point(self, object_key, stage, cartesian=False):
        state = self.object_states[object_key]
        self.world_pose_cache = self.get_world_poses()
        try:
            actual_pose = self.get_world_model_pose(state['model_name'])
            if actual_pose is not None:
                state.update({
                    'x': actual_pose['x'],
                    'y': actual_pose['y'],
                    'z': actual_pose['z'],
                })
            return self.move_gripper_to_pose_point(state, stage, cartesian=cartesian)
        finally:
            self.world_pose_cache = None

    def move_gripper_to_pose_point(self, target, stage, cartesian=False):
        target_pose = self.compute_gripper_target(target, stage)
        if target_pose is None:
            return False
        if cartesian:
            return self.send_cartesian_gripper_goal(target_pose, stage)
        return self.send_move_group_pose_goal(
            target_pose['base_x'], target_pose['base_y'], target_pose['base_z'], stage)

    def compute_gripper_target(self, target, stage):
        stage_params = {
            'pregrasp': (self.pregrasp_clearance, self.approach_distance + self.gripper_target_standoff),
            'grasp': (0.0, self.gripper_target_standoff),
            'preplace': (self.place_clearance, self.approach_distance + self.gripper_target_standoff),
            'place': (0.0, self.gripper_target_standoff),
        }
        if stage not in stage_params:
            raise RuntimeError(f'unknown gripper target stage: {stage}')

        robot = self.get_world_model_pose(self.robot_model_name)
        if robot is None:
            self.get_logger().error(f'cannot query robot world pose: {self.robot_model_name}')
            return False

        world_x = float(target['x'])
        world_y = float(target['y'])
        world_z = float(target['z'])
        z_offset, stand_off = stage_params[stage]

        if self.approach_style == 'top_down':
            target_world_x = world_x + self.edge_table_nudge(target)
            target_world_y = world_y + self.top_down_xy_offset * (1.0 if world_y < 0.0 else -1.0)
            if stage in ('grasp', 'place'):
                z_offset += self.top_down_grasp_z_offset
        else:
            approach_dx = robot['x'] - world_x
            approach_dy = robot['y'] - world_y
            approach_norm = math.hypot(approach_dx, approach_dy)
            if approach_norm < 1e-6:
                fallback_sign = -1.0 if world_y >= 0.0 else 1.0
                approach_dx = 0.0
                approach_dy = fallback_sign
                approach_norm = 1.0
            approach_dx /= approach_norm
            approach_dy /= approach_norm
            target_world_x = world_x + stand_off * approach_dx
            target_world_y = world_y + stand_off * approach_dy
        target_world_z = world_z + z_offset

        relative = self.world_to_base(target_world_x, target_world_y, target_world_z)
        if relative is None:
            return False

        x = relative['x']
        y = relative['y']
        z = max(relative['z'], 0.0)
        self.get_logger().info(
            f'MoveIt {stage} ({self.approach_style}): ROS1-style gripper_link target '
            f'object x={world_x:.3f} y={world_y:.3f} z={world_z:.3f}, '
            f'target x={target_world_x:.3f} y={target_world_y:.3f} z={target_world_z:.3f} -> '
            f'base_link x={x:.3f} y={y:.3f} z={z:.3f}')
        return {
            'base_x': x,
            'base_y': y,
            'base_z': z,
            'world_x': target_world_x,
            'world_y': target_world_y,
            'world_z': target_world_z,
        }

    def edge_table_nudge(self, target):
        table_name = target.get('table', '')
        if not table_name or table_name not in self.table_surfaces:
            return 0.0
        table_x = float(self.table_surfaces[table_name]['x'])
        all_x = [float(surface['x']) for surface in self.table_surfaces.values()]
        if not all_x:
            return 0.0
        if table_x <= min(all_x) + 1e-6:
            return self.edge_table_x_offset
        if table_x >= max(all_x) - 1e-6:
            return -self.edge_table_x_offset
        return 0.0

    def world_to_base(self, world_x, world_y, world_z, world_yaw=0.0):
        transform = self.manipulation_world_to_base
        if transform is None and not self.uses_gazebo_world_frame():
            transform = self.lookup_world_to_base_transform()
        if transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rx, ry, rz = rotate_vector_by_quaternion(
                float(world_x), float(world_y), float(world_z), rotation)
            return {
                'x': rx + translation.x,
                'y': ry + translation.y,
                'z': rz + translation.z,
                'yaw': float(world_yaw) + yaw_from_ros_quaternion(rotation),
            }

        robot = self.manipulation_robot_pose or self.get_world_model_pose(self.robot_model_name)
        if robot is None:
            self.get_logger().error(f'cannot query robot world pose: {self.robot_model_name}')
            return None

        yaw = yaw_from_quaternion(robot.get('orientation', {}))
        dx = float(world_x) - robot['x']
        dy = float(world_y) - robot['y']
        return {
            'x': math.cos(yaw) * dx + math.sin(yaw) * dy,
            'y': -math.sin(yaw) * dx + math.cos(yaw) * dy,
            'z': float(world_z) - robot['z'],
            'yaw': float(world_yaw) - yaw,
        }

    def move_arm_to_joint_goal(self, pose_name):
        return self.send_move_group_joint_goal(self.arm_pose(pose_name), pose_name)

    def send_move_group_pose_goal(self, x, y, z, name):
        constraints = Constraints()
        constraints.name = name

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'gripper_link'
        position_constraint.weight = 1.0
        region = SolidPrimitive()
        region.type = SolidPrimitive.BOX
        region.dimensions = [0.06, 0.06, 0.06]
        position_constraint.constraint_region.primitives.append(region)
        position_constraint.constraint_region.primitive_poses.append(make_pose(x, y, z))
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.link_name = 'gripper_link'
        q = quaternion_from_rpy(self.grasp_roll, self.grasp_pitch, self.grasp_yaw)
        orientation_constraint.orientation.x = q.x
        orientation_constraint.orientation.y = q.y
        orientation_constraint.orientation.z = q.z
        orientation_constraint.orientation.w = q.w
        orientation_constraint.absolute_x_axis_tolerance = self.goal_orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.goal_orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.goal_orientation_tolerance
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return self.send_move_group_goal([constraints], timeout_sec=120.0)

    def make_gripper_pose(self, x, y, z):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        q = quaternion_from_rpy(self.grasp_roll, self.grasp_pitch, self.grasp_yaw)
        pose.orientation.x = q.x
        pose.orientation.y = q.y
        pose.orientation.z = q.z
        pose.orientation.w = q.w
        return pose

    def send_cartesian_gripper_goal(self, target_pose, name):
        if not self.compute_cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('MoveIt compute_cartesian_path unavailable')
            return False
        if not self.wait_action_server(
                self.execute_trajectory_client, '/execute_trajectory', timeout_sec=30.0):
            self.get_logger().error('MoveIt execute_trajectory unavailable')
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state.is_diff = True
        req.group_name = self.move_group_name
        req.link_name = 'gripper_link'
        req.waypoints = [
            self.make_gripper_pose(
                target_pose['base_x'], target_pose['base_y'], target_pose['base_z'])
        ]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        result = self.wait_future(
            self.compute_cartesian_path_client.call_async(req), timeout_sec=10.0)
        if result is None:
            self.get_logger().error(f'Cartesian {name} path timeout')
            return False
        if result.error_code.val != MoveItErrorCodes.SUCCESS or result.fraction < 0.99:
            self.get_logger().error(
                f'Cartesian {name} path failed, fraction={result.fraction:.3f}, '
                f'error_code={result.error_code.val}')
            return False
        if not result.solution.joint_trajectory.points:
            self.get_logger().error(f'Cartesian {name} path is empty')
            return False
        self.ensure_trajectory_timing(result.solution.joint_trajectory, 0.12)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution
        goal_handle = self.wait_future(
            self.execute_trajectory_client.send_goal_async(goal), timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Cartesian {name} execution rejected')
            return False
        execute_result = self.wait_future(goal_handle.get_result_async(), timeout_sec=60.0)
        if not execute_result:
            self.get_logger().error(f'Cartesian {name} execution timeout')
            return False
        error_code = execute_result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Cartesian {name} execution failed, error_code={error_code}')
            return False
        self.get_logger().info(f'Cartesian {name} path executed, fraction={result.fraction:.3f}')
        return True

    def ensure_trajectory_timing(self, joint_trajectory, point_dt):
        has_timing = any(
            point.time_from_start.sec or point.time_from_start.nanosec
            for point in joint_trajectory.points)
        if has_timing:
            return
        for index, point in enumerate(joint_trajectory.points, start=1):
            total = float(index) * float(point_dt)
            point.time_from_start.sec = int(total)
            point.time_from_start.nanosec = int((total - int(total)) * 1e9)

    def send_move_group_joint_goal(self, positions, name):
        joint_names = [
            'torso_lift_joint',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
        ]
        if self.move_group_name == 'arm':
            joint_names = joint_names[1:]
            positions = positions[1:]
        constraints = Constraints()
        constraints.name = name
        for joint_name, position in zip(joint_names, positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.03
            joint_constraint.tolerance_below = 0.03
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return self.send_move_group_goal([constraints], timeout_sec=120.0)

    def send_move_group_goal(self, goal_constraints, timeout_sec):
        if not self.wait_action_server(self.move_group_client, '/move_action', timeout_sec=60.0):
            self.get_logger().error('MoveIt move_action unavailable')
            return False

        goal = MoveGroup.Goal()
        goal.request.group_name = self.move_group_name
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 8
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = 0.45
        goal.request.max_acceleration_scaling_factor = 0.35
        goal.request.goal_constraints = goal_constraints
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 0.2

        goal_handle = self.wait_future(
            self.move_group_client.send_goal_async(goal), timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return False
        result = self.wait_future(goal_handle.get_result_async(), timeout_sec=timeout_sec)
        if not result:
            self.get_logger().error('MoveIt goal timeout')
            return False
        error_code = result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'MoveIt goal failed, error_code={error_code}')
            return False
        return True

    def command_gripper(self, opening):
        opening = max(0.0, min(float(opening), 0.05))
        return self.send_trajectory(
            self.gripper_client,
            ['l_gripper_finger_joint', 'r_gripper_finger_joint'],
            [opening, opening],
            1.0)

    def command_head(self, positions, duration_sec):
        return self.send_trajectory(
            self.head_client,
            ['head_pan_joint', 'head_tilt_joint'],
            positions,
            duration_sec)

    def send_trajectory(self, client, joint_names, positions, duration_sec):
        action_name = getattr(client, '_action_name', 'trajectory action')
        if not self.wait_action_server(client, action_name, timeout_sec=60.0):
            self.get_logger().error(f'action unavailable: {action_name}')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points.append(point)

        goal_handle = self.wait_future(client.send_goal_async(goal), timeout_sec=10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'trajectory rejected by {client._action_name}')
            return False
        result = self.wait_future(goal_handle.get_result_async(), timeout_sec=max(10.0, duration_sec + 8.0))
        if not result:
            return False
        return result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def get_table_place_pose(self, table_name, object_key):
        surface = self.table_surfaces.get(table_name)
        if not surface:
            return None
        count = 0
        for key, state in self.object_states.items():
            if key == object_key:
                continue
            if state.get('table') == table_name and not state.get('held', False):
                count += 1
        dx, dy = slot_offset(count)
        return {
            'x': float(surface['x']) + dx,
            'y': float(surface['y']) + dy,
            'z': float(surface['z']) + BOX_SIZE / 2.0,
            'table': table_name,
            'slot': count,
        }

    def get_world_model_pose(self, model_name):
        poses = self.world_pose_cache if self.world_pose_cache is not None else self.get_world_poses()
        pose = poses.get(model_name)
        if pose is None:
            self.get_logger().warn(f'model pose not found in /world/{self.world_name}/pose/info: {model_name}')
        return pose

    def get_world_poses(self):
        cmd = ['ign', 'topic', '-e', '-t', f'/world/{self.world_name}/pose/info', '-n', '1']
        try:
            result = subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, timeout=2.0)
        except Exception as exc:
            self.get_logger().warn(f'ign pose topic exception: {exc}')
            return {}

        if result.returncode != 0:
            self.get_logger().warn(
                f'ign pose topic failed: {result.stdout} {result.stderr}')
            return {}
        return self.parse_pose_info(result.stdout)

    def parse_pose_info(self, text):
        poses = {}
        lines = text.splitlines()
        index = 0
        while index < len(lines):
            if lines[index].strip() != 'pose {':
                index += 1
                continue

            block = []
            depth = 0
            while index < len(lines):
                line = lines[index]
                depth += line.count('{')
                depth -= line.count('}')
                block.append(line)
                index += 1
                if depth == 0:
                    break

            pose = self.parse_pose_block(block)
            if pose and pose.get('name'):
                poses[pose['name']] = pose
        return poses

    def parse_pose_block(self, block):
        pose = {
            'name': '',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        }
        section = None
        for raw_line in block:
            line = raw_line.strip()
            if line.startswith('name:'):
                pose['name'] = line.split(':', 1)[1].strip().strip('"')
            elif line == 'position {':
                section = 'position'
            elif line == 'orientation {':
                section = 'orientation'
            elif line == '}':
                section = None
            elif section == 'position' and ':' in line:
                key, value = line.split(':', 1)
                if key.strip() in ('x', 'y', 'z'):
                    pose[key.strip()] = float(value.strip())
            elif section == 'orientation' and ':' in line:
                key, value = line.split(':', 1)
                if key.strip() in ('x', 'y', 'z', 'w'):
                    pose['orientation'][key.strip()] = float(value.strip())
        return pose

    def wait_action_server(self, client, action_name, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        logged_wait = False
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=1.0):
                return True
            if not logged_wait:
                self.get_logger().info(f'waiting for action server: {action_name}')
                logged_wait = True
        return False

    def wait_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()


def main():
    rclpy.init()
    node = TaskDispatcher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
