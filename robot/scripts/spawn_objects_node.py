#!/usr/bin/env python3

import subprocess
import rclpy
import time
from rclpy.node import Node

from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity

from common import BOX_SIZE, color_for_index, load_task_config, make_box_sdf, make_pose, slot_offset


class SpawnObjectsNode(Node):
    def __init__(self):
        super().__init__('spawn_objects_node')
        self.declare_parameter('world_name', 'aisle')
        self.declare_parameter('config_dir', '')
        self.world_name = self.get_parameter('world_name').value
        config = load_task_config(self.get_parameter('config_dir').value)

        self.table_surfaces = config.get('table_surfaces', {})
        self.objects = config.get('objects', {})

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        self.delete_client = self.create_client(DeleteEntity, f'/world/{self.world_name}/remove')
        self.pose_client = self.create_client(SetEntityPose, f'/world/{self.world_name}/set_pose')

        self.get_logger().info('waiting for Ignition entity services...')
        if not self.wait_for_service(self.spawn_client, f'/world/{self.world_name}/create', 15.0):
            self.get_logger().warn('spawn service timeout; falling back to ign service')
        self.wait_for_service(self.pose_client, f'/world/{self.world_name}/set_pose', 10.0)
        self.wait_for_service(self.delete_client, f'/world/{self.world_name}/remove', 5.0)

        self.spawn_all_objects()
        self.get_logger().info(f'spawned/reset {len(self.objects)} voice task objects')

    def wait_for_service(self, client, name, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
            self.get_logger().info(f'waiting for {name}...')
        return client.service_is_ready()

    def call_and_wait(self, client, request, timeout_sec=5.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > deadline:
                return None
        return future.result() if future.done() else None

    def spawn_all_objects(self):
        table_counts = {}
        for idx, object_key in enumerate(sorted(self.objects.keys())):
            config = self.objects[object_key]
            table_name = config.get('table', '')
            surface = self.table_surfaces.get(table_name)
            if not surface:
                self.get_logger().warn(f'skip {object_key}: table {table_name} missing')
                continue

            model_name = config.get('model_name', object_key)
            slot = table_counts.get(table_name, 0)
            table_counts[table_name] = slot + 1
            dx, dy = slot_offset(slot)
            x = float(surface['x']) + dx
            y = float(surface['y']) + dy
            z = float(surface['z']) + BOX_SIZE / 2.0

            self.delete_model(model_name)
            spawned = self.spawn_model(model_name, object_key, idx, x, y, z)
            if not spawned:
                self.set_model_pose(model_name, x, y, z)
            self.get_logger().info(f'{model_name} ready on {table_name}')

    def delete_model(self, model_name):
        if not self.delete_client.service_is_ready():
            self.delete_model_with_ign(model_name)
            return
        req = DeleteEntity.Request()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL
        self.call_and_wait(self.delete_client, req, timeout_sec=1.0)

    def spawn_model(self, model_name, object_key, idx, x, y, z):
        if not self.spawn_client.service_is_ready():
            return self.spawn_model_with_ign(model_name, object_key, idx, x, y, z)
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = model_name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = make_box_sdf(object_key, color_for_index(idx))
        req.entity_factory.pose = make_pose(x, y, z)
        req.entity_factory.relative_to = 'world'
        result = self.call_and_wait(self.spawn_client, req, timeout_sec=5.0)
        if result and result.success:
            return True
        return self.spawn_model_with_ign(model_name, object_key, idx, x, y, z)

    def set_model_pose(self, model_name, x, y, z):
        if not self.pose_client.service_is_ready():
            return False
        req = SetEntityPose.Request()
        req.entity.name = model_name
        req.entity.type = Entity.MODEL
        req.pose = make_pose(x, y, z)
        result = self.call_and_wait(self.pose_client, req, timeout_sec=2.0)
        return bool(result and result.success)

    def delete_model_with_ign(self, model_name):
        cmd = [
            'ign', 'service',
            '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'ignition.msgs.Entity',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "{model_name}" type: MODEL',
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
        except Exception:
            pass

    def spawn_model_with_ign(self, model_name, object_key, idx, x, y, z):
        sdf = make_box_sdf(object_key, color_for_index(idx)).replace('\\', '\\\\')
        sdf = sdf.replace('"', '\\"').replace('\n', '')
        request = (
            f'name: "{model_name}" '
            f'sdf: "{sdf}" '
            f'pose {{ position {{ x: {float(x)} y: {float(y)} z: {float(z)} }} '
            'orientation { x: 0 y: 0 z: 0 w: 1 } } '
            'allow_renaming: false'
        )
        cmd = [
            'ign', 'service',
            '-s', f'/world/{self.world_name}/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', request,
        ]
        try:
            result = subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3.0)
        except Exception as exc:
            self.get_logger().warn(f'ign spawn exception for {model_name}: {exc}')
            return False
        if result.returncode == 0 and 'true' in result.stdout.lower():
            return True
        self.get_logger().warn(
            f'ign spawn failed for {model_name}: {result.stdout} {result.stderr}')
        return False


def main():
    rclpy.init()
    node = SpawnObjectsNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
