#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from common import load_task_config


class LabelPublisher(Node):
    def __init__(self):
        super().__init__('label_publisher')
        self.declare_parameter('config_dir', '')
        config = load_task_config(self.get_parameter('config_dir').value)
        self.table_surfaces = config.get('table_surfaces', {})
        self.objects = config.get('objects', {})
        self.publisher = self.create_publisher(MarkerArray, '/workspace_labels', 1)
        self.timer = self.create_timer(1.0, self.publish_labels)

    def publish_labels(self):
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        marker_id = 0
        for table_name in sorted(self.table_surfaces.keys(), key=self.sort_table_name):
            surface = self.table_surfaces[table_name]
            markers.markers.append(self.make_text_marker(
                marker_id, 'table_labels', surface.get('frame_id', 'world'),
                self.pretty_table_name(table_name),
                float(surface['x']), float(surface['y']), float(surface['z']) + 0.18,
                0.16, (1.0, 1.0, 1.0, 1.0)))
            marker_id += 1

        for object_key in sorted(self.objects.keys()):
            config = self.objects[object_key]
            table_name = config.get('table', '')
            surface = self.table_surfaces.get(table_name)
            if not surface:
                continue
            markers.markers.append(self.make_text_marker(
                marker_id, 'object_labels', surface.get('frame_id', 'world'),
                config.get('display_name', object_key),
                float(surface['x']), float(surface['y']), float(surface['z']) + 0.30,
                0.12, (1.0, 1.0, 0.0, 1.0)))
            marker_id += 1

        self.publisher.publish(markers)

    def make_text_marker(self, marker_id, namespace, frame_id, text, x, y, z, scale, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = str(text)
        return marker

    def pretty_table_name(self, table_name):
        try:
            return f'{int(table_name.split("_")[-1])}号桌'
        except Exception:
            return table_name

    def sort_table_name(self, name):
        try:
            return int(name.split('_')[-1])
        except Exception:
            return 9999


def main():
    rclpy.init()
    node = LabelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
