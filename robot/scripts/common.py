#!/usr/bin/env python3

import math
import os
import re

import yaml
from geometry_msgs.msg import Pose


BOX_SIZE = 0.06


def quaternion_from_yaw(yaw):
    pose_q = type('QuaternionTuple', (), {})()
    pose_q.x = 0.0
    pose_q.y = 0.0
    pose_q.z = math.sin(yaw * 0.5)
    pose_q.w = math.cos(yaw * 0.5)
    return pose_q


def make_pose(x, y, z, yaw=0.0):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    q = quaternion_from_yaw(float(yaw))
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.orientation.w = q.w
    return pose


def load_yaml_file(path):
    with open(path, 'r', encoding='utf-8') as file:
        return yaml.safe_load(file) or {}


def load_task_config(config_dir):
    data = {}
    for name in (
        'location_map.yaml',
        'table_surface_map.yaml',
        'object_instances.yaml',
        'manipulation.yaml',
    ):
        path = os.path.join(config_dir, name)
        if os.path.exists(path):
            data.update(load_yaml_file(path))
    return data


def make_box_sdf(name, color, size=BOX_SIZE):
    r, g, b, a = color
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <self_collide>false</self_collide>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.000015</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000015</iyy><iyz>0</iyz><izz>0.000015</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def color_for_index(idx):
    colors = [
        (1.0, 0.2, 0.2, 1.0),
        (0.2, 0.6, 1.0, 1.0),
        (1.0, 0.8, 0.2, 1.0),
        (0.2, 1.0, 0.4, 1.0),
        (0.8, 0.2, 1.0, 1.0),
        (1.0, 0.5, 0.1, 1.0),
        (0.3, 0.9, 0.9, 1.0),
        (0.8, 0.8, 0.8, 1.0),
        (0.5, 0.3, 0.1, 1.0),
        (1.0, 0.4, 0.7, 1.0),
        (0.3, 0.3, 0.3, 1.0),
        (0.1, 0.8, 0.6, 1.0),
    ]
    return colors[idx % len(colors)]


def slot_offset(slot_index):
    offsets = [
        (0.00, 0.00),
        (0.08, 0.00),
        (-0.08, 0.00),
        (0.00, 0.08),
        (0.00, -0.08),
    ]
    if slot_index < len(offsets):
        return offsets[slot_index]
    return (0.0, 0.0)


def resolve_audio_path(file_path, package_share):
    if file_path and os.path.exists(file_path):
        return file_path

    basename = os.path.basename(file_path or '')
    candidates = [
        os.path.join(package_share, 'test', basename),
        os.path.join(package_share, 'test', 'test.wav'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return file_path


PUNCTUATION_RE = re.compile(r"[\s,，。.;；:：!！?？、]")
TABLE_RE = re.compile(r"([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌)")


def chinese_number_to_int(token):
    token = str(token or '').strip()
    if not token:
        return None
    if re.match(r"^\d+$", token):
        value = int(token)
        return value if 1 <= value <= 12 else None

    basic = {
        '零': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
        '五': 5, '六': 6, '七': 7, '八': 8, '九': 9, '十': 10,
    }
    if token in basic and token != '零':
        value = basic[token]
        return value if 1 <= value <= 12 else None
    if token == '十一':
        return 11
    if token == '十二':
        return 12
    if token.startswith('十') and len(token) == 2:
        value = 10 + basic.get(token[1], 0)
        return value if 1 <= value <= 12 else None
    if token.endswith('十') and len(token) == 2:
        value = basic.get(token[0], 0) * 10
        return value if 1 <= value <= 12 else None
    return None
