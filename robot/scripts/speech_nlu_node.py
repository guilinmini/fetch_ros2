#!/usr/bin/env python3

import re
from collections import OrderedDict

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from common import PUNCTUATION_RE, TABLE_RE, chinese_number_to_int
from robot.srv import SpeechNLUSrv, SpeechSrv


OBJECT_SYNONYMS = OrderedDict([
    ('glue', ['胶水']),
    ('book', ['书本', '书']),
    ('ball', ['小球', '球']),
    ('tissue', ['纸巾', '抽纸']),
    ('snack', ['零食']),
    ('cup', ['水杯', '杯子']),
    ('parcel', ['快递盒', '盒子']),
    ('scissors', ['剪刀']),
    ('pencil', ['铅笔']),
    ('coat', ['外套']),
    ('mouse', ['鼠标']),
    ('charger', ['充电器']),
])

PICK_WORDS = ['抓', '抓取', '拿', '拿起', '取', '取走', '拿走', '拿来', '取来']
PLACE_WORDS = ['放', '放到', '放在', '送到', '搬到', '移到', '拿到']
NAVIGATE_WORDS = ['去', '前往', '到']

SOURCE_PATTERNS = [
    re.compile(r'从([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌)'),
    re.compile(r'把([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌)(?:上)?的'),
    re.compile(r'在([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌).{0,8}(?:抓取|拿|取)'),
]

TARGET_PATTERNS = [
    re.compile(r'(?:放到|放在|送到|搬到|移到|拿到)([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌)'),
    re.compile(r'(?:去|前往|到)([零一二两三四五六七八九十\d]{1,3})\s*(?:号)?\s*(?:桌子|桌)'),
]

NORMALIZE_REPLACEMENTS = [
    ('桌子', '桌'),
    ('拿起', '抓取'),
    ('拿走', '抓取'),
    ('拿来', '抓取'),
    ('取来', '抓取'),
    ('取走', '抓取'),
    ('拿到', '放到'),
    ('送到', '放到'),
    ('搬到', '放到'),
    ('移到', '放到'),
]


class SpeechNLUNode(Node):
    def __init__(self):
        super().__init__('speech_nlu_node')
        self.callback_group = ReentrantCallbackGroup()
        self.asr_client = self.create_client(
            SpeechSrv, '/speech_service', callback_group=self.callback_group)
        self.service = self.create_service(
            SpeechNLUSrv, '/speech_nlu_service', self.handle_request,
            callback_group=self.callback_group)
        self.get_logger().info('speech NLU service ready: /speech_nlu_service')

    def handle_request(self, request, response):
        if not self.asr_client.wait_for_service(timeout_sec=10.0):
            return self.fail(response, 'ASR service unavailable')

        asr_req = SpeechSrv.Request()
        asr_req.file_path = request.file_path
        future = self.asr_client.call_async(asr_req)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)

        asr_response = future.result()
        if not asr_response or not asr_response.success:
            return self.fail(response, f'ASR failed: {getattr(asr_response, "message", "")}')

        raw_text = str(asr_response.text).strip()
        normalized = self.normalize_text(raw_text)
        target_object, actions, source_location, target_location = self.parse_text(normalized)
        ok, message = self.validate_parse(actions, target_object, source_location, target_location)

        response.success = ok
        response.raw_text = raw_text
        response.target_object = target_object
        response.actions = actions
        response.source_location = source_location
        response.target_location = target_location
        response.message = message
        self.get_logger().info(
            f'nlu raw="{raw_text}" actions={actions} object={target_object} '
            f'src={source_location} dst={target_location}')
        return response

    def fail(self, response, message):
        response.success = False
        response.raw_text = ''
        response.target_object = ''
        response.actions = []
        response.source_location = ''
        response.target_location = ''
        response.message = message
        return response

    def normalize_text(self, text):
        text = PUNCTUATION_RE.sub('', str(text or ''))
        for old, new in NORMALIZE_REPLACEMENTS:
            text = text.replace(old, new)
        return text

    def parse_text(self, text):
        target_object = self.parse_object(text)
        actions = self.parse_actions(text)
        source_location, target_location = self.parse_locations(text)
        if target_object and source_location and target_location and 'place' in actions and 'pick' not in actions:
            actions.insert(0, 'pick')
        return target_object, actions, source_location, target_location

    def parse_object(self, text):
        for object_key, aliases in OBJECT_SYNONYMS.items():
            if any(alias in text for alias in aliases):
                return object_key
        return ''

    def parse_actions(self, text):
        actions = []
        if any(word in text for word in PICK_WORDS):
            actions.append('pick')
        if any(word in text for word in PLACE_WORDS):
            actions.append('place')
        if not actions and any(word in text for word in NAVIGATE_WORDS):
            actions.append('navigate')
        return actions

    def parse_locations(self, text):
        source_num = self.extract_num_by_patterns(text, SOURCE_PATTERNS)
        target_num = self.extract_num_by_patterns(text, TARGET_PATTERNS)
        ordered_nums = self.extract_all_table_nums(text)

        if source_num is None and ordered_nums:
            source_num = ordered_nums[0]
        if target_num is None and len(ordered_nums) >= 2:
            target_num = ordered_nums[1]
        elif target_num is None and len(ordered_nums) == 1:
            target_num = ordered_nums[0]

        if source_num is not None and target_num is not None and source_num == target_num:
            unique_nums = []
            for value in ordered_nums:
                if value not in unique_nums:
                    unique_nums.append(value)
            if len(unique_nums) >= 2:
                source_num = unique_nums[0]
                target_num = unique_nums[1]

        source_location = f'table_{source_num}' if source_num is not None else ''
        target_location = f'table_{target_num}' if target_num is not None else ''
        return source_location, target_location

    def extract_num_by_patterns(self, text, patterns):
        for pattern in patterns:
            match = pattern.search(text)
            if match:
                value = chinese_number_to_int(match.group(1))
                if value is not None:
                    return value
        return None

    def extract_all_table_nums(self, text):
        ordered = []
        for match in TABLE_RE.finditer(text):
            value = chinese_number_to_int(match.group(1))
            if value is not None and value not in ordered:
                ordered.append(value)
        return ordered

    def validate_parse(self, actions, target_object, source_location, target_location):
        if not actions:
            return False, '无法识别任务动作'
        if 'pick' in actions and not target_object:
            return False, '无法识别目标物体'
        if 'pick' in actions and not source_location:
            return False, '无法识别取物桌位'
        if 'place' in actions and not target_location:
            return False, '无法识别放置桌位'
        if 'navigate' in actions and not (target_location or source_location):
            return False, '无法识别导航桌位'
        return True, 'ok'


def main():
    rclpy.init()
    node = SpeechNLUNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
