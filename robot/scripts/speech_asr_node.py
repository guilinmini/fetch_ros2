#!/usr/bin/env python3

import json
import os
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from common import resolve_audio_path
from robot.srv import SpeechSrv


class SpeechASRNode(Node):
    def __init__(self):
        super().__init__('speech_asr_node')
        self.declare_parameter('fallback_text', '从一号桌抓取胶水放到六号桌')
        self.declare_parameter('python3_bin', '/usr/bin/python3')
        self.robot_share = get_package_share_directory('robot')
        self.iflytek_script = os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), 'iflytek_recognize.py')
        self.service = self.create_service(SpeechSrv, '/speech_service', self.handle_request)
        self.get_logger().info('speech ASR service ready: /speech_service')

    def handle_request(self, request, response):
        file_path = resolve_audio_path(request.file_path.strip(), self.robot_share)
        fallback_text = self.get_parameter('fallback_text').value

        if not file_path or not os.path.exists(file_path):
            response.text = fallback_text
            response.success = True
            response.message = f'using fallback text because audio file was not found: {request.file_path}'
            self.get_logger().warn(response.message)
            return response

        result = self.try_iflytek(file_path)
        if result and result.get('success') and result.get('text'):
            response.text = str(result.get('text', ''))
            response.success = True
            response.message = str(result.get('message', 'ok'))
            self.get_logger().info(f'asr result: {response.text}')
            return response

        response.text = fallback_text
        response.success = True
        detail = result.get('message', '') if result else 'iflytek helper unavailable'
        response.message = f'using fallback text: {detail}'
        self.get_logger().warn(response.message)
        return response

    def try_iflytek(self, file_path):
        if not os.path.exists(self.iflytek_script):
            return None

        cmd = [self.get_parameter('python3_bin').value, self.iflytek_script, file_path]
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=20.0)
            return json.loads(output.decode('utf-8', 'ignore').strip())
        except Exception as exc:
            return {'success': False, 'text': '', 'message': str(exc)}


def main():
    rclpy.init()
    node = SpeechASRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
