#!/usr/bin/env python3

import json
import os
import shutil
import subprocess
import sys
import tempfile

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
        self.declare_parameter('record_seconds', 5.0)
        self.declare_parameter('record_device', '')
        self.robot_share = get_package_share_directory('robot')
        self.iflytek_script = os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), 'iflytek_recognize.py')
        self.service = self.create_service(SpeechSrv, '/speech_service', self.handle_request)
        self.get_logger().info('speech ASR service ready: /speech_service')

    def handle_request(self, request, response):
        requested_path = request.file_path.strip()
        recorded_path = ''
        if requested_path:
            file_path = resolve_audio_path(requested_path, self.robot_share)
        else:
            recorded_path = self.record_microphone_audio()
            file_path = recorded_path
        fallback_text = self.get_parameter('fallback_text').value

        try:
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
        finally:
            if recorded_path and os.path.exists(recorded_path):
                try:
                    os.remove(recorded_path)
                except OSError:
                    pass

    def record_microphone_audio(self):
        arecord = shutil.which('arecord')
        if not arecord:
            self.get_logger().warn('arecord not found; using fallback text')
            return ''

        duration = max(1.0, float(self.get_parameter('record_seconds').value))
        device = str(self.get_parameter('record_device').value).strip()
        fd, wav_path = tempfile.mkstemp(prefix='speech_asr_', suffix='.wav')
        os.close(fd)

        cmd = [
            arecord,
            '-q',
            '-f', 'S16_LE',
            '-r', '16000',
            '-c', '1',
            '-d', str(int(round(duration))),
        ]
        if device:
            cmd.extend(['-D', device])
        cmd.append(wav_path)

        self.get_logger().info(f'recording microphone for {duration:.1f}s')
        try:
            subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=duration + 3.0)
            return wav_path
        except Exception as exc:
            if os.path.exists(wav_path):
                try:
                    os.remove(wav_path)
                except OSError:
                    pass
            self.get_logger().warn(f'microphone recording failed: {exc}')
            return ''

    def try_iflytek(self, file_path):
        if not os.path.exists(self.iflytek_script):
            return None

        cmd = [self.get_parameter('python3_bin').value, self.iflytek_script, file_path]
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=20.0)
            return json.loads(output.decode('utf-8', 'ignore').strip())
        except subprocess.CalledProcessError as exc:
            output = exc.output.decode('utf-8', 'ignore').strip() if exc.output else ''
            try:
                parsed = json.loads(output)
                return {
                    'success': False,
                    'text': parsed.get('text', ''),
                    'message': parsed.get('message', output or str(exc)),
                }
            except Exception:
                return {'success': False, 'text': '', 'message': output or str(exc)}
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
