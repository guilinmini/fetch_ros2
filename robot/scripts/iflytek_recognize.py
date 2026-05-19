#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import ssl
import json
import time
import base64
import hmac
import hashlib
import tempfile
import subprocess
import threading
from datetime import datetime
from wsgiref.handlers import format_date_time
from time import mktime
try:
    from urllib.parse import urlencode
except ImportError:
    from urllib import urlencode

import websocket

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None



def resolve_config_path():
    if get_package_share_directory is not None:
        try:
            package_path = get_package_share_directory("robot")
            config_path = os.path.join(package_path, "config", "iflytek_asr.json")
            if os.path.exists(config_path):
                return config_path
        except Exception:
            pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "iflytek_asr.json")

def make_result(success, text="", message=""):
    return {
        "success": bool(success),
        "text": text,
        "message": message
    }


def load_config():
    config_path = resolve_config_path()

    if not os.path.exists(config_path):
        raise IOError("Config file not found: %s" % config_path)

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    required = ["APPID", "APIKey", "APISecret", "host", "request_path", "language", "domain", "accent", "sample_rate"]
    for k in required:
        if not cfg.get(k):
            raise ValueError("Missing config field: %s" % k)

    return cfg


def build_auth_url(cfg):
    host = cfg["host"]
    request_path = cfg["request_path"]
    api_key = cfg["APIKey"]
    api_secret = cfg["APISecret"]

    now = datetime.utcnow()
    date = format_date_time(mktime(now.timetuple()))

    signature_origin = "host: {host}\ndate: {date}\nGET {path} HTTP/1.1".format(
        host=host,
        date=date,
        path=request_path
    )

    signature_sha = hmac.new(
        api_secret.encode("utf-8"),
        signature_origin.encode("utf-8"),
        digestmod=hashlib.sha256
    ).digest()

    signature = base64.b64encode(signature_sha).decode("utf-8")

    authorization_origin = (
        'api_key="{api_key}", algorithm="hmac-sha256", '
        'headers="host date request-line", signature="{signature}"'
    ).format(api_key=api_key, signature=signature)

    authorization = base64.b64encode(
        authorization_origin.encode("utf-8")
    ).decode("utf-8")

    query = urlencode({
        "authorization": authorization,
        "date": date,
        "host": host
    })

    return "wss://{host}{path}?{query}".format(
        host=host,
        path=request_path,
        query=query
    )


def convert_audio_to_pcm16k(input_file, sample_rate):
    if not os.path.exists(input_file):
        raise IOError("Audio file does not exist: %s" % input_file)

    fd, pcm_path = tempfile.mkstemp(prefix="iflytek_", suffix=".pcm")
    os.close(fd)

    cmd = [
        "ffmpeg",
        "-y",
        "-loglevel", "error",
        "-i", input_file,
        "-ac", "1",
        "-ar", str(sample_rate),
        "-f", "s16le",
        "-acodec", "pcm_s16le",
        pcm_path
    ]

    proc = subprocess.run(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        universal_newlines=True
    )

    if proc.returncode != 0 or not os.path.exists(pcm_path):
        try:
            if os.path.exists(pcm_path):
                os.remove(pcm_path)
        except Exception:
            pass
        raise RuntimeError("ffmpeg conversion failed: %s" % proc.stderr.strip())

    return pcm_path


class IflytekSession(object):
    def __init__(self, cfg, pcm_path):
        self.cfg = cfg
        self.pcm_path = pcm_path
        self.url = build_auth_url(cfg)

        self.done = False
        self.error = None
        self.fragments = []

    def _extract_text(self, obj):
        try:
            ws_list = obj["data"]["result"]["ws"]
        except Exception:
            return ""

        parts = []
        for ws_item in ws_list:
            cw_list = ws_item.get("cw", [])
            if cw_list:
                parts.append(cw_list[0].get("w", ""))
        return "".join(parts)

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            code = data.get("code", -1)
            if code != 0:
                self.error = data.get("message", "iflytek error code: %s" % code)
                self.done = True
                ws.close()
                return

            text_piece = self._extract_text(data)
            if text_piece:
                self.fragments.append(text_piece)

            if data.get("data", {}).get("status") == 2:
                self.done = True
                ws.close()

        except Exception as e:
            self.error = "parse message failed: %s" % str(e)
            self.done = True
            ws.close()

    def on_error(self, ws, error):
        self.error = str(error)
        self.done = True

    def on_close(self, ws, close_status_code, close_msg):
        self.done = True

    def on_open(self, ws):
        def run():
            frame_size = 1280
            interval = 0.04
            status = 0
            sample_rate = self.cfg["sample_rate"]

            try:
                with open(self.pcm_path, "rb") as f:
                    while True:
                        chunk = f.read(frame_size)

                        if not chunk:
                            end_frame = {
                                "data": {
                                    "status": 2
                                }
                            }
                            ws.send(json.dumps(end_frame))
                            break

                        audio_b64 = base64.b64encode(chunk).decode("utf-8")

                        if status == 0:
                            payload = {
                                "common": {
                                    "app_id": self.cfg["APPID"]
                                },
                                "business": {
                                    "language": self.cfg["language"],
                                    "domain": self.cfg["domain"],
                                    "accent": self.cfg["accent"]
                                },
                                "data": {
                                    "status": 0,
                                    "format": "audio/L16;rate=%d" % sample_rate,
                                    "encoding": "raw",
                                    "audio": audio_b64
                                }
                            }
                            status = 1
                        else:
                            payload = {
                                "data": {
                                    "status": 1,
                                    "format": "audio/L16;rate=%d" % sample_rate,
                                    "encoding": "raw",
                                    "audio": audio_b64
                                }
                            }

                        ws.send(json.dumps(payload))
                        time.sleep(interval)

            except Exception as e:
                self.error = "send audio failed: %s" % str(e)
                self.done = True
                ws.close()

        threading.Thread(target=run, daemon=True).start()

    def run(self):
        ws = websocket.WebSocketApp(
            self.url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )

        ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

        if self.error:
            raise RuntimeError(self.error)

        return "".join(self.fragments).strip()


def recognize(audio_path):
    cfg = load_config()
    pcm_path = None
    try:
        pcm_path = convert_audio_to_pcm16k(audio_path, cfg["sample_rate"])
        session = IflytekSession(cfg, pcm_path)
        text = session.run()
        return text
    finally:
        if pcm_path and os.path.exists(pcm_path):
            try:
                os.remove(pcm_path)
            except Exception:
                pass


def main():
    if len(sys.argv) != 2:
        out = make_result(False, "", "Usage: python3 iflytek_recognize.py <audio_path>")
        sys.stdout.write(json.dumps(out, ensure_ascii=False))
        sys.stdout.flush()
        sys.exit(1)

    audio_path = sys.argv[1]

    try:
        text = recognize(audio_path)
        out = make_result(True, text, "ok")
        sys.stdout.write(json.dumps(out, ensure_ascii=False))
        sys.stdout.flush()
        sys.exit(0)
    except Exception as e:
        out = make_result(False, "", str(e))
        sys.stdout.write(json.dumps(out, ensure_ascii=False))
        sys.stdout.flush()
        sys.exit(1)


if __name__ == "__main__":
    main()
