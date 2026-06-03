#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import html
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
except ImportError:
    print("[ERROR] OpenCV(cv2) is required. Install python3-opencv on the Raspberry Pi.")
    sys.exit(1)

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 20
DEFAULT_PORT = 8000
DEFAULT_JPEG_QUALITY = 80
DEFAULT_EXPOSURE_VALUE = -1.0


class CameraWorker:
    def __init__(self, args):
        self.args = args
        self.camera = None
        self.backend = "unknown"
        self.running = threading.Event()
        self.thread = None
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_error = ""
        self.frames_encoded = 0
        self.last_frame_time = 0.0

    def start(self):
        self.camera = self._open_camera()
        if self.camera is None:
            return False

        self.running.set()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running.clear()

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        if self.camera is not None:
            if self.backend == "picamera2":
                self.camera.stop()
            else:
                self.camera.release()

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg

    def get_status(self):
        with self.lock:
            age = time.time() - self.last_frame_time if self.last_frame_time else None
            return {
                "backend": self.backend,
                "frames": self.frames_encoded,
                "age": age,
                "error": self.latest_error,
            }

    def _open_camera(self):
        if Picamera2 is not None and not self.args.usb_only:
            try:
                picam2 = Picamera2()
                config = picam2.create_preview_configuration(
                    main={
                        "format": "RGB888",
                        "size": (self.args.width, self.args.height),
                    }
                )
                picam2.configure(config)
                try:
                    picam2.set_controls({"ExposureValue": self.args.exposure_value})
                except Exception:
                    pass
                picam2.start()
                time.sleep(1.0)
                self.backend = "picamera2"
                print("[INFO] Picamera2 camera started")
                return picam2
            except Exception as exc:
                print(f"[WARN] Picamera2 failed: {exc}")
                print("[WARN] Falling back to USB/OpenCV camera")

        cap = cv2.VideoCapture(self.args.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
        cap.set(cv2.CAP_PROP_FPS, self.args.fps)

        if not cap.isOpened():
            print("[ERROR] Camera open failed")
            return None

        self.backend = "usb"
        print("[INFO] USB/OpenCV camera started")
        return cap

    def _capture_loop(self):
        frame_delay = 1.0 / max(1, self.args.fps)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality]

        while self.running.is_set():
            started_at = time.time()
            ok, frame = self._read_frame()

            if not ok:
                with self.lock:
                    self.latest_error = "Failed to read frame"
                time.sleep(0.1)
                continue

            if self.args.overlay:
                self._draw_overlay(frame)

            encoded, buffer = cv2.imencode(".jpg", frame, encode_params)
            if encoded:
                with self.lock:
                    self.latest_jpeg = buffer.tobytes()
                    self.latest_error = ""
                    self.frames_encoded += 1
                    self.last_frame_time = time.time()

            elapsed = time.time() - started_at
            if elapsed < frame_delay:
                time.sleep(frame_delay - elapsed)

    def _read_frame(self):
        if self.backend == "picamera2":
            frame_rgb = self.camera.capture_array()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            return True, self._apply_flip(frame)

        ok, frame = self.camera.read()
        if ok:
            frame = self._apply_flip(frame)
        return ok, frame

    def _apply_flip(self, frame):
        if self.args.flip_horizontal and self.args.flip_vertical:
            return cv2.flip(frame, -1)
        if self.args.flip_horizontal:
            return cv2.flip(frame, 1)
        if self.args.flip_vertical:
            return cv2.flip(frame, 0)
        return frame

    def _draw_overlay(self, frame):
        status = f"{self.backend} {self.args.width}x{self.args.height} @{self.args.fps}fps"
        cv2.putText(
            frame,
            status,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


class CameraHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, worker, verbose=False):
        super().__init__(server_address, handler_class)
        self.worker = worker
        self.verbose = verbose


class CameraRequestHandler(BaseHTTPRequestHandler):
    server_version = "RaspiCameraStream/1.0"

    def do_GET(self):
        path = self.path.split("?", 1)[0]

        if path in ("/", "/index.html"):
            self._send_index()
        elif path == "/stream.mjpg":
            self._send_stream()
        elif path == "/snapshot.jpg":
            self._send_snapshot()
        elif path == "/health":
            self._send_health()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def log_message(self, fmt, *args):
        if self.server.verbose:
            super().log_message(fmt, *args)

    def _send_index(self):
        status = self.server.worker.get_status()
        backend = html.escape(status["backend"])
        body = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Raspberry Pi Camera</title>
  <style>
    html, body {{
      margin: 0;
      min-height: 100%;
      background: #111;
      color: #eee;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    main {{
      width: min(100vw, 960px);
      margin: 0 auto;
      padding: 16px;
      box-sizing: border-box;
    }}
    h1 {{
      margin: 0 0 12px;
      font-size: 20px;
      font-weight: 650;
    }}
    img {{
      display: block;
      width: 100%;
      height: auto;
      background: #000;
      border: 1px solid #333;
    }}
    p {{
      color: #bbb;
      font-size: 14px;
    }}
    a {{
      color: #80c7ff;
    }}
  </style>
</head>
<body>
  <main>
    <h1>Raspberry Pi Camera</h1>
    <img src="/stream.mjpg" alt="Live camera stream">
    <p>Backend: {backend} · <a href="/snapshot.jpg">snapshot</a> · <a href="/health">health</a></p>
  </main>
</body>
</html>
"""
        payload = body.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_snapshot(self):
        jpeg = self._wait_for_jpeg(timeout=3.0)
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No camera frame yet")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_stream(self):
        jpeg = self._wait_for_jpeg(timeout=3.0)
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No camera frame yet")
            return

        self.send_response(HTTPStatus.OK)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                jpeg = self.server.worker.get_latest_jpeg()
                if jpeg is None:
                    time.sleep(0.05)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(1.0 / max(1, self.server.worker.args.fps))
        except (BrokenPipeError, ConnectionResetError):
            return

    def _send_health(self):
        status = self.server.worker.get_status()
        age = "none" if status["age"] is None else f"{status['age']:.3f}"
        payload = (
            f"backend={status['backend']}\n"
            f"frames={status['frames']}\n"
            f"last_frame_age_sec={age}\n"
            f"error={status['error']}\n"
        ).encode("utf-8")

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _wait_for_jpeg(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            jpeg = self.server.worker.get_latest_jpeg()
            if jpeg is not None:
                return jpeg
            time.sleep(0.05)
        return None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream a Raspberry Pi camera to an SSH-connected laptop browser."
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Frame width")
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Frame height")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Target stream FPS")
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=DEFAULT_JPEG_QUALITY,
        choices=range(1, 101),
        metavar="1-100",
        help="JPEG quality",
    )
    parser.add_argument("--camera-index", type=int, default=0, help="USB camera index")
    parser.add_argument(
        "--usb-only",
        action="store_true",
        help="Skip Picamera2 and use cv2.VideoCapture",
    )
    parser.add_argument(
        "--exposure-value",
        type=float,
        default=DEFAULT_EXPOSURE_VALUE,
        help="Picamera2 ExposureValue control",
    )
    parser.add_argument(
        "--no-flip-horizontal",
        dest="flip_horizontal",
        action="store_false",
        help="Disable horizontal flip",
    )
    parser.add_argument(
        "--no-flip-vertical",
        dest="flip_vertical",
        action="store_false",
        help="Disable vertical flip",
    )
    parser.add_argument(
        "--no-overlay",
        dest="overlay",
        action="store_false",
        help="Disable status text drawn on the video",
    )
    parser.add_argument("--verbose", action="store_true", help="Log HTTP requests")
    parser.set_defaults(flip_horizontal=True, flip_vertical=True, overlay=True)
    return parser.parse_args()


def main():
    args = parse_args()
    worker = CameraWorker(args)

    if not worker.start():
        return 1

    server = CameraHTTPServer((args.host, args.port), CameraRequestHandler, worker, args.verbose)

    def stop_server(signum, frame):
        print("\n[INFO] Stopping camera stream")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)

    print(f"[INFO] Streaming on http://{args.host}:{args.port}/")
    print("[INFO] From your laptop, open http://<raspberry-pi-ip>:%d/" % args.port)
    print("[INFO] Or use: ssh -L %d:localhost:%d pi@<raspberry-pi-ip>" % (args.port, args.port))

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        worker.stop()
        print("[INFO] Camera closed")

    return 0


if __name__ == "__main__":
    sys.exit(main())
