#!/usr/bin/env python3
"""Live preview for one or two Intel RealSense color streams.

Controls:
    q / Esc  Quit
    s        Save a side-by-side snapshot
"""

from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


DEFAULT_WRIST_SERIAL = "238222076529"
DEFAULT_GLOBAL_SERIAL = "142122071524"


class RealSenseColorStream:
    def __init__(self, name: str, serial: str, width: int, height: int, fps: int):
        self.name = name
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.frame_count = 0
        self.fps_window_start = time.perf_counter()
        self.measured_fps = 0.0

    def start(self) -> None:
        self.pipeline.start(self.config)

    def stop(self) -> None:
        self.pipeline.stop()

    def read(self, timeout_ms: int) -> np.ndarray | None:
        ok, frames = self.pipeline.try_wait_for_frames(timeout_ms=timeout_ms)
        if not ok:
            return None
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None

        rgb = np.asanyarray(color_frame.get_data())
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self.frame_count += 1
        now = time.perf_counter()
        elapsed = now - self.fps_window_start
        if elapsed >= 1.0:
            self.measured_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_window_start = now
        return bgr


def label_frame(frame: np.ndarray, label: str, measured_fps: float) -> np.ndarray:
    out = frame.copy()
    text = f"{label}  {measured_fps:.1f} fps"
    cv2.rectangle(out, (0, 0), (360, 34), (0, 0, 0), thickness=-1)
    cv2.putText(out, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return out


def stack_frames(frames: list[np.ndarray], layout: str) -> np.ndarray:
    if len(frames) == 1:
        return frames[0]
    if layout == "vertical":
        return np.vstack(frames)
    return np.hstack(frames)


class SharedPreview:
    def __init__(self, snapshot_dir: Path):
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.snapshot_dir = snapshot_dir

    def update(self, frame: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return
        with self.lock:
            self.jpeg = encoded.tobytes()

    def get(self) -> bytes | None:
        with self.lock:
            return self.jpeg

    def save_snapshot(self) -> Path | None:
        jpeg = self.get()
        if jpeg is None:
            return None
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = self.snapshot_dir / f"realsense_live_{ts}.jpg"
        path.write_bytes(jpeg)
        return path


def make_handler(shared: SharedPreview):
    class LiveViewHandler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args) -> None:
            return

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                body = b"""<!doctype html>
<html>
<head><title>PiPER RealSense Live View</title></head>
<body style="margin:0;background:#111;color:#eee;font-family:sans-serif">
  <div style="padding:8px 12px">PiPER RealSense Live View |
    <a href="/snapshot" style="color:#8cf">save snapshot</a>
  </div>
  <img src="/stream.mjpg" style="width:100%;height:auto;display:block" />
</body>
</html>
"""
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/snapshot":
                path = shared.save_snapshot()
                body = f"saved: {path}\n".encode() if path else b"no frame available\n"
                self.send_response(200 if path else 503)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path != "/stream.mjpg":
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                jpeg = shared.get()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    break

    return LiveViewHandler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wrist-serial", default=DEFAULT_WRIST_SERIAL)
    parser.add_argument("--global-serial", default=DEFAULT_GLOBAL_SERIAL)
    parser.add_argument("--only", choices=["both", "wrist", "global"], default="both")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--layout", choices=["horizontal", "vertical"], default="horizontal")
    parser.add_argument("--viewer", choices=["web", "opencv"], default="web")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--snapshot-dir", default=str(Path(__file__).resolve().parent.parent / "camera_check" / "live_view"))
    args = parser.parse_args()

    streams: list[RealSenseColorStream] = []
    if args.only in ("both", "wrist"):
        streams.append(RealSenseColorStream("wrist", args.wrist_serial, args.width, args.height, args.fps))
    if args.only in ("both", "global"):
        streams.append(RealSenseColorStream("global", args.global_serial, args.width, args.height, args.fps))

    snapshot_dir = Path(args.snapshot_dir)
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    print("Starting RealSense live view:")
    for stream in streams:
        print(f"  {stream.name}: serial={stream.serial}, {args.width}x{args.height}@{args.fps}")
        stream.start()

    shared = SharedPreview(snapshot_dir)
    server: ThreadingHTTPServer | None = None
    if args.viewer == "web":
        server = ThreadingHTTPServer((args.host, args.port), make_handler(shared))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"Open this URL in a browser: http://{args.host}:{args.port}")
        print("Use Ctrl-C in this terminal to quit. Browser snapshot: /snapshot")
    else:
        print("Controls: q/Esc=quit, s=save snapshot")
        window_name = "PiPER RealSense Live View"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    latest: dict[str, np.ndarray] = {}
    try:
        while True:
            labeled_frames: list[np.ndarray] = []
            for stream in streams:
                frame = stream.read(args.timeout_ms)
                if frame is not None:
                    latest[stream.name] = frame
                elif stream.name in latest:
                    frame = latest[stream.name]
                else:
                    frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        f"waiting for {stream.name}",
                        (30, args.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                labeled_frames.append(label_frame(frame, stream.name, stream.measured_fps))

            canvas = stack_frames(labeled_frames, args.layout)
            if args.viewer == "web":
                shared.update(canvas)
            else:
                cv2.imshow(window_name, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    path = snapshot_dir / f"realsense_live_{ts}.jpg"
                    cv2.imwrite(str(path), canvas)
                    print(f"Saved snapshot: {path}")
    finally:
        if server is not None:
            server.shutdown()
        for stream in streams:
            stream.stop()
        if args.viewer == "opencv":
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
