#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import json
import cv2
import numpy as np
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from ultralytics import YOLO

# ===== TF2（用于实时坐标变换）=====
import tf2_ros
from tf2_ros import TransformException

try:
    import tf2_geometry_msgs  # noqa: F401
except Exception:
    tf2_geometry_msgs = None

# ===== RealSense =====
try:
    import pyrealsense2 as rs
except Exception as e:
    raise RuntimeError(
        "pyrealsense2 not found. For Intel RealSense D435i you MUST use RealSense SDK.\n"
        "Install: pip install pyrealsense2\n"
        f"Original import error: {e}"
    )

# =========================
# 1) 配置
# =========================
MODEL_PATH = "/home/student07/Camera/best.pt"

ALLOWED_CLASSES = {
    "red_cylinder", "yellow_cylinder", "green_cylinder",
    "red_bin", "yellow_bin", "green_bin",
}

YOLO_CONF = 0.25
YOLO_IOU = 0.50
MAX_DET = 20

COLOR_W, COLOR_H, FPS = 640, 480, 30
DEPTH_W, DEPTH_H = 640, 480

MIN_Z = 0.10
MAX_Z = 5.00

DEPTH_SAMPLE_HALF = 6
MIN_VALID_SAMPLES = 30

EMA_ALPHA = 0.65
TRACK_MAX_AGE = 0.4
IOU_MATCH_TH = 0.25

CAMERA_FRAME = "camera_frame"
ARM_BASE_FRAME = "arm_base_link"
MAP_FRAME = "map"

TF_TIMEOUT_SEC = 0.05

# =========================
# 2) 工具函数
# =========================
def bbox_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0, inter_x2 - inter_x1)
    ih = max(0, inter_y2 - inter_y1)
    inter = iw * ih
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = area_a + area_b - inter + 1e-6
    return inter / union

def robust_depth_in_bbox(depth_m, x1, y1, x2, y2):
    h, w = depth_m.shape[:2]
    x1 = int(max(0, min(w - 1, x1)))
    x2 = int(max(0, min(w - 1, x2)))
    y1 = int(max(0, min(h - 1, y1)))
    y2 = int(max(0, min(h - 1, y2)))

    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    r = DEPTH_SAMPLE_HALF

    sx1 = max(0, cx - r)
    sx2 = min(w, cx + r + 1)
    sy1 = max(0, cy - r)
    sy2 = min(h, cy + r + 1)

    patch = depth_m[sy1:sy2, sx1:sx2]
    vals = patch[np.isfinite(patch)]

    if vals.size < MIN_VALID_SAMPLES:
        patch2 = depth_m[y1:y2, x1:x2]
        vals2 = patch2[np.isfinite(patch2)]
        if vals2.size < MIN_VALID_SAMPLES:
            return np.nan
        z = float(np.median(vals2))
    else:
        z = float(np.median(vals))

    if not (MIN_Z <= z <= MAX_Z):
        return np.nan
    return z

def ema_xyz(prev_xyz, new_xyz):
    if not np.isfinite(prev_xyz[2]):
        return new_xyz
    if not np.isfinite(new_xyz[2]):
        return prev_xyz
    x = EMA_ALPHA * prev_xyz[0] + (1 - EMA_ALPHA) * new_xyz[0]
    y = EMA_ALPHA * prev_xyz[1] + (1 - EMA_ALPHA) * new_xyz[1]
    z = EMA_ALPHA * prev_xyz[2] + (1 - EMA_ALPHA) * new_xyz[2]
    return (x, y, z)

@dataclass
class Track:
    cls_name: str
    bbox: tuple
    xyz: tuple
    last_t: float
    conf: float

class SimpleTracker:
    def __init__(self):
        self.tracks = []

    def update(self, detections, now_t):
        updated = []
        used_det = set()

        for tr in self.tracks:
            best_j = -1
            best_iou = 0.0
            for j, det in enumerate(detections):
                if j in used_det:
                    continue
                if det["cls_name"] != tr.cls_name:
                    continue
                iou = bbox_iou(tr.bbox, det["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_j = j

            if best_j >= 0 and best_iou >= IOU_MATCH_TH:
                det = detections[best_j]
                used_det.add(best_j)
                xyz = ema_xyz(tr.xyz, det["xyz"])
                updated.append(Track(det["cls_name"], det["bbox"], xyz, now_t, det["conf"]))
            else:
                if (now_t - tr.last_t) <= TRACK_MAX_AGE:
                    updated.append(tr)

        for j, det in enumerate(detections):
            if j in used_det:
                continue
            updated.append(Track(det["cls_name"], det["bbox"], det["xyz"], now_t, det["conf"]))

        self.tracks = updated
        return self.tracks

# =========================
# 3) RealSense 初始化
# =========================
def start_realsense():
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)

    profile = pipeline.start(config)

    align = rs.align(rs.stream.color)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()

    return pipeline, align, depth_scale, intr

# =========================
# 4) ROS2 节点
# =========================
class YoloRealsenseNode(Node):
    def __init__(self):
        super().__init__("yolo_realsense_node")

        self.pub_point = self.create_publisher(PointStamped, "/cylinder_point_cam", 10)
        self.pub_point_arm = self.create_publisher(PointStamped, "/cylinder_point_arm", 10)
        self.pub_point_map = self.create_publisher(PointStamped, "/cylinder_point_map", 10)

        # 合成后的单条消息
        self.pub_target = self.create_publisher(String, "/cylinder_target", 10)

        self.pub_debug = self.create_publisher(String, "/cylinder_debug", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._last_tf_warn_t = 0.0

        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)

        self.pipeline, self.align, self.depth_scale, self.intr = start_realsense()
        self.tracker = SimpleTracker()

        self.get_logger().info("RealSense started.")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /cylinder_point_cam (PointStamped)")
        self.get_logger().info("  /cylinder_point_arm (PointStamped, if TF available)")
        self.get_logger().info("  /cylinder_point_map (PointStamped, if TF available)")
        self.get_logger().info("  /cylinder_target (String JSON: class + arm point)")
        self.get_logger().info(f"Frames: camera={CAMERA_FRAME}, arm_base={ARM_BASE_FRAME}, map={MAP_FRAME}")

        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def _throttle_tf_warn(self, text: str, period_sec: float = 1.0):
        now = time.time()
        if now - self._last_tf_warn_t > period_sec:
            self._last_tf_warn_t = now
            self.get_logger().warn(text)

    def _tf_to(self, msg_in: PointStamped, target_frame: str):
        timeout = Duration(seconds=float(TF_TIMEOUT_SEC))
        try:
            return self.tf_buffer.transform(msg_in, target_frame, timeout=timeout)
        except TransformException as e:
            self._throttle_tf_warn(f"TF {msg_in.header.frame_id}->{target_frame} failed: {e}")
            return None

    def tick(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return

        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_raw * float(self.depth_scale)
        depth_m[(depth_m < MIN_Z) | (depth_m > MAX_Z)] = np.nan

        res = self.model.predict(
            source=color,
            imgsz=640,
            conf=YOLO_CONF,
            iou=YOLO_IOU,
            max_det=MAX_DET,
            verbose=False
        )[0]

        h, w = color.shape[:2]
        detections = []

        for box in res.boxes:
            cls_id = int(box.cls[0])
            cls_name = res.names[cls_id]
            if cls_name not in ALLOWED_CLASSES:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1 = int(max(0, min(w - 1, x1)))
            x2 = int(max(0, min(w - 1, x2)))
            y1 = int(max(0, min(h - 1, y1)))
            y2 = int(max(0, min(h - 1, y2)))

            z = robust_depth_in_bbox(depth_m, x1, y1, x2, y2)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            if np.isfinite(z):
                X, Y, Z = rs.rs2_deproject_pixel_to_point(
                    self.intr, [float(cx), float(cy)], float(z)
                )
                xyz = (float(X), float(Y), float(Z))
            else:
                xyz = (np.nan, np.nan, np.nan)

            detections.append({
                "cls_name": cls_name,
                "bbox": (x1, y1, x2, y2),
                "xyz": xyz,
                "conf": float(box.conf[0])
            })

        now_t = time.time()
        tracks = self.tracker.update(detections, now_t)

        best = None
        for tr in tracks:
            if "cylinder" not in tr.cls_name:
                continue
            if not np.isfinite(tr.xyz[2]):
                continue
            if best is None or tr.conf > best.conf:
                best = tr

        if best is not None:
            # 相机系点
            msg_cam = PointStamped()
            msg_cam.header.stamp = self.get_clock().now().to_msg()
            msg_cam.header.frame_id = CAMERA_FRAME
            msg_cam.point.x = float(best.xyz[0])
            msg_cam.point.y = float(best.xyz[1])
            msg_cam.point.z = float(best.xyz[2])
            self.pub_point.publish(msg_cam)

            # arm 系点
            msg_arm = self._tf_to(msg_cam, ARM_BASE_FRAME)
            if msg_arm is not None:
                self.pub_point_arm.publish(msg_arm)

                # 合成一条 String 消息（JSON）
                target = {
                    "class": best.cls_name,
                    "x": float(msg_arm.point.x),
                    "y": float(msg_arm.point.y),
                    "z": float(msg_arm.point.z),
                    "frame": msg_arm.header.frame_id
                }
                target_msg = String()
                target_msg.data = json.dumps(target)
                self.pub_target.publish(target_msg)

            # map 系点
            msg_map = self._tf_to(msg_cam, MAP_FRAME)
            if msg_map is not None:
                self.pub_point_map.publish(msg_map)

            # debug
            dbg = String()
            dbg.data = (
                f"{best.cls_name} "
                f"x={best.xyz[0]:.3f} y={best.xyz[1]:.3f} z={best.xyz[2]:.3f} "
                f"frame={CAMERA_FRAME}"
            )
            self.pub_debug.publish(dbg)

        vis = color.copy()
        for tr in tracks:
            x1, y1, x2, y2 = tr.bbox
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            label = tr.cls_name
            if np.isfinite(tr.xyz[2]):
                label += f" Z={tr.xyz[2]:.2f}m"
            cv2.putText(
                vis,
                label,
                (x1, max(15, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        cv2.imshow("YOLO + 3D (camera_frame)", vis)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = YoloRealsenseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
