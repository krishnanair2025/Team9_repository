#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import cv2
import numpy as np
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ultralytics import YOLO

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException


# =========================
# CONFIG
# =========================
MODEL_PATH = "/home/student14/temp_vision_ws/src/cylinder_detector_pkg/models/best.pt"

RGB_TOPIC = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/image'
DEPTH_TOPIC = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/depth_image'
INFO_TOPIC = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/camera_info'

CAMERA_FRAME = "realsense_optical_frame"
MAP_FRAME = "map"

YOLO_CONF = 0.25
YOLO_IOU = 0.50
MIN_Z = 0.10
MAX_Z = 5.0

REQUIRED_STABLE_FRAMES = 5
TEMPORAL_XY_TOLERANCE = 0.05
SEEN_OBJECT_TOLERANCE = 1.0


# =========================
# NODE
# =========================
class YoloGazeboNode(Node):

    def __init__(self):
        super().__init__("yolo_gazebo_node")

        self.bridge = CvBridge()

        self.rgb = None
        self.rgb_stamp = None
        self.depth = None
        self.K = None

        # Internal seen flags
        self.seen_red_cylinder = False
        self.seen_yellow_cylinder = False
        self.seen_green_cylinder = False

        # Seen objects in map frame
        self.seen_objects = []

        # Temporal consistency buffers
        self.reference_point = None
        self.reference_class = None
        self.temporal_buffer = []

        # Subscribers
        self.create_subscription(Image, RGB_TOPIC, self.rgb_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_cb, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.info_cb, 10)

        # Publishers
        self.pub_img = self.create_publisher(Image, "/yolo/debug_image", 10)
        self.pub_point_cam = self.create_publisher(PointStamped, "/cylinder_point_cam", 10)
        self.pub_point_map = self.create_publisher(PointStamped, "/cylinder_point_map", 10)
        self.pub_target = self.create_publisher(String, "/cylinder_target", 10)
        self.pub_detected_flag = self.create_publisher(Bool, "/detected_flag", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLO
        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)

        self.timer = self.create_timer(1 / 30, self.tick)

        self.get_logger().info("Node initialized with map-frame temporal consistency.")

    # =========================
    # Callbacks
    # =========================
    def rgb_cb(self, msg):
        self.rgb_stamp = msg.header.stamp
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)

    def info_cb(self, msg):
        self.K = msg.k

    # =========================
    # TF FIXED
    # =========================
    def tf_transform_latest(self, pt, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pt.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.1)
            )

            return tf2_geometry_msgs.do_transform_point(pt, transform)

        except TransformException as e:
            self.get_logger().warn(f"TF failed: {e}")
            return None

    # =========================
    # TEMPORAL CONSISTENCY LOGIC
    # =========================
    def temporal_consistency_check(self, cls_name, x, y, z):
        if self.reference_point is None or self.reference_class != cls_name:
            self.reference_point = (x, y, z)
            self.reference_class = cls_name
            self.temporal_buffer = [(x, y, z)]
            return False

        rx, ry, _ = self.reference_point

        if (
            abs(x - rx) < TEMPORAL_XY_TOLERANCE and
            abs(y - ry) < TEMPORAL_XY_TOLERANCE
        ):
            self.temporal_buffer.append((x, y, z))
        else:
            self.reference_point = (x, y, z)
            self.reference_class = cls_name
            self.temporal_buffer = [(x, y, z)]
            return False

        return len(self.temporal_buffer) >= REQUIRED_STABLE_FRAMES

    def is_already_seen(self, x, y, z):
        return any(
            abs(x - sx) < SEEN_OBJECT_TOLERANCE and
            abs(y - sy) < SEEN_OBJECT_TOLERANCE
            for sx, sy, _ in self.seen_objects
        )

    def reset_temporal_buffer(self):
        self.reference_point = None
        self.reference_class = None
        self.temporal_buffer = []

    def log_seen_objects(self):
        self.get_logger().info("Updated seen objects list:")
        for i, (x, y, z) in enumerate(self.seen_objects, 1):
            self.get_logger().info(
                f"  {i}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}"
            )

    # =========================
    # DETECTION FLAG LOGIC
    # =========================
    def publish_detected_flag_if_new(self, cls_name):
        is_new = False

        if cls_name == "red_cylinder" and not self.seen_red_cylinder:
            self.seen_red_cylinder = True
            is_new = True

        elif cls_name == "yellow_cylinder" and not self.seen_yellow_cylinder:
            self.seen_yellow_cylinder = True
            is_new = True

        elif cls_name == "green_cylinder" and not self.seen_green_cylinder:
            self.seen_green_cylinder = True
            is_new = True

        if is_new:
            msg = Bool()
            msg.data = True
            self.pub_detected_flag.publish(msg)
            self.get_logger().info(
                f"New stable cylinder detected: {cls_name}. Published /detected_flag=True"
            )

    # =========================
    # DEBUG IMAGE TEXT
    # =========================
    def draw_bottom_right_text(self, image, lines):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 2
        line_gap = 8
        margin = 12

        line_sizes = [
            cv2.getTextSize(line, font, font_scale, thickness)[0]
            for line in lines
        ]

        box_width = max(size[0] for size in line_sizes) + 2 * margin
        box_height = (
            sum(size[1] for size in line_sizes)
            + line_gap * (len(lines) - 1)
            + 2 * margin
        )

        h, w = image.shape[:2]
        x0 = max(w - box_width - margin, 0)
        y0 = max(h - box_height - margin, 0)
        x1 = w - margin
        y1 = h - margin

        overlay = image.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, image, 0.45, 0, image)

        y = y0 + margin
        for i, line in enumerate(lines):
            text_h = line_sizes[i][1]
            y += text_h
            cv2.putText(
                image,
                line,
                (x0 + margin, y),
                font,
                font_scale,
                (0, 255, 255),
                thickness
            )
            y += line_gap

    def draw_bottom_left_text(self, image, lines):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 2
        line_gap = 8
        margin = 12

        line_sizes = [
            cv2.getTextSize(line, font, font_scale, thickness)[0]
            for line in lines
        ]

        box_width = max(size[0] for size in line_sizes) + 2 * margin
        box_height = (
            sum(size[1] for size in line_sizes)
            + line_gap * (len(lines) - 1)
            + 2 * margin
        )

        h, w = image.shape[:2]
        x0 = margin
        y0 = max(h - box_height - margin, 0)
        x1 = min(x0 + box_width, w)
        y1 = h - margin

        overlay = image.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, image, 0.45, 0, image)

        y = y0 + margin
        for i, line in enumerate(lines):
            text_h = line_sizes[i][1]
            y += text_h
            cv2.putText(
                image,
                line,
                (x0 + margin, y),
                font,
                font_scale,
                (0, 255, 255),
                thickness
            )
            y += line_gap

    # =========================
    # MAIN LOOP
    # =========================
    def tick(self):
        if self.rgb is None or self.depth is None or self.K is None:
            return

        color = self.rgb.copy()
        depth = self.depth.copy()

        depth[(depth < MIN_Z) | (depth > MAX_Z)] = np.nan

        res = self.model.predict(color, conf=YOLO_CONF, iou=YOLO_IOU, verbose=False)[0]

        fx, fy = float(self.K[0]), float(self.K[4])
        cx_i, cy_i = float(self.K[2]), float(self.K[5])

        best = None

        for box in res.boxes:
            cls_name = res.names[int(box.cls[0])]
            conf = float(box.conf[0])

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            # Centroid of the YOLO bounding box in image pixel coordinates
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            r = 5
            patch = depth[
                max(0, cy - r):cy + r + 1,
                max(0, cx - r):cx + r + 1
            ]
            valid = patch[np.isfinite(patch)]

            if valid.size > 10:
                z = float(np.median(valid))
                X = float((cx - cx_i) * z / fx)
                Y = float((cy - cy_i) * z / fy)
                xyz = (X, Y, z)
            else:
                xyz = (0.0, 0.0, 0.0)

            # Draw bounding box
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 255), 2)

            label = f"{cls_name} {xyz[2]:.2f}m" if xyz[2] > 0 else cls_name
            cv2.putText(
                color,
                label,
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

            # Mark centroid only
            cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)
            cv2.drawMarker(
                color,
                (cx, cy),
                (255, 0, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2
            )

            if "cylinder" in cls_name and xyz[2] > 0:
                if best is None or conf > best["conf"]:
                    best = {
                        "cls": cls_name,
                        "xyz": xyz,
                        "conf": conf,
                        "centroid_px": (cx, cy)
                    }

        # publish points
        if best is not None:
            x, y, z = best["xyz"]

            self.draw_bottom_left_text(
                color,
                [
                    f"x = {x:.3f}",
                    f"y = {y:.3f}",
                    f"z = {z:.3f}"
                ]
            )

            msg_cam = PointStamped()
            msg_cam.header.stamp = self.rgb_stamp
            msg_cam.header.frame_id = CAMERA_FRAME
            msg_cam.point.x = float(x)
            msg_cam.point.y = float(y)
            msg_cam.point.z = float(z)

            # Camera-frame point still publishes continuously
            self.pub_point_cam.publish(msg_cam)

            # Transform to map
            msg_map = self.tf_transform_latest(msg_cam, MAP_FRAME)

            if msg_map is not None:
                raw_map_x = float(msg_map.point.x)
                raw_map_y = float(msg_map.point.y)
                raw_map_z = float(msg_map.point.z)

                confirmed = self.temporal_consistency_check(
                    best["cls"],
                    raw_map_x,
                    raw_map_y,
                    raw_map_z
                )

                self.draw_bottom_right_text(
                    color,
                    [
                        f"x = {raw_map_x:.3f}",
                        f"y = {raw_map_y:.3f}",
                        f"z = {raw_map_z:.3f}"
                    ]
                )

                if confirmed:
                    mean_x = float(np.mean([p[0] for p in self.temporal_buffer]))
                    mean_y = float(np.mean([p[1] for p in self.temporal_buffer]))
                    mean_z = float(np.mean([p[2] for p in self.temporal_buffer]))

                    if not self.is_already_seen(mean_x, mean_y, mean_z):
                        self.seen_objects.append((mean_x, mean_y, mean_z))
                        self.log_seen_objects()

                        stable_map = PointStamped()
                        stable_map.header.stamp = self.get_clock().now().to_msg()
                        stable_map.header.frame_id = MAP_FRAME
                        stable_map.point.x = mean_x
                        stable_map.point.y = mean_y
                        stable_map.point.z = mean_z

                        self.pub_point_map.publish(stable_map)

                        # Detection flag only publishes after stable map-frame confirmation
                        self.publish_detected_flag_if_new(best["cls"])

                        target = {
                            "class": best["cls"],
                            "x": mean_x,
                            "y": mean_y,
                            "z": mean_z,
                            "frame": MAP_FRAME
                        }

                        msg_str = String()
                        msg_str.data = json.dumps(target)
                        self.pub_target.publish(msg_str)

                        self.get_logger().info(
                            f"Published stable {best['cls']} map point: "
                            f"x={mean_x:.3f}, y={mean_y:.3f}, z={mean_z:.3f}"
                        )

                    self.reset_temporal_buffer()

            else:
                self.draw_bottom_right_text(
                    color,
                    [
                        "x = TF failed",
                        "y = TF failed",
                        "z = TF failed"
                    ]
                )
                self.reset_temporal_buffer()

        else:
            self.reset_temporal_buffer()

        # publish image after map coordinates have been drawn
        msg_img = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        msg_img.header.stamp = self.rgb_stamp
        msg_img.header.frame_id = CAMERA_FRAME
        self.pub_img.publish(msg_img)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloGazeboNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
