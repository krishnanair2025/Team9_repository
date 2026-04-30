#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs

import cv2
import numpy as np
import math


class GreenObjectDetector(Node):

    REQUIRED_STABLE_FRAMES = 5 #10
    TEMPORAL_XY_TOLERANCE = 0.35 #0.2    # meters (tight consistency check)
    SEEN_OBJECT_TOLERANCE = 1.0      # meters (1x1m grid)

    def __init__(self):
        super().__init__('green_object_detector')

        self.bridge = CvBridge()
        self.latest_depth = None

        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_received = False

        self.camera_frame = "realsense_optical_frame"

        # Topics
        self.rgb_topic = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/image'
        self.depth_topic = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/depth_image'
        self.camera_info_topic = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/camera_info'

        # Subscribers
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/green_objects/annotated_image', 10)
        self.detected_flag_pub = self.create_publisher(Bool, 'detected_flag', 10)
        self.object_location_pub = self.create_publisher(PointStamped, '/cylinder_point_map', 10)

        # >>> ADDED publisher (camera-frame, continuous)
        self.cam_object_pub = self.create_publisher(
            PointStamped, '/cylinder_point_cam', 10
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Seen objects (map frame)
        self.seen_objects = []

        # Temporal consistency buffers (single object assumption)
        self.reference_point = None
        self.temporal_buffer = []

        self.get_logger().info("Green Object Detector started (temporal consistency enabled)")

    # ------------------- Callbacks -------------------

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_received:
            return

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True

        self.get_logger().info(
            f"Camera intrinsics received | fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def rgb_callback(self, msg: Image):
        if self.latest_depth is None or not self.camera_info_received:
            return

        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return

        if self.latest_depth.shape != rgb_img.shape[:2]:
            self.latest_depth = cv2.resize(
                self.latest_depth,
                (rgb_img.shape[1], rgb_img.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )

        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue

            #cx = int(M['m10'] / M['m00'])
            #cy = int(M['m01'] / M['m00'])

            cx = int(np.mean(cnt[:,0,0]))
            cy = int(np.mean(cnt[:,0,1]))

            window = 4
            x0, x1 = max(cx - window, 0), min(cx + window, self.latest_depth.shape[1] - 1)
            y0, y1 = max(cy - window, 0), min(cy + window, self.latest_depth.shape[0] - 1)

            depth_window = self.latest_depth[y0:y1 + 1, x0:x1 + 1]
            valid_depths = depth_window[np.isfinite(depth_window) & (depth_window > 0)]

            if len(valid_depths) == 0:
                continue

            Z = float(np.mean(valid_depths))
            X = (cx - self.cx) * Z / self.fx
            Y = (cy - self.cy) * Z / self.fy

            # >>> PREPARE + CONTINUOUSLY PUBLISH CAMERA-FRAME POSE
            cam_pose = PointStamped()
            cam_pose.header.frame_id = self.camera_frame
            cam_pose.header.stamp = self.get_clock().now().to_msg()
            cam_pose.point.x = X
            cam_pose.point.y = Y
            cam_pose.point.z = Z

            self.cam_object_pub.publish(cam_pose)

            # Transform to map frame
            point_cam = PointStamped()
            point_cam.header.frame_id = self.camera_frame
            point_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()
            point_cam.point.x = X
            point_cam.point.y = Y
            point_cam.point.z = Z

            try:
                point_map = self.tf_buffer.transform(
                    point_cam, 'map', timeout=Duration(seconds=0.5)
                )
            except Exception:
                self.annotate(rgb_img, cnt, cx, cy, None)
                continue

            X_map = point_map.point.x
            Y_map = point_map.point.y
            Z_map = point_map.point.z

            confirmed = self.temporal_consistency_check(X_map, Y_map, Z_map)

            if confirmed:
                mean_x = np.mean([p[0] for p in self.temporal_buffer])
                mean_y = np.mean([p[1] for p in self.temporal_buffer])
                mean_z = np.mean([p[2] for p in self.temporal_buffer])

                if not self.is_already_seen(mean_x, mean_y, mean_z):
                    self.seen_objects.append((mean_x, mean_y, mean_z))

                    self.log_seen_objects()

                    self.detected_flag_pub.publish(Bool(data=True))

                    pose = PointStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.point.x = mean_x
                    pose.point.y = mean_y
                    pose.point.z = mean_z
                    self.object_location_pub.publish(pose)

                self.temporal_buffer.clear()
                self.reference_point = None

            self.annotate(rgb_img, cnt, cx, cy, (X_map, Y_map, Z_map))

        self.annotated_pub.publish(
            self.bridge.cv2_to_imgmsg(rgb_img, encoding='bgr8')
        )

    # ------------------- Helpers -------------------

    def temporal_consistency_check(self, x, y, z):
        if self.reference_point is None:
            self.reference_point = (x, y, z)
            self.temporal_buffer = [(x, y, z)]
            return False

        rx, ry, _ = self.reference_point

        if abs(x - rx) < self.TEMPORAL_XY_TOLERANCE and abs(y - ry) < self.TEMPORAL_XY_TOLERANCE:
            self.temporal_buffer.append((x, y, z))
        else:
            self.reference_point = (x, y, z)
            self.temporal_buffer = [(x, y, z)]
            return False

        return len(self.temporal_buffer) >= self.REQUIRED_STABLE_FRAMES

    def is_already_seen(self, x, y, z):
        return any(
            abs(x - sx) < self.SEEN_OBJECT_TOLERANCE and
            abs(y - sy) < self.SEEN_OBJECT_TOLERANCE
            for sx, sy, _ in self.seen_objects
        )

    def log_seen_objects(self):
        self.get_logger().info("Updated seen objects list:")
        for i, (x, y, z) in enumerate(self.seen_objects, 1):
            self.get_logger().info(f"  {i}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    def annotate(self, img, cnt, cx, cy, coords):
        cv2.drawContours(img, [cnt], -1, (0, 0, 255), 2)
        cv2.circle(img, (cx, cy), 4, (255, 0, 0), -1)

        if coords is not None:
            text = f"X:{coords[0]:.2f} Y:{coords[1]:.2f}"
        else:
            text = "TF FAIL"

        cv2.putText(
            img, text, (cx + 5, cy - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )


def main(args = None):
    try: 
        rclpy.init(args=args)
        node = GreenObjectDetector()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()
