import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from cv_bridge import CvBridge


class DepthToPointCloud(Node):

    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.bridge = CvBridge()

        # Gazebo topics
        self.depth_topic = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/depth_image'
        self.camera_info_topic = '/world/empty/model/leo_rover/link/base_footprint/sensor/realsense_rgbd/camera_info'

        self.depth_image = None
        self.camera_info = None

        # Throttle control (10 Hz)
        self.last_time = self.get_clock().now()

        # Subscribers (sensor QoS)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, qos_profile_sensor_data)

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/camera/points', 10)

        self.get_logger().info("Fast Depth → PointCloud node started with height filtering.")

    def depth_callback(self, msg):
        now = self.get_clock().now()

        # Limit processing to ~10 Hz
        if (now - self.last_time).nanoseconds < 1e8:
            return

        self.last_time = now
        self.depth_image = msg
        self.try_generate_cloud()

    def info_callback(self, msg):
        self.camera_info = msg

    def try_generate_cloud(self):
        if self.depth_image is None or self.camera_info is None:
            return

        # Convert depth image
        if self.depth_image.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='32FC1')
        elif self.depth_image.encoding == '16UC1':
            depth = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='16UC1')
            depth = depth.astype(np.float32) / 1000.0
        else:
            self.get_logger().warn(f"Unsupported encoding: {self.depth_image.encoding}")
            return

        height, width = depth.shape

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        max_range = 10.0  # max depth in meters

        # === VECTORIZED POINT CLOUD GENERATION ===
        u = np.arange(width)
        v = np.arange(height)
        uu, vv = np.meshgrid(u, v)
        z = depth

        # Mask valid points
        mask = np.isfinite(z) & (z > 0.0) & (z < max_range)
        z = z[mask]
        uu = uu[mask]
        vv = vv[mask]

        # Compute XYZ coordinates
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        points = np.stack((x, y, z), axis=-1)

        # === HEIGHT FILTERING: 0–10 cm below camera ===
        min_height = 0.0
        max_height = 0.10
        height_mask = (points[:, 1] >= min_height) & (points[:, 1] <= max_height)
        points = points[height_mask]

        # Prepare header and publish
        header = Header()
        header.stamp = self.depth_image.header.stamp
        #header.frame_id = self.camera_info.header.frame_id
        header.frame_id = "realsense_optical_frame"

        pc_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.pc_pub.publish(pc_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DepthToPointCloud()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()