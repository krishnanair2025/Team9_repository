import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math


class LidarStaticTFPublisher(Node):

    def __init__(self):
        super().__init__('lidar_static_tf_publisher')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Create all transforms
        t0 = self.create_basefootprint_to_base()
        t1 = self.create_base_to_lidar()
        t2 = self.create_lidar_to_laser()

        # Send static transforms
        self.tf_broadcaster.sendTransform([t0, t1, t2])

    def create_basefootprint_to_base(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'

        # From the relationship shown: Position = 0, 0, 0.19783
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.19783

        # Orientation = 0, 0, 0, 1
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        return t

    def create_base_to_lidar(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rplidar_link'

        # From URDF: xyz="0.084806 0 0.0645"
        t.transform.translation.x = 0.084806
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0645

        # No rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        return t

    def create_lidar_to_laser(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'rplidar_link'
        t.child_frame_id = 'laser_frame'

        # From URDF: xyz="0 0 0"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # From URDF: rpy="0 0 -pi/2"
        yaw = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)

        return t


def main(args=None):
    try:
        rclpy.init(args=args)
        node = LidarStaticTFPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
