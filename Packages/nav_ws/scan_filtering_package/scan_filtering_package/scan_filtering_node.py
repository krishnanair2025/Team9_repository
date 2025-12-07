import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LaserScanThresholdFilter(Node):
    """
    Replaces any LaserScan range value below a threshold
    with NaN so SLAM ignores invalid close readings.
    """

    def __init__(self):
        super().__init__('laser_scan_threshold_filter')

        # Threshold in meters
        self.threshold = 0.2225

        # Subscriber → raw scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher → filtered scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/filtered_scan',
            10
        )

        self.get_logger().info(
            f"Laser scan threshold filter started. Threshold = {self.threshold} m"
        )

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()

        # Copy metadata (angles, increments, timing, etc.)
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        # Filter ranges
        filtered.ranges = [
            (math.nan if r < self.threshold else r)
            for r in msg.ranges
        ]

        # Keep intensities unchanged (optional)
        filtered.intensities = msg.intensities

        self.scan_pub.publish(filtered)



def main(args = None):
    try: 
        rclpy.init(args=args)
        node = LaserScanThresholdFilter()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()
