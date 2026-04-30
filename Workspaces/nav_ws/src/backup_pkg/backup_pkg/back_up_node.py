#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool

class BackupNode(Node):
    """ ROS2 Node that moves rover back for a fixed duration """

    def __init__(self):
        super().__init__('back_up_node')

        # Service to trigger backup
        self.approach_obj_server = self.create_service(
            srv_type=Trigger,
            srv_name="/backup",
            callback=self.backup_server_callback
        )

        # Publisher for backup success
        self.success_pub = self.create_publisher(
            Bool,
            '/backup_success',
            10
        )

        # Publisher for command velocity
        self.velocity_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.active = False
        self.timer = None
        self.backup_duration = 15.0   # seconds
        self.elapsed_time = 0.0

    def backup_server_callback(self, request, response):
        if self.active:
            response.success = False
            response.message = "Backup already in progress"
            return response

        self.get_logger().info("Backup requested")
        self.active = True
        self.elapsed_time = 0.0
        self.timer = self.create_timer(0.05, self.reverse)  # 20 Hz

        response.success = True
        response.message = "Backing up rover"
        return response

    def reverse(self):
        if not self.active:
            return

        twist = Twist()
        twist.linear.x = -0.1  # reverse speed
        self.velocity_pub.publish(twist)

        self.elapsed_time += 0.05
        if self.elapsed_time >= self.backup_duration:
            # Stop rover
            twist.linear.x = 0.0
            self.velocity_pub.publish(twist)

            # Publish success
            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)

            self.active = False
            if self.timer:
                self.timer.cancel()
                self.timer = None


def main(args=None):
    try:
        rclpy.init(args=args)
        node = BackupNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
