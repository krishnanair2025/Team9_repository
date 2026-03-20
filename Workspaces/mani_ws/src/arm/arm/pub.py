import rclpy
from rclpy.node import Node
from rover_interface.msg import ColorPoint
print("done")

class TargetPublisher(Node):

    def __init__(self):
        super().__init__('target_publisher')

        self.publisher_ = self.create_publisher(ColorPoint, '/leorover', 10)

        # Send once after small delay to allow discovery
        self.timer = self.create_timer(1.0, self.publish_target)
        # self.sent = False

    def publish_target(self):
        # if self.sent:
        #    return

        msg = ColorPoint()
        msg.header.frame_id = "map"
        
        msg.color = "red"

        # 👇 Set your target coordinate here (mm for MyCobot)
        msg.position.x =  200.0
        msg.position.y = 0.0
        msg.position.z = 156.0

        self.publisher_.publish(msg)
        self.get_logger().info("Target pose published!")

        # self.sent = True
        # self.timer.cancel()  # stop after sending once


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
