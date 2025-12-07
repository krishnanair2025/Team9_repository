import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import random
import time

class DummyObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('dummy_objection_detection_node')

        self.detection_publisher = self.create_publisher(
            Bool, 
            'detected_flag', 
            10
        )

        self.coords_publisher = self.create_publisher(
            PoseStamped,
            'object_location',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.active = True
        self.coord = [2, -0.0]

    def timer_callback(self):
        if self.active == False:
            return
        
        num = random.randint(0,30)
        self.get_logger().info(f"Generated number: {num}")

        if num == 13:
            msg = Bool()
            msg.data = True
            self.detection_publisher.publish(msg)
            self.get_logger().info("Object Detected")

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(self.coord[0])
            pose.pose.position.y = float(self.coord[1])
            pose.pose.orientation.w = 1.0
            self.coords_publisher.publish(pose)

            self.active = False
            self.create_timer(10.0,self.resume_rng)

    def resume_rng(self):
        self.get_logger().info("Resuming object detection")
        self.active = True

def main(args = None):
    try: 
        rclpy.init(args=args)
        node = DummyObjectDetectionNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()
