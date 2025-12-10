# checking git update

import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import random
import time

class DummyObjectDetectionNode(Node):

    """ ROS2 Node that simulates the camera detecting a coloured object using a random number generator.
    An object detection trigger is published whenever the random number generator rolls 13"""

    def __init__(self):
        super().__init__('dummy_objection_detection_node')

        # Publisher for the object detection flag
        self.detection_publisher = self.create_publisher(
            Bool, 
            'detected_flag', 
            10
        )

        # Publisher for the object location coordinates
        self.coords_publisher = self.create_publisher(
            PoseStamped,
            'object_location',
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        # Internal variables
        self.active = True 
        self.coord = [2, -0.0]

    def timer_callback(self):

        if self.active == False:
            return
        
        # Randomly generating a number between 0 and 30.
        num = random.randint(0,30)
        self.get_logger().info(f"Generated number: {num}")

        # Publishing coordinates and object detection flag only if number is 13.
        if num == 13:
            msg = Bool()
            msg.data = True
            self.detection_publisher.publish(msg)
            self.get_logger().info("Object Detected") # Publishing object detection flag.

            # Converting target coordinates to stamped pose message. 
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(self.coord[0])
            pose.pose.position.y = float(self.coord[1])
            pose.pose.orientation.w = 1.0
            self.coords_publisher.publish(pose)

            # Pusing random number generation for a while 
            self.active = False
            self.create_timer(10.0,self.resume_rng)

    # Function to resume random number generation after sleep time.
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
