import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool

class BackupNode(Node):
    """ ROS2 Node that moves rover back """

    def __init__(self):
        super().__init__('back_up_node')

        self.approach_obj_server = self.create_service(
            srv_type = Trigger,
            srv_name= "/backup",
            callback=self.backup_server_callback
        )

        # Subscriber to pose of object in camera's frame
        self.pose_pub = self.create_subscription(
            PoseStamped,
            'cam_obj_loc',
            self.pose_callback,
            10
        )

        # Publisher for approach success
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

        self.distance = None
        self.active = False
        self.timer = None

    def backup_server_callback(self,request,response):
        if self.active:
            response.success = False
            response.message = "Backup already in progress"
            return response
        
        self.get_logger().info("Backup requested")
        self.active = True

        self.timer = self.create_timer(0.05,self.reverse)

        response.success = True
        response.message = "Backing up rover"

        return response

    def pose_callback(self,msg:PoseStamped):
        self.distance = msg.pose.position.z

    def reverse(self):

        if self.distance is None or not self.active:
            return

        twist = Twist()
        if self.distance < 0.4:
            twist.linear.x = -0.1
            self.get_logger().info(f"Backing up rover, current distance = {self.distance}")
            self.velocity_pub.publish(twist)
        else:
            twist.linear.x = 0.0
            self.get_logger().info("Rover at sufficient distance from object")
            self.velocity_pub.publish(twist)
            self.active = False

            msg = Bool()
            msg.data=True
            self.success_pub.publish(msg)

            self.active = False

            if self.timer:
                self.timer.cancel()
                self.timer = None


def main(args = None):
    try: 
        rclpy.init(args=args)
        node = BackupNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()