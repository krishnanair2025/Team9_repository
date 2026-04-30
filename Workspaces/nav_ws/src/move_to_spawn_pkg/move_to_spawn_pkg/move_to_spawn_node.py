import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool


class MoveToSpawn(Node):

    def __init__(self):
        super().__init__('move_to_spawn_node')

        self.moving = False

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Trigger service
        self.move_to_spawn_service = self.create_service(
            Trigger,
            '/move_to_spawn',
            self.move_to_spawn_callback
        )

        # Publisher for result
        self.reached_pub = self.create_publisher(
            Bool,
            '/reached_spawn',
            10
        )

        self.get_logger().info("MoveToSpawn node ready.")

    # Trigger service callback
    def move_to_spawn_callback(self, request, response):

        if self.moving:
            response.success = False
            response.message = "Robot already moving."
            return response

        self.get_logger().info("Trigger received. Sending robot to spawn (0,0).")
        self.moving = True
        self.send_goal()

        response.success = True
        response.message = "Navigation to spawn started."
        return response

    # Send fixed goal (0,0)
    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # Goal accepted/rejected
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")

            self.moving = False

            fail_msg = Bool()
            fail_msg.data = False
            self.reached_pub.publish(fail_msg)
            return

        self.get_logger().info("Goal accepted by Nav2.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # Result callback
    def get_result_callback(self, future):
        result = future.result()

        msg = Bool()

        if result.status == 4:
            self.get_logger().info("Spawn reached successfully!")
            msg.data = True
        else:
            self.get_logger().warn(f"Navigation failed with status={result.status}")
            msg.data = False

        self.reached_pub.publish(msg)
        self.moving = False


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MoveToSpawn()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()