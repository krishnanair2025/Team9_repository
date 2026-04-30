import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool


class MoveToCoord(Node):

    def __init__(self):
        super().__init__('move_to_coord_node')

        self.latest_pose = None
        self.moving = False  # Track whether robot is currently navigating

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Trigger service
        self.move_to_coord_service = self.create_service(
            Trigger,
            '/move_to_coord',
            self.move_to_coord_callback
        )

        # Subscribe to target location
        self.coords_sub = self.create_subscription(
            PoseStamped,
            '/target_location',
            self.coord_retrieval_callback,
            10
        )

        # Publisher for result
        self.reached_pub = self.create_publisher(
            Bool,
            '/reached_object',
            10
        )

        self.get_logger().info("MoveToCoord node ready.")

    # Store latest target coordinates
    def coord_retrieval_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.get_logger().info(
            f"Updated target coords: x={msg.pose.position.x}, y={msg.pose.position.y}"
        )

    # Trigger service callback
    def move_to_coord_callback(self, request, response):

        if self.moving:
            response.success = False
            response.message = "Robot already moving."
            return response

        if self.latest_pose is None:
            response.success = False
            response.message = "No coordinates available yet."
            self.get_logger().warn("Cannot start navigation: no target_location received.")
            return response

        self.get_logger().info("Trigger received. Starting navigation.")
        self.moving = True
        self.send_goal(self.latest_pose)

        response.success = True
        response.message = "Navigation started."
        return response

    # Send goal to Nav2
    def send_goal(self, pose_msg: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    # Goal accepted/rejected
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")

            self.moving = False  # reset state

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

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Goal reached successfully!")
            msg.data = True
        else:
            self.get_logger().warn(f"Navigation failed with status={result.status}")
            msg.data = False

        self.reached_pub.publish(msg)

        self.moving = False  # reset state after completion


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MoveToCoord()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()