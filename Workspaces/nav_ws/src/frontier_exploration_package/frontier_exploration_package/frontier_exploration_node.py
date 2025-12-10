import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from example_interfaces.srv import SetBool
import numpy as np

class FrontierExplorationNode(Node):
    """ ROS2 Node that repeatedly sends frontier goals to Nav2, its state is triggered using a service call
    from the task manager. It can be paused or started as required. """

    def __init__(self):
        super().__init__('frontier_exploration_node')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Initialising subscriber to get occupancy grid from SLAM
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Initialising service to control state of the frontier explorer
        self.exploration_service = self.create_service(
            srv_type = SetBool,
            srv_name= '/explore_status',
            callback = self.exploration_client_callback
        )

        # Internal variables
        self._current_goal_handle = None
        self.startx = 0.3
        self.starty = 1.0
        self.map_data = None
        self.map_info = None
        self.exploring = False
        self.goal_active = False


    # Callback for request to start or stop explorer
    def exploration_client_callback(self, request, response):
        self.exploring = request.data
        response.success = True

        if self.exploring == True:
            response.message = f"Explorer running"
            self.get_logger().info(response.message)
        else:
            response.message = f"Explorer paused"
            self.get_logger().info(response.message)

            if self._current_goal_handle is not None:
                self.get_logger().info("Cancelling current goal...")
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)

        return response

    # Callback for accessing the occupancy grid as well as calling the pick frontier and send goal method
    def map_callback(self, msg):
        # Extracting map information
        self.map_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.map_data = data.reshape((msg.info.height, msg.info.width))

        # Only picks and sends a goal if explorer is active and there is no current goal being pursued.
        if (self.exploring == True) and (self.goal_active == False):

            self.get_logger().info('Picking new frontier')

            # calling function to select frontier
            frontier = self.pick_frontier()

            # checking if frontier is not empty
            if frontier is not None:
                # calling send_goal function with pose as argument
                pose = self.frontier_to_pose(frontier)
                self.send_goal(pose)
    
    # Method to pick frontier from the occupancy grid
    def pick_frontier(self):
        if self.map_data is None:
            return None

        # Identify free and unknown cells
        free = (self.map_data == 0)
        unknown = (self.map_data == -1)

        # Pad unknown map with zeros to avoid wrap-around at edges
        padded_unknown = np.pad(unknown, pad_width=1, mode='constant', constant_values=0)

        # Initialize frontier mask
        frontier_mask = np.zeros_like(free, dtype=bool)

        # Check all 4 neighbors (up, down, left, right)
        frontier_mask = free & (
            padded_unknown[0:-2, 1:-1] |  # up
            padded_unknown[2:, 1:-1]   |  # down
            padded_unknown[1:-1, 0:-2] |  # left
            padded_unknown[1:-1, 2:]      # right
        )

        # Find frontier coordinates
        points = np.argwhere(frontier_mask)

        if len(points) == 0:
            self.get_logger().info('No frontiers found')
            self.exploring = False
            return None

        # Randomly pick one frontier
        idx = np.random.choice(len(points))
        return points[idx]

    
    # Method which converts chosen frontier index to a pose for nav2
    def frontier_to_pose(self,cell):
        y,x = cell
        res = self.map_info.resolution 
        origin = self.map_info.origin.position

        wx = origin.x + x * res
        wy = origin.y + y * res

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.orientation.w = 1.0
        self.get_logger().info(f"x = {wx}")
        self.get_logger().info(f"y = {wy}")
        return pose

    # Method used to send goal to nav2's action server
    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.goal_active = True


    # callback for nav2 action server's response
    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._current_goal_handle = goal_handle  # <-- store goal handle

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected, attempting to pick new goal")
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    # callback for nav2 action server's result
    def get_result_callback(self, future):
        result = future.result().result
        self.goal_active = False

    # callback for cancelling a goal
    def cancel_done_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info("Current goal cancelled successfully")
        else:
            self.get_logger().warn("Failed to cancel current goal")
        
        # Clear the goal handle after cancelling
        self._current_goal_handle = None


def main(args = None):
    try: 
        rclpy.init(args=args)
        node = FrontierExplorationNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()

