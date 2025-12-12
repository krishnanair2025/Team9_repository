# Terminal command to activate frontier exploration from terminal
# ros2 service call /explore_status example_interfaces/srv/SetBool "{data: true}"

# Imports

import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from example_interfaces.srv import SetBool
import numpy as np
import time


""" 
Logic:

Node waits for service call from client in task manager, once receiving a True boolean request, it starts 
frontier exploration by firstly shortlisting all free cells in the occupancy grid which has unknown cells around it.
Then, it checks which of these free cells could be potentially safe targets for the rover based on the rover's
footprint. Out of all the shortlisted frontiers, a random choice is made. This is converted to a pose and sent
as a goal to the Nav2 action server. Once the goal is reached, another function makes the rover spin on the spot
in order to maximise the chances of spotting a target object. If the task manager client calls with a False 
boolean request at any point, the Nav2 goal is cancelled and rover is stopped if spinning.

"""


class FrontierExplorationNode(Node):

    """ 
    ROS2 Node that repeatedly sends frontier goals to Nav2 while considering the costmap and robot footprint, 
    its state is triggered using a service call from the task manager. It can be paused or started as required.
    Once the rover reaches the target, it spins on the spot at the required angular velocity and duration.
    """

    def __init__(self):
        super().__init__('frontier_exploration_node')

        # Action client for nav2
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose')
        

        # Subscriber to get map from SLAM
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscriber to get global costmap
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Service to control state of the frontier explorer
        self.exploration_service = self.create_service(
            srv_type = SetBool,
            srv_name= '/explore_status',
            callback = self.exploration_server_callback
        )

        # Publisher for command velocities to the rover 
        self.spin_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal variables
        self._current_goal_handle = None 

        # SLAM map data + info
        self.map_data = None
        self.map_info = None

        # Costmap data + info
        self.costmap_data = None
        self.costmap_info = None

        # Status of the exploration service
        self.exploring = False

        # Tracking active goal
        self.goal_active = False




    # Callback for request from task manager to start/stop exploration service 
    def exploration_server_callback(self, request, response):

        """ 
        This callback function takes a service request from the task manager and based on the set boolean
        value, starts or stops frontier exploration. Current goal is cancelled if frontier exploration is stopped.
        """

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




    # Callback for the costmap subscriber
    def costmap_callback(self, msg):

        """
        This callback function stores the costmap data and info in internal variables for later reference 
        """

        # Extracting map information
        self.costmap_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = data.reshape((msg.info.height, msg.info.width))




    # Callback for accessing the occupancy grid as well as calling the pick frontier and send goal method
    def map_callback(self, msg):

        """
        This callback function triggers frontier picking whenever the map is updated by SLAM but only if 
        the exploration service is active and there is no current goal being pursued 
        """

        # Extracting and storing map information in internal variables
        self.map_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.map_data = data.reshape((msg.info.height, msg.info.width))


        # Only picks and sends a goal if explorer is active and there is no current goal being pursued.
        if (self.exploring == True) and (self.goal_active == False):

            self.get_logger().info('Picking new frontier')

            # calling function to select frontier
            frontier = self.pick_frontier()

            # checking that frontier is not empty
            if frontier is not None:
                # calling send_goal function with the chosen frontier as argument
                pose = self.frontier_to_pose(frontier)
                self.send_goal(pose)




    # Method to pick frontier from the occupancy grid
    def pick_frontier(self):

        """
        This function picks the next suitable frontier to be pursued by the rover
        """

        # Skip if map is not available
        if self.map_data is None:
            return None
        
        # Skip if costmap is not available
        if self.costmap_data is None:
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

        # Rover's height and width including additional 50mm buffer
        rover_width = (448/1000)+0.1
        rover_length = (425/1000)+0.1

        # Calculating rover's dimensions w.r.t map's cells
        res = self.costmap_info.resolution
        rover_width_cells = int(np.ceil(rover_width / res)) 
        rover_length_cells = int(np.ceil(rover_length / res)) 

        # Checking whether no frontiers are found
        if len(points) == 0:
            self.get_logger().info('No frontiers found')
            self.exploring = False
            return None
        
        # Initialising list for frontiers where the rover's footprint does not overlap with obstacles or inflation zones on the costmap
        safe_frontiers = []

        height,width = self.costmap_data.shape

        # Iterating through all the frontiers to identify safe ones
        for candidate in points:
            y,x = candidate 

            # Extracting coordinates of the four corners binding the robot's footprint centred at the frontier
            x_start = int(x - rover_width_cells/2)
            x_end = int(x + rover_width_cells/2)
            y_start = int(y - rover_length_cells/2)
            y_end = int(y + rover_length_cells/2)

            # Skipping if the coordinates fall outside of the map
            if x_start < 0 or y_start < 0 or x_end >= width or y_end >= height:
                continue

            # Taking a mask of the costmap using robot's footprint
            footprint_mask = self.costmap_data[y_start:y_end+1, x_start:x_end+1]

            # Checking that all the costmap cell values in the mask are 0 (free cells)
            if np.all(footprint_mask == 0):
                safe_frontiers.append(candidate)

        # Checking if there are no suitable frontiers
        if len(safe_frontiers)==0:
            self.get_logger().info('No safe frontiers found')
            self.exploring = False
            return None

        # Randomly picking one frontier from the safe frontiers list
        idx = np.random.choice(len(safe_frontiers))
        return safe_frontiers[idx]




    # Method which converts chosen frontier index to a pose for nav2
    def frontier_to_pose(self,cell):

        """
        This method takes the chosen frontier and converts it to a Pose
        """

        y,x = cell
        res = self.map_info.resolution 
        origin = self.map_info.origin.position

        # Converting cell coordinates to world coordinates
        wx = origin.x + x * res
        wy = origin.y + y * res

        # Preparing and returning pose
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

        """
        This function calls the nav2 action server and sends it the chosen pose as a goal.
        """

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Marked as currently pursuing a goal
        self.goal_active = True 




    # callback for nav2 action server's response
    def goal_response_callback(self, future):

        """
        This callback function handles the nav2 action server's response
        """

        goal_handle = future.result()
        self._current_goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected, attempting to pick new goal")
            self._current_goal_handle = None
            self.goal_active = False
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)




    # callback for nav2 action server's result
    def get_result_callback(self, future):

        """
        This callback function gets the result of the nav2 action server request and if the goal was 
        successfully reached, it calls another function to make the rover spin on the spot with stated 
        angular velocity and duration
        """

        result = future.result().result
        self.goal_active = False

        # If succcessful, calls function to spin rover
        if future.result().status == 4:
            self.spin_in_place(angular_speed=1.0, duration=5.0)




    # callback for cancelling a goal
    def cancel_done_callback(self, future):

        """
        This is a callback for cancelling the goal, in cases where the task manager requests to stop the 
        frontier exploration service while a goal is being pursued by nav2.
        """

        result = future.result()
        if result:
            self.get_logger().info("Current goal cancelled successfully")
        else:
            self.get_logger().warn("Failed to cancel current goal")
        
        # Clear the goal handle after cancelling
        self._current_goal_handle = None




    # Function to make rover spin in place
    def spin_in_place (self,angular_speed = 0.5, duration = 5.0):

        """
        This function prepares and publishes a Twist type message to make the rover spin on the spot,
        if the explorer is paused while spinning or if the duration is met, a 0 angular velocity command is 
        published to stop spinning the rover.
        """

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed

        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            if not self.exploring:
                break
            self.spin_pub.publish(twist)
            time.sleep(0.05)  # publish at ~20 Hz

        # Stop spinning
        twist.angular.z = 0.0
        self.spin_pub.publish(twist)
        self.get_logger().info("Spin completed!")



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

