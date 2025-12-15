import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from example_interfaces.srv import SetBool, Trigger
from std_msgs.msg import Bool
import numpy as np
import math

class ApproachObjNode(Node):
    """ ROS2 Node that calculates the most optimal approach point towards a detected object. 
    Requires: target object pose, current rover pose, costmap"""

    def __init__(self):
        super().__init__('approach_object_node')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.approach_obj_server = self.create_service(
            srv_type = Trigger,
            srv_name= "/approach_object",
            callback=self.approach_object_server_callback
        )

        # Intialising subscriber to get global costmap
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Subscriber to receive object coordinates from message published by object detection node.
        self.coords_sub = self.create_subscription(
            PoseStamped,
            '/object_location',
            self.coord_retrieval_callback,
            10
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
            '/approach_success',
            10
        )

        # Publisher for command velocity
        self.velocity_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.robot_pose_callback)  # 10 Hz

        self.x = None
        self.distance = None
        self.refining = None

        self.retry_count = 0

        self.c_box = None # Coordinate of the target object, to be filled in when pose is received
        self.box_dim = 205/1000
        self.box_radius = np.sqrt((self.box_dim**2)+(self.box_dim**2))/2
        self.rover_dims = [448/1000,425/1000]
        self.buffer = 50/1000
        self.rover_radius = self.buffer + (np.sqrt((self.rover_dims[0]**2) + (self.rover_dims[1]**2)))/2
        self.rover_coords = None # current coordinate of rover, to be filled in when received
        self.optimal_radius = self.rover_radius + self.box_radius

        self.costmap_data = None
        self.costmap_info = None

        self.latest_pose = None
        self.candidate_points = []
        self.approach_point = None
        self.heading_angle = None

        self.active = False
        self.refinement_done = False



    # Callback for accessing the costmap
    def costmap_callback(self, msg):
        # Extracting map information
        self.costmap_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = data.reshape((msg.info.height, msg.info.width))


    # Callback for request to start or stop explorer
    def approach_object_server_callback(self,request,response):

        if self.latest_pose is None:
            response.success = False
            response.message = "No coordinates available yet."
            self.get_logger().warn("Cannot start navigation: no object_location received.")
            return response
        
        self.active = True

        if self.active:
            self.main_process()

            response.success = True
            response.message = "Approach request received and goal sent"

            return response
    
    def main_process(self):
        
        self.candidate_points = [] # clearing old candidate points
        
        self.find_candidate_points(self.latest_pose) # calling function to find new candidate points

        self.approach_point_picker()
        
        approach_pose = self.point_to_pose()
        
        self.send_goal(approach_pose)
    

    # Store latest object coordinates
    def coord_retrieval_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.get_logger().info(
            f"Updated target coords: x={msg.pose.position.x}, y={msg.pose.position.y}"
        )

    def robot_pose_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y

            self.rover_coords = [robot_x,robot_y]
        except Exception:
            pass


    def find_candidate_points(self,msg:PoseStamped):
        wx_box = msg.pose.position.x
        wy_box = msg.pose.position.y

        self.c_box = [wx_box,wy_box]

        res = self.costmap_info.resolution 
        origin = self.costmap_info.origin.position

        box_gx = (wx_box-origin.x)/res
        box_gy = (wy_box-origin.y)/res

        radius_cells = self.optimal_radius/res

        height,width = self.costmap_data.shape

        for deg in range(360):
            rad = math.radians(deg)
            gx_candidate = box_gx + radius_cells * math.cos(rad)
            gy_candidate = box_gy + radius_cells * math.sin(rad)

            if 0 <= gx_candidate < width and 0 <= gy_candidate < height:
                self.candidate_points.append([gx_candidate, gy_candidate])


    def approach_point_picker(self):
        if self.rover_coords is None:
            self.get_logger().warn("Current rover pose not yet received, cannot pick approach point")
            return

        safe_points = []
        res = self.costmap_info.resolution 
        origin = self.costmap_info.origin.position
        
        self.get_logger().info(f"Robot currently at {self.rover_coords}")

        rover_width_cells = int(np.ceil((self.rover_dims[0]+2*self.buffer)/ res)) 
        rover_length_cells = int(np.ceil((self.rover_dims[1]+2*self.buffer)/ res)) 

        height,width = self.costmap_data.shape

        for point in self.candidate_points:
            gx_candidate,gy_candidate = point

            gx_start = int(gx_candidate - rover_width_cells/2)
            gx_end = int(gx_candidate + rover_width_cells/2)
            gy_start = int(gy_candidate - rover_length_cells/2)
            gy_end = int(gy_candidate + rover_length_cells/2)

            if gx_start < 0 or gy_start < 0 or gx_end >= width or gy_end >= height:
                continue

            footprint_mask = self.costmap_data[gy_start:gy_end+1, gx_start:gx_end+1]

            if np.all((footprint_mask < 100) | (footprint_mask == 255)):
                safe_points.append(point)

        if not safe_points:
            self.get_logger().info("No safe points found")
            return 

        # Converting robot position from world coordiantes to map cell coordiantes
        rover_cells_x = (self.rover_coords[0]-origin.x)/res
        rover_cells_y = (self.rover_coords[1]-origin.y)/res

        distances = []

        for coords in safe_points:
            distance_to_coord = np.sqrt((abs(coords[0]-rover_cells_x))**2 + (abs(coords[1]-rover_cells_y))**2)
            distances.append(distance_to_coord)

        #self.get_logger().info(f"Distances list = {distances}")

        approach_point = safe_points[distances.index(min(distances))] # picking the point in the middle out of all the safe points
        #self.get_logger().info(f"Distance to chosen point = {min(distances)}")

        wx_approach = origin.x + approach_point[0] * res
        wy_approach = origin.y + approach_point[1] * res

        self.approach_point = [wx_approach,wy_approach]

        self.heading_angle = math.atan2(self.c_box[1]-self.approach_point[1],self.c_box[0]-self.approach_point[0])

    def point_to_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.approach_point[0])
        pose.pose.position.y = float(self.approach_point[1])

        yaw = self.heading_angle
        pose.pose.orientation.z = math.sin(yaw/2.0)
        pose.pose.orientation.w = math.cos(yaw/2.0)

        #self.get_logger().info(f"x = {self.approach_point[0]}")
        #self.get_logger().info(f"y = {self.approach_point[1]}")
        #self.get_logger().info(f"heading = {math.degrees(self.heading_angle)} degrees")
        return pose

    def send_goal(self, pose: PoseStamped):
        """Send a navigation goal to Nav2 with full logging."""

        # Wait until the action server is available
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal → x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, heading={math.degrees(self.heading_angle):.2f}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal.")
            return

        self.get_logger().info("Nav2 accepted the goal.")

        # Request result callback
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can log minimal feedback
        self.get_logger().debug(
            f"Nav2 feedback: distance_remaining={feedback.distance_remaining:.2f}"
        )

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Rover successfully reached the approach point! Starting refinement")
            self.align_timer = self.create_timer(0.05, self.align_object)

            self.get_logger().info("Refinement complete, rover in position for pickup")

        elif status == 5:  # CANCELED
            self.get_logger().warn("Goal was cancelled. Trying again")
            self.retry_count = self.retry_count+1
            if self.retry_count < 3:
                self.main_process()
            
            
            msg = Bool()
            msg.data = False
            self.success_pub.publish(msg)
            self.active = False

        elif status == 6:  # ABORTED
            self.get_logger().error("Nav2 aborted the goal. Trying again")
            self.retry_count = self.retry_count+1
            if self.retry_count < 3:
                self.main_process()

            msg = Bool()
            msg.data = False
            self.success_pub.publish(msg)
            self.active = False
                
        else:
            self.get_logger().error(f"Unknown Nav2 result status: {status}. Trying again")
            self.retry_count = self.retry_count+1
            if self.retry_count < 3:
                self.main_process()

            msg = Bool()
            msg.data = False
            self.success_pub.publish(msg)
            self.active = False


    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.distance = msg.pose.position.z


    def align_object(self):

        if self.x is None:
            return

        twist = Twist()
        twist.angular.z = 0.0

        if abs(self.x) > 0.02:
            twist.angular.z = 0.1 if self.x < 0 else -0.1
            self.get_logger().info(f"Rotating {'left' if self.x < 0 else 'right'}")
            self.velocity_pub.publish(twist)
        else:
            self.get_logger().info("Object centered")
            twist.angular.z = 0.0
            self.velocity_pub.publish(twist)
            self.move_to_obj_timer = self.create_timer(0.05,self.move_to_obj)
            self.align_timer.cancel()
            #self.refining = False  # stop further rotation

    def move_to_obj(self):
        if self.distance is None:
            return
        
        twist = Twist()

        if self.distance > 0.2:
            twist.linear.x = 0.1
            self.velocity_pub.publish(twist)
            self.get_logger().info(f"Moving closer, current distance = {self.distance}")

        else:
            self.get_logger().info("Rover is close to object")
            twist.linear.x = 0.0
            self.velocity_pub.publish(twist)
            self.move_to_obj_timer.cancel()
            
            # Pubishing success message to task manager
            self.get_logger().info("Rover in position for pickup")
            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)
            self.active = False


def main(args = None):
    try: 
        rclpy.init(args=args)
        node = ApproachObjNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()