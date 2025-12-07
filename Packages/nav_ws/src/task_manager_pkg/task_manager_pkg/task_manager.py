# Terminal command to start rover
# ros2 service call /trigger_rover example_interfaces/srv/Trigger "{}"

# Terminal command to launch rover in gazebo with task manager active
# ros2 launch leo_gz_bringup task_manager.launch.py | grep task_manager

import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool
from example_interfaces.srv import SetBool, Trigger
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class TaskManagerNode(Node):

    """ ROS2 Node that acts as a task manager and orchestrates the overall behaviour of the rover
    during its mission to pick and retrieve coloured objects from a search area """


    def __init__(self):
        super().__init__('task_manager_node')

        # Internal variables
        self.state = "IDLE"
        self.get_logger().info(f"Robot in {self.state} state")
        self.object_count = 0
        self.startx = 0.3
        self.starty = 1.0
        self.startw = 1.0

        # Initialising service server to receive trigger call from terminal to start rover
        self.activate_rover = self.create_service(
            Trigger,
            '/trigger_rover',
            self.trigger_callback,
        )

        # Subscriber to check whether move_to_coord node has succesfully moved rover to object
        self.reached_sub = self.create_subscription(
            Bool,
            '/reached_object',
            self.reached_object_callback,
            10
        )
        
        # Subscriber to monitor whether object detection node has detected a coloured object
        self.detection_subscription = self.create_subscription(
            Bool,
            'detected_flag',
            self.callback_detected,
            10
        )

        # Subscriber to monitor location of the object detected by the object detection node
        self.object_location_subscription = self.create_subscription(
            PoseStamped,
            'object_location',
            self.object_location_callback,
            10
        )

        # Service client to call frontier exploration node with request to start or pause exploring
        self.exploration_client = self.create_client(
            SetBool,
            '/explore_status',
        )

        while not self.exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /explore status service...")

        # Service client to call move_to_coord node with request to move rover to target coordinates
        self.move_to_coord_client = self.create_client(
            SetBool,
            '/move_to_coord',
        )

        while not self.move_to_coord_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /move_to_coord status service...")


    # Callback function for trigger service call from terminal to start rover
    def trigger_callback(self, request, response):

        """ If the trigger service call is made while the rover is in IDLE state, the 
        rover is switched to EXPLORE state and a service call is made to frontier_exploration
        node to start exploration """

        if self.state == "IDLE":
            self.get_logger().info("Exploration triggered -> Starting exploration ... ")

            req = SetBool.Request()
            req.data = True

            future = self.exploration_client.call_async(req)
            future.add_done_callback(self.service_response)

            response.success = True
            response.message = "Exploration start command sent."

            self.state = "EXPLORING"
            self.get_logger().info(f"Robot in {self.state} state")

        else:
            response.success = False
            response.message = "Exploration already running."

        return response


    # Callback function for detection flagged by objection detection node
    def callback_detected(self,msg):

        """ When the object detection node flags that a coloured object has been detected, 
        if the rover is in EXPLORE mode, it is switched to "APPROACH_OBJECT" mode, two other 
        methods are called to pause exploration and to move rover to target coords"""

        if self.state == "EXPLORING":
            if msg.data == True:
                self.get_logger().info("Detection received! Sending explorer pause command...")
                self.pause_exploration()

                self.state = "APPROACH_OBJECT"
                self.get_logger().info(f"Robot in {self.state} state")
                self.move_rover_to_coords()


    # Method to pause exploration by calling frontier exploration node
    def pause_exploration(self):

        """ Makes a service call to frontier exploration node to pause exploration """

        req = SetBool.Request()
        req.data = False

        future = self.exploration_client.call_async(req)
        future.add_done_callback(self.service_response)


    # Method to check response of frontier_exploration node
    def service_response(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"/explorer responded: success={response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


    # Method to retrieve object coordinates published by object detection node
    def object_location_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        w = msg.pose.orientation.w  
        self.get_logger().info(f"Object location -> x: {x}, y: {y}, w: {w}")


    # Method to trigger move_to_coord service 
    def move_rover_to_coords(self):

        """ Makes a service call to move_to_coord node to move the rover to coordinates 
        published by object detection node """

        req = SetBool.Request()
        req.data = True

        future = self.move_to_coord_client.call_async(req)
        future.add_done_callback(self.service_response)     

    # Callback to check whether move_to_coord has succesfully moved rover to target location
    def reached_object_callback(self,msg):

        """ Checks if move_to_coord node has published that the rover has succesfully moved
        to the target coordiantes """

        if self.state == "APPROACH_OBJECT":
            if msg.data == True:
                self.get_logger().info("Rover has reached the object")
                self.object_count = self.object_count+1 # Incrementing number of objects picked
                self.picking_time()

    # Simulating time taken for rover to pick object
    def picking_time(self):

        """ Simulates the time taken for the manipulator to pick and place the object in the 
        onboard storage bin using a 30s timer """

        self.pick_timer = self.create_timer(30.0,self.continue_exploration)
        self.get_logger().info("Simulating object picking time")
        self.state = "PICKING"
        self.get_logger().info(f"Robot in {self.state} state")

    # Method to continue frontier exploration 
    def continue_exploration(self):

        """ Checks if the number of objects collected is 3, if it is 3, the rover stops. 
        If less than 3, the rover makes a call to frontier exploration to continue exploration """

        self.get_logger().info(f"Rover has collected {self.object_count} objects.")

        self.pick_timer.cancel()

        if self.object_count >= 3:
            self.state = "RETURNING TO START"
            self.get_logger().info("Rover has collected all three objects")
            self.get_logger().info(f"Robot in {self.state} state")

            return

        if self.state != "EXPLORING":
            req = SetBool.Request()
            req.data = True

            future = self.exploration_client.call_async(req)
            future.add_done_callback(self.service_response)
            self.state = "EXPLORING"
            self.get_logger().info(f"Robot in {self.state} state")



def main(args = None):
    try: 
        rclpy.init(args=args)
        node = TaskManagerNode()
        rclpy.spin(node)
    
    except KeyboardInterrupt:

        pass

    except Exception as e:

        print(e)

if __name__ == '__main__':
    main()
