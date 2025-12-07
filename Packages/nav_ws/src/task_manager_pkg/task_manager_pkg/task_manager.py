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
    def __init__(self):
        super().__init__('task_manager_node')

        self.state = "IDLE"
        self.get_logger().info(f"Robot in {self.state} state")
        self.object_count = 0
        self.startx = 0.3
        self.starty = 1.0
        self.startw = 1.0

        self.activate_rover = self.create_service(
            Trigger,
            '/trigger_rover',
            self.trigger_callback,
        )

        self.reached_sub = self.create_subscription(
            Bool,
            '/reached_object',
            self.reached_object_callback,
            10
        )
        
        self.detection_subscription = self.create_subscription(
            Bool,
            'detected_flag',
            self.callback_detected,
            10
        )

        self.object_location_subscription = self.create_subscription(
            PoseStamped,
            'object_location',
            self.object_location_callback,
            10
        )

        self.exploration_client = self.create_client(
            SetBool,
            '/explore_status',
        )


        while not self.exploration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /explore status service...")

        self.move_to_coord_client = self.create_client(
            SetBool,
            '/move_to_coord',
        )

        while not self.move_to_coord_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /move_to_coord status service...")


    def trigger_callback(self, request, response):

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

    def callback_detected(self,msg):
        if self.state == "EXPLORING":
            if msg.data == True:
                self.get_logger().info("Detection received! Sending explorer pause command...")
                self.pause_exploration()

                self.state = "APPROACH_OBJECT"
                self.get_logger().info(f"Robot in {self.state} state")
                self.move_rover_to_coords()


    def pause_exploration(self):
        req = SetBool.Request()
        req.data = False

        future = self.exploration_client.call_async(req)
        future.add_done_callback(self.service_response)

    def service_response(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"/explorer responded: success={response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def object_location_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        w = msg.pose.orientation.w  
        self.get_logger().info(f"Object location -> x: {x}, y: {y}, w: {w}")

    def move_rover_to_coords(self):
        req = SetBool.Request()
        req.data = True

        future = self.move_to_coord_client.call_async(req)
        future.add_done_callback(self.service_response)     

    def reached_object_callback(self,msg):
        if self.state == "APPROACH_OBJECT":
            if msg.data == True:
                self.get_logger().info("Rover has reached the object")
                self.object_count = self.object_count+1
                self.picking_time()

    def picking_time(self):
        self.pick_timer = self.create_timer(30.0,self.continue_exploration)
        self.get_logger().info("Simulating object picking time")
        self.state = "PICKING"
        self.get_logger().info(f"Robot in {self.state} state")


    def continue_exploration(self):
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
