# Terminal command to start rover
# ros2 service call /trigger_rover example_interfaces/srv/Trigger "{}" 

# Terminal command to launch rover in gazebo with task manager active
# ros2 launch leo_gz_bringup task_manager.launch.py | grep task_manager

# Imports 
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from example_interfaces.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped 
import time

"""
Logic:

This task manager node acts as the main orchestrator for the rover's mission. It is activated using a trigger service
call which acts like a way of switching on the rover at the start of the mission. When activated, the task manager 
switches from IDLE to EXPLORE state. When entering explore state, it calls the service in the frontier exploration node.
If a detection flag is published by the object detection node, exploration is stopped and task manager enters APPROACH 
state. The task manager calls the service in the approach object node which sends the rover to the most ideal approach
point near the target object's coordinates. Currently, the task manager simulates the time taken to pick the object using
a 30 second timer before switching to explore state again but only if the number of objects collected is less than 3. 
If all three objects have been collected, the task manager calls the service in the move_to_coords node which sends
the rover back to the starting point for sorting. 

"""

class TaskManagerNode(Node):

    """
    ROS2 Node that orchestrates the overall behaviour of the rover during its mission to pick and retrieve 
    coloured objects from a search area 
    """

    def __init__(self):
        super().__init__('task_manager_node')

        ########## SERVICE SERVER ##########

        # For trigger call from terminal to start robot
        self.activate_rover = self.create_service(
            Trigger,
            '/trigger_rover',
            self.trigger_mission_callback,
        )


        ########## SERVICE CLIENTS ##########

        # To start/pause frontier exploration
        self.exploration_client = self.create_client(
            SetBool,
            '/explore_status',
        )

        # To start approach_obj service
        self.approach_object_client = self.create_client(
            Trigger,
            '/approach_object',
        )

        # To start manipulator service
        self.manipulator_trigger_client = self.create_client(
            Trigger,
            '/trigger_arm',
        )

        # To call backup service
        self.backup_client = self.create_client(
            Trigger,
            '/backup',
        )

        
        # Service client to call move_to_coord node with request to move rover to target coordinates
        self.move_to_spawn_client = self.create_client(
            Trigger,
            '/move_to_spawn',
        )


        ########## SUBSCRIBERS ##########

        # To check whether move_to_coord node has succesfully moved rover to coordinates
        self.reached_sub = self.create_subscription(
            Bool,
            '/approach_success',
            self.reached_object_callback,
            10
        )
        
        # To check whether move_to_coord node has succesfully moved rover to coordinates
        self.reached_sub = self.create_subscription(
            Bool,
            '/manipulator_success',
            self.manipulator_success_callback,
            10
        )

        # To monitor whether object detection node has detected a coloured object
        self.detection_sub = self.create_subscription(
            Bool,
            'detected_flag',
            self.callback_detected,
            10
        )

        # To monitor backup success 
        self.backup_success_sub = self.create_subscription(
            Bool,
            'backup_success',
            self.backup_success_callback,
            10
        )

        # To monitor return to spawn success 
        self.reached_spawn_sub = self.create_subscription(
            Bool,
            'reached_spawn',
            self.reached_spawn_callback,
            10
        )

        # Publisher for slot 
        self.slot_pub = self.create_publisher(
            Int32,
            '/slot',
            10
        )

        ########## INTERNAL VARIABLES ##########
        self.state = "IDLE"
        self.get_logger().info(f"Robot in {self.state} state")
        self.object_count = 0 #replace with array of colours in order collected
        self.slot_number = 1

        # Status variables
        self.mission_active = False
        self.object_seen = False
        self.explorer_active = False
        self.approaching = False
        self.approach_success = False
        self.arm_active = False
        self.backing_up = False
        self.returning = False

        # Mission timing variables
        self.start_time = None
        self.end_time = None

        # Timer for main mission loop
        self.mission_timer = self.create_timer(0.1,self.mission_state)

        # Timer for publishing slot number
        self.slot_timer = self.create_timer(0.1,self.publish_slot)

    #################### MAIN STATE MACHINE LOOP ####################

    def mission_state(self):
        if not self.mission_active:
            self.switch_state("IDLE")
            return
        
        if self.state == "IDLE":
            self.execute_idle()

        elif self.state == "EXPLORE":
            self.execute_explore()

        elif self.state == "APPROACH":
            self.execute_approach()

        elif self.state == "PICKUP":
            self.execute_pickup()
        
        elif self.state == "BACKING_UP":
            self.execute_backup()

        elif self.state == "RETURN_TO_START":
            self.execute_return()

    
    #################### STATE EXECUTE FUNCTIONS ####################

    def execute_idle(self):
        self.get_logger().info(f"Robot idle")

    def execute_explore(self):

        if not self.exploration_client.service_is_ready():
            self.get_logger().warn("Exploration service not ready yet")
            return

        if not self.explorer_active:
            # Activating frontier exploration service
            explore_req = SetBool.Request()
            explore_req.data = True

            future = self.exploration_client.call_async(explore_req)
            future.add_done_callback(self.exploration_response)
            self.get_logger().info("Exploration start request sent...")

            self.explorer_active = True

    def execute_approach(self):

        if not self.approach_object_client.service_is_ready():
            self.get_logger().warn("Approach service not ready yet")
            return
        
        if self.explorer_active:
            # Deactivating frontier exploration service
            explore_req = SetBool.Request()
            explore_req.data = False

            future = self.exploration_client.call_async(explore_req)
            future.add_done_callback(self.exploration_response)
            self.get_logger().info("Exploration pause request sent...")

            self.explorer_active = False

        time.sleep(1)

        if not self.approaching:
            # Triggering approach service
            approach_req = Trigger.Request()

            future = self.approach_object_client.call_async(approach_req)
            future.add_done_callback(self.approach_trigger_response)
            self.approaching = True

    def execute_pickup(self):

        # ================= TEMPORARY MANUAL CONTROL MODE =================
        # Rover does nothing in PICKUP state so you can manually control the arm

        #if not self.arm_active:
         #   self.get_logger().info("PICKUP state: manual arm control enabled. Rover is idle.")
          #  self.arm_active = True  # prevent repeated logging

        #return

        if not self.manipulator_trigger_client.service_is_ready():
            self.get_logger().warn("Manipulator service not ready yet")
            return
            
        if not self.arm_active:
            # Triggering manipulator service
            manipulator_trigger_req = Trigger.Request()

            future = self.manipulator_trigger_client.call_async(manipulator_trigger_req)
            future.add_done_callback(self.manipulator_trigger_response)
            self.arm_active = True

    def execute_backup(self):

        #if not self.backing_up:
        #    self.get_logger().info("Idle")
        #    self.backing_up = True

         #   return 

        #"""
        if not self.backing_up:
            self.get_logger().info("Backing up rover")
            backup_req = Trigger.Request()

            future = self.backup_client.call_async(backup_req)
            future.add_done_callback(self.backup_trigger_response)
            self.backing_up = True
        #"""

    def execute_return(self):

        if not self.move_to_spawn_client.service_is_ready():
            self.get_logger().warn("Move to spawn service not ready yet")
            return

        if not self.returning:
            self.get_logger().info("RETURNING TO START")
            # Triggering move to spawn service
            move_to_spawn_req = Trigger.Request()

            future = self.move_to_spawn_client.call_async(move_to_spawn_req)
            future.add_done_callback(self.move_to_spawn_trigger_response)
            self.returning = True

    #################### SERVICE RESPONSES ####################

    def exploration_response(self,future):
        try: 
            explore_response = future.result()

            if explore_response.success:
                self.get_logger().info(explore_response.message)
            else:
                self.get_logger().info(explore_response.message)

        except Exception as e:
            self.get_logger().error(f"Explorer service call failed: {e}")

    def approach_trigger_response(self,future):
        try:
            approach_response = future.result()

            if approach_response.success:
                self.get_logger().info(approach_response.message)
            else:
                self.get_logger().info(approach_response.message)
        
        except Exception as e:
            self.get_logger().error(f"Approach service call failed: {e}")

    def backup_trigger_response(self,future):
        try:
            backup_response = future.result()

            if backup_response.success:
                self.get_logger().info(backup_response.message)
            else:
                self.get_logger().info(backup_response.message)
        
        except Exception as e:
            self.get_logger().error(f"Approach service call failed: {e}")

    def manipulator_trigger_response(self,future):
        try:
            manipulator_response = future.result()

            if manipulator_response.success:
                self.get_logger().info(manipulator_response.message)
            else:
                self.get_logger().info(manipulator_response.message)
        
        except Exception as e:
            self.get_logger().error(f"Manipulator service call failed: {e}")

    def move_to_spawn_trigger_response(self,future):
        try:
            move_to_spawn_response = future.result()

            if move_to_spawn_response.success:
                self.get_logger().info(move_to_spawn_response.message)
            else:
                self.get_logger().info(move_to_spawn_response.message)
        
        except Exception as e:
            self.get_logger().error(f"Move to spawn service call failed: {e}")

    def publish_slot(self):
        msg = Int32()
        msg.data = self.slot_number
        self.slot_pub.publish(msg)


    #################### CALLBACK FUNCTIONS ####################

    # Function to change state 
    def switch_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f"State change: {self.state} -> {new_state}")
            self.state = new_state

    # Callback function for triggering mission 
    def trigger_mission_callback(self,request,response):

        """ 
        This actives the task manager.
        """        

        # No action if mission is already running
        if self.mission_active:
            response.success = False
            response.message = "Mission already active"
            self.get_logger().warn(response.message)

            return response
        
        # Enable mission if not active
        self.mission_active = True

        # Recording start time
        self.start_time = time.time()

        # Switch state 
        self.switch_state("EXPLORE")

        response.success = True
        response.message = "Mission started"
        self.get_logger().info(response.message)

        return response

    def reached_object_callback(self,msg):
        
        if msg.data == True:
            self.get_logger().info("Rover has reached the object")
            self.switch_state("PICKUP")
            self.approaching = False

        else:
            self.get_logger().info("Approach object service failed, backing up and returning to explore")
            self.switch_state("EXPLORE")
            self.approaching = False
    
    def callback_detected(self,msg):
        if self.state == "EXPLORE":
            if msg.data:
                self.get_logger().info("Object detected! Stopping explorer and requesting approach")
                self.switch_state("APPROACH")

    def backup_success_callback(self,msg):
        self.backing_up = False
        if msg.data:
            self.get_logger().info("Rover backed up to sufficient distance from object pedestal")
            if self.object_count == 3:
                self.get_logger().info(f"Rover collected all {self.object_count} objects, returning to start.")
                self.switch_state("RETURN_TO_START")
            else:
                self.get_logger().info("Continuing exploration")
                self.switch_state("EXPLORE")

    def manipulator_success_callback(self,msg):
        if msg.data == True:
            self.get_logger().info("Object successfully stored")

            # Updating number of objects stored
            self.object_count = self.object_count + 1
            self.slot_number = self.slot_number + 1
            self.get_logger().info(f"Picked object, number of objects stored: {self.object_count}")
            
            # COMMENT OUT AFTER MANUAL CONTROL OF ARM
            #self.get_logger().info("TEMP MANUAL ARM MODE: Task manager paused after manipulator success.")
            #self.switch_state("IDLE")
            #self.mission_active = False
            #self.arm_active = False
            #return

            # Switching state to backup 
            self.switch_state("BACKING_UP")
            self.arm_active = False

        else:
            # Setting arm active variable to False so that manipulator service is triggered again in pickup_execute function
            self.get_logger().info("Manipulator service has failed. Attempting again.")
            self.arm_active = False

    def reached_spawn_callback(self,msg):
        if msg.data == True:
            self.get_logger().info("Successfully returned to spawn")

            self.end_time = time.time()
            duration = (self.end_time - self.start_time)/60
            self.get_logger().info(f"Time till all objects collected and returned to spawn: {duration} mins")

            # Switching state to idle 
            self.switch_state("IDLE")
            self.returning = False

        else:
            # Setting returning variable to False so that move to spawn service is triggered again in execute_return function
            self.get_logger().info("Move to spawn service has failed. Attempting again.")
            self.returning = False





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
