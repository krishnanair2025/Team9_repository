import rclpy
from rclpy.node import Node
import numpy as np
import time
import json
from pymycobot import MyCobot, PI_PORT, PI_BAUD
from pymycobot.mycobot280 import MyCobot280
# from rover_interface.msg import ColorPoint
from std_msgs.msg import String


class ArmRunner(Node):

    def __init__(self):
        super().__init__('arm_runner_node')

        print("PI_PORT:", PI_PORT)
        print("PI_BAUD:", PI_BAUD)

        self.mc = MyCobot(PI_PORT, PI_BAUD)
        time.sleep(1)

        self.mc.power_on()
        time.sleep(0.5)

        self.get_logger().info("Robot powered on")

        # Home
        self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        self.mc.set_gripper_value(100, 50)
        time.sleep(3)

        self.obj_coords_subscriber = self.create_subscription(
            msg_type=String,
            topic='/cylinder_target',
            callback=self.pick_and_place,
            qos_profile=1)
            
        self.obj_coords = np.array([0, 0, 0, 0, 0, 0])
        
    #funcs
    
    def open_gripper(self):
        self.mc.set_gripper_value(100, 50)
        time.sleep(1)

    def close_gripper(self):
        self.mc.set_gripper_value(0, 50)
        time.sleep(1)

    def move_to(self, coords):
        # Normal motion (not necessarily straight)
        self.mc.send_coords(list(coords), SPEED_SLOW)
        time.sleep(4)

    def move_up(self, coords):
        # Pure vertical lift (straight line in Z)
        lifted = coords.copy()
        lifted[2] += LIFT
        self.mc.send_coords(list(lifted), SPEED_SLOW, 1)
        time.sleep(3)
        return lifted

    def rotate_base_to(self, coords):
        x, y = coords[0], coords[1]
        angle = np.degrees(np.arctan2(y, x))

        angles = self.mc.get_angles()
        angles[0] = angle

        self.mc.send_angles(angles, SPEED_FAST)
        time.sleep(3)

        
        
    def pick_and_place(self, msg: String):
        print("\n--- PICK & PLACE ---")
        bin_coords = np.array([-200, 0, 160, 180, 0, -45])
        
        data = json.loads(msg.data)
        obj_class = data["class"]
        x = data["x"]
        y = data["y"]
        z = data["z"]
        print(f"Received positions (meters): {[x, y, z]}")
    
        self.obj_coords[0] = x * 1000
        self.obj_coords[1] = y * 1000
        self.obj_coords[2] = z * 1000
        self.obj_coords[3] = 180
        self.obj_coords[4] = 0
        self.obj_coords[5] = 0
        color = obj_class[:-9]
        print(f"Cylinder position: {self.obj_coords[0]}")
        print(f"Cylinder position: {self.obj_coords[1]}")
        print(f"Cylinder position: {self.obj_coords[2]}")
        obj_coords0 = self.obj_coords
        print(f"Cylinder position assigned: {obj_coords0}")
        obj_coords0 = [295, 5, 60, 180, 0, -45]

        self.mc.set_gripper_value(100, 50)
        self.mc.send_angles([0,0,0,0,0,0],20)
        time.sleep(6)

        # 1. Move to object
        self.mc.send_coords(list(self.obj_coords), 15)
        time.sleep(4)

        # 2. Close gripper
        self.mc.set_gripper_value(20, 50)
        time.sleep(1)

        # 3. Lift straight up
        # lifted = self.obj_coords.copy()
        # lifted[2] += 110
        # self.mc.send_coords(list(lifted), 5, 1)
        angles = None
        while angles is None:
            angles = self.mc.get_angles()
        angles[1] += 90
        angles[3] -= 90
        print(f"lift angle: {angles}")
        self.mc.send_angles(angles, 30)
        time.sleep(2)

        # 4. Rotate base to bin
        x, y = bin_coords[0], bin_coords[1]
        angle = np.rad2deg(np.arctan2(y, x))
        print(angle)
        angle = np.clip(angle, -168, 168)

        angles = None
        while angles is None:
            angles = self.mc.get_angles()
        print(angles)
        angles[0] = -angle
        self.mc.send_angles(angles, 30)
        time.sleep(5)
        
        # 5. Move to above bin (KEEP SAME HEIGHT)
        goal_above = bin_coords.copy()
        goal_above[2] += 70
        self.mc.send_coords(list(goal_above), 15)
        time.sleep(3)

        # 6. Move to bin (KEEP SAME HEIGHT)
        self.mc.send_coords(list(bin_coords), 15, 1)
        time.sleep(7)

        # 6. Open gripper
        self.mc.set_gripper_value(100, 50)
        time.sleep(3)



def main(args=None):
    rclpy.init(args=args)
    node = ArmRunner()
    
    # ---- Fixed Coordinates ----
    # object_coords = np.array([-200, 0, 156, 180, 0, -45])
    # bin_coords = np.array([250, 0, 160, 180, 0, -45])
    
    rclpy.spin(node)  # will exit after shutdown()


if __name__ == '__main__':
    main()
