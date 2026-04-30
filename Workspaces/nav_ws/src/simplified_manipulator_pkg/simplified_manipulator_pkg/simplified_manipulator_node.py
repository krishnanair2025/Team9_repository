#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Int32
from example_interfaces.srv import Trigger
import numpy as np

class JointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_server')

        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        self.done_pub = self.create_publisher(Bool, '/trajectory_done', 10)
        self.success_pub = self.create_publisher(Bool, '/manipulator_success', 10)

        # Subscriber for slot selection
        self.slot_sub = self.create_subscription(
            Int32,
            '/slot',
            self.slot_callback,
            10
        )

        self.selected_slot = 1  # default

        self.srv = self.create_service(
            Trigger,
            'trigger_arm',
            self.trigger_callback
        )

        # Waypoints (FILLED EXACTLY)
        self.waypoints_slot1 = np.array([
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0], #Approach
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.0], #Align so that cylinder in the middle of gripper
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.022], # Close gripper, grasp cylinder
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022], # Move up with cylinder grasped to default position
            [-0.35,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022], # Rotate backwards to face storage box
            [-0.35,0.125,-1.57,0.785398,-1.570796,0.0,0.0,0.022], # Align with target slot for cylinder in gripper
            [-0.35,0.2,-1.57,-0.0,-1.570796,0.0,0.0,0.022], # Refine alignment with slot
            [-0.35,0.2,-1.57,-0.0,-1.570796,0.0,0.0,0.0], #Open gripper, drop cylinder into slot
            [-0.35,0.125,-1.57,0.785398,-1.570796,0.0,0.0,0.0],
            [-0.35,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0],
            [0.0,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0]
        ])

        self.waypoints_slot2 = np.array([
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0],
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.0],
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.022],
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022],
            [-0.35,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022],
            [-0.35,0.125,-1.57,0.785398,-1.570796,0.0,0.0,0.022],
            [-0.10,-0.15,-1.57,0.55,-1.57,0.0,0.0,0.022],
            [-0.10,-0.15,-1.57,0.55,-1.57,0.0,0.0,0.0],
            [0.0,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0]
        ])

        self.waypoints_slot3 = np.array([
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0],
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.0],
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.022],
            [0.12,1.57,0.2,-0.55,1.570796,0.0,0.0,0.022],
            [0.12,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022],
            [-0.35,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.022],
            [-0.35,0.125,-1.57,0.785398,-1.570796,0.0,0.0,0.022],
            [-0.4,-0.15,-1.57,0.55,-1.57,0.0,0.0,0.022],
            [-0.4,-0.15,-1.57,0.55,-1.57,0.0,0.0,0.0],
            [0.0,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0]
        ])

        self.waypoints = self.waypoints_slot1

        self.freq = 50.0
        self.duration = 10.0
        self.timer = None
        self.one_shot_timer = None
        self.success_timer = None
        self.current_traj = None
        self.start_time = None

        self.current_positions = np.zeros(self.waypoints.shape[1])

        self.fixed_start_pose = np.array(
            [0.0,-0.785398,0.785398,0.785398,1.570796,0.0,0.0,0.0]
        )

        self.current_waypoint = 0

    def slot_callback(self, msg):
        self.selected_slot = msg.data

    def trigger_callback(self, request, response):
        if self.selected_slot == 1:
            self.waypoints = self.waypoints_slot1
        elif self.selected_slot == 2:
            self.waypoints = self.waypoints_slot2
        elif self.selected_slot == 3:
            self.waypoints = self.waypoints_slot3
        else:
            response.success = False
            response.message = 'Invalid slot'
            return response

        self.current_waypoint = 0
        self.current_positions = self.fixed_start_pose.copy()

        target = self.waypoints[self.current_waypoint]

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_traj = self.generate_trapezoidal_trajectory(
            self.fixed_start_pose,
            target,
            self.duration,
            self.freq
        )

        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

        response.success = True
        response.message = f'Started trajectory for slot {self.selected_slot}'
        return response

    def generate_trapezoidal_trajectory(self, start, end, duration, freq):
        n_steps = int(duration * freq)
        traj = np.zeros((n_steps, len(start)))
        t = np.linspace(0.0, 1.0, n_steps)
        smooth_t = 3.0 * t**2 - 2.0 * t**3

        for i in range(len(start)):
            traj[:, i] = start[i] + (end[i] - start[i]) * smooth_t

        return traj

    def timer_callback(self):
        if self.current_traj is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        step = int((now - self.start_time) * self.freq)

        if step >= len(self.current_traj):
            msg = Float64MultiArray()
            msg.data = self.waypoints[self.current_waypoint].tolist()
            self.joint_pub.publish(msg)

            self.current_positions = self.waypoints[self.current_waypoint].copy()

            if self.timer:
                self.timer.cancel()
                self.timer = None

            self.current_waypoint += 1

            if self.current_waypoint < len(self.waypoints):
                self.one_shot_timer = self.create_timer(
                    10.0,
                    self.start_next_waypoint_once
                )
            else:
                self.success_timer = self.create_timer(
                    self.duration,
                    self.publish_success_once
                )
            return

        msg = Float64MultiArray()
        msg.data = self.current_traj[step].tolist()
        self.joint_pub.publish(msg)

        self.current_positions = self.current_traj[step]

    def start_next_waypoint_once(self):
        if self.one_shot_timer:
            self.one_shot_timer.cancel()
            self.one_shot_timer = None

        target = self.waypoints[self.current_waypoint]

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_traj = self.generate_trapezoidal_trajectory(
            self.current_positions,
            target,
            self.duration,
            self.freq
        )

        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

    def publish_success_once(self):
        if self.success_timer:
            self.success_timer.cancel()
            self.success_timer = None

        msg = Bool()
        msg.data = True
        self.success_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = JointTrajectoryServer()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
