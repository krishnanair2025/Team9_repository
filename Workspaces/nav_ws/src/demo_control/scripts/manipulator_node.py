#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from std_srvs.srv import Trigger
import numpy as np

class JointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory_server')

        # Publisher for joint positions
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        # Publisher for completion flag
        self.done_pub = self.create_publisher(Bool, '/trajectory_done', 10)

        # NEW: Publisher for overall success
        self.success_pub = self.create_publisher(Bool, '/manipulator_success', 10)

        # Service to trigger motion
        self.srv = self.create_service(
            Trigger,
            'trigger_arm',
            self.trigger_callback
        )

        # Waypoints (multi-row array, 8 joints each)
        self.waypoints = np.array([
            [0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.15, 1.57,  0.174, -0.88, 1.57, 0.0, 0.0, 0.0],
            [0.15, 1.57,  0.174, -0.44, 1.57, 0.0, 0.0, 0.0],
            [0.15, 1.57,  0.174, -0.44, 1.57, 0.0, 0.0, 0.019]
        ])

        # Trajectory parameters
        self.freq = 50.0  # Hz
        self.duration = 2.0  # seconds per waypoint
        self.timer = None
        self.one_shot_timer = None
        self.current_traj = None
        self.start_time = None

        # Current joint state
        self.current_positions = np.zeros(self.waypoints.shape[1])

        # Waypoint index
        self.current_waypoint = 0

        self.get_logger().info('Joint trajectory server ready.')

    def trigger_callback(self, request, response):
        if self.current_waypoint >= len(self.waypoints):
            response.success = False
            response.message = 'All waypoints completed'
            return response

        # Start trajectory to current waypoint
        target = self.waypoints[self.current_waypoint]
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_traj = self.generate_trapezoidal_trajectory(
            self.current_positions,
            target,
            self.duration,
            self.freq
        )
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

        response.success = True
        response.message = f'Trajectory to waypoint {self.current_waypoint} started'
        return response

    def generate_trapezoidal_trajectory(self, start, end, duration, freq):
        n_steps = int(duration * freq)
        traj = np.zeros((n_steps, len(start)))
        t = np.linspace(0.0, 1.0, n_steps)
        smooth_t = 3.0 * t**2 - 2.0 * t**3  # cubic ease-in-out

        for i in range(len(start)):
            traj[:, i] = start[i] + (end[i] - start[i]) * smooth_t

        return traj

    def timer_callback(self):
        if self.current_traj is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        step = int((now - self.start_time) * self.freq)

        if step >= len(self.current_traj):
            # Publish final position of this waypoint
            msg = Float64MultiArray()
            msg.data = self.waypoints[self.current_waypoint].tolist()
            self.joint_pub.publish(msg)

            # Update current joint positions
            self.current_positions = self.waypoints[self.current_waypoint].copy()

            # Publish True only if this is the last waypoint
            done_msg = Bool()
            done_msg.data = (self.current_waypoint >= len(self.waypoints) - 1)
            self.done_pub.publish(done_msg)

            # Stop main trajectory timer
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None

            # Move to next waypoint if any
            self.current_waypoint += 1
            if self.current_waypoint < len(self.waypoints):
                self.get_logger().info(
                    f'Waiting 5s before starting waypoint {self.current_waypoint}'
                )
                self.one_shot_timer = self.create_timer(
                    5.0,
                    self.start_next_waypoint_once
                )
            else:
                self.get_logger().info('All waypoints completed.')

                # NEW: Publish manipulator success
                success_msg = Bool()
                success_msg.data = True
                self.success_pub.publish(success_msg)

            return

        # Publish current step
        msg = Float64MultiArray()
        msg.data = self.current_traj[step].tolist()
        self.joint_pub.publish(msg)

        # Update current positions
        self.current_positions = self.current_traj[step]

    def start_next_waypoint_once(self):
        if self.one_shot_timer is not None:
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
        self.get_logger().info(
            f'Starting trajectory to waypoint {self.current_waypoint}'
        )


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