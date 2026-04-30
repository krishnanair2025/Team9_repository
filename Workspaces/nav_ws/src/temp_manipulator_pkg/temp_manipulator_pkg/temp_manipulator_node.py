#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from example_interfaces.srv import Trigger
import random

class ManipulatorTriggerNode(Node):
    def __init__(self):
        super().__init__('manipulator_trigger_node')

        # Publisher for completion signal
        self.pub_done = self.create_publisher(Bool, '/manipulator_success', 10)

        # Service to trigger the manipulator timer
        self.srv = self.create_service(Trigger, '/trigger_arm', self.trigger_callback)

        self.timer = None  # store timer reference

        self.get_logger().info("Manipulator trigger node ready.")

    def trigger_callback(self, request, response):
        # Generate random duration between 30 and 60 seconds
        duration = random.uniform(1.0, 5.0)
        self.get_logger().info(f"Trigger received. Timer set for {duration:.2f} seconds.")

        # Cancel existing timer if running
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)

        # Create timer
        self.timer = self.create_timer(duration, self._timer_wrapper)

        response.success = True
        response.message = f"Manipulator timer started for {duration:.2f} seconds."
        return response

    def _timer_wrapper(self):
        # Run once, then cancel
        self.timer_finished()

        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

    def timer_finished(self):
        msg = Bool()
        msg.data = True
        self.pub_done.publish(msg)
        self.get_logger().info("Manipulator action complete. Published True.")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ManipulatorTriggerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()