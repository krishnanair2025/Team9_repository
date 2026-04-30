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
from nav_msgs.msg import Path
import numpy as np
import time

class FrontierExplorationNode(Node):

    def __init__(self):
        super().__init__('frontier_exploration_node')

        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            '/navigate_to_pose')

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.exploration_service = self.create_service(
            srv_type=SetBool,
            srv_name='/explore_status',
            callback=self.exploration_server_callback
        )

        self.spin_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.bcd_path_pub = self.create_publisher(
            Path,
            '/bcd_sweep_path',
            10
        )

        self._current_goal_handle = None 

        self.map_data = None
        self.map_info = None

        self.costmap_data = None
        self.costmap_info = None

        self.exploring = False
        self.spinning = False
        self.goal_active = False

        self.previous_targets = []
        self.min_target_separation = 0.3

        # ---- BCD SWEEP STATE ----
        self.sweep_cells = None
        self.sweep_index = 0

        # Spin variables
        self.spin_timer = None
        self.spin_start_time = None
        self.spin_duration = 30.0
        self.spin_speed = 0.25

        # Costmap update waiting timer
        self.waiting_for_costmap = False
        self.wait_timer = None
        self.wait_duration = 0.5

    # ---------------- SWEEP LOGIC ----------------

    """def generate_bcd_sweep(self):
        if self.map_data is None or self.costmap_data is None:
            return None

        map_free = (self.map_data == 0)

        h, w = self.costmap_data.shape
        res = self.costmap_info.resolution

        robot_w = int(np.ceil(0.448 / res))
        robot_l = int(np.ceil(0.425 / res))

        inflation_threshold = 30

        safe_mask = np.zeros_like(map_free, dtype=bool)

        ys, xs = np.where(map_free)

        for y, x in zip(ys, xs):

            x0 = int(x - robot_w // 2)
            x1 = int(x + robot_w // 2)
            y0 = int(y - robot_l // 2)
            y1 = int(y + robot_l // 2)

            if x0 < 0 or y0 < 0 or x1 >= w or y1 >= h:
                continue

            footprint = self.costmap_data[y0:y1+1, x0:x1+1]

            if np.any(footprint == -1):
                continue

            if np.max(footprint) > inflation_threshold:
                continue

            safe_mask[y, x] = True

        if not np.any(safe_mask):
            self.get_logger().info("No safe sweep cells")
            return None

        row_step = max(int(robot_l), 1)

        sweep = []
        reverse = False

        # ---- FIX 1: evaluate all rows ----
        for y in range(h):

            # ---- FIX 2: enforce spacing manually ----
            #if y % row_step != 0:
            #    continue

            row_x = np.where(safe_mask[y])[0]

            if len(row_x) == 0:
                continue

            segments = np.split(row_x, np.where(np.diff(row_x) != 1)[0] + 1)

            for seg in segments:
                if len(seg) == 0:
                    continue

                # ---- FIX 3: continuous sweep instead of endpoints ----
                if reverse:
                    xs = seg[::-1]
                else:
                    xs = seg

                for x_val in xs:
                    sweep.append((y, x_val))

            reverse = not reverse

        self.sweep_cells = sweep
        self.publish_bcd_path()
        return sweep """
    
    def generate_region_tour(self):
        if self.map_data is None or self.costmap_data is None:
            return None

        map_free = (self.map_data == 0)

        h, w = self.costmap_data.shape
        res = self.costmap_info.resolution

        robot_w = int(np.ceil(0.488 / res))
        robot_l = int(np.ceil(0.455 / res))

        inflation_threshold = 60

        safe_mask = np.zeros_like(map_free, dtype=bool)

        ys, xs = np.where(map_free)

        # ---- STEP 1: build traversible mask ----
        for y, x in zip(ys, xs):

            x0 = int(x - robot_w // 2)
            x1 = int(x + robot_w // 2)
            y0 = int(y - robot_l // 2)
            y1 = int(y + robot_l // 2)

            if x0 < 0 or y0 < 0 or x1 >= w or y1 >= h:
                continue

            footprint = self.costmap_data[y0:y1+1, x0:x1+1]

            if np.any(footprint == -1):
                continue

            # Reject only if the robot footprint overlaps an obstacle cell
            if np.any((footprint >= 100) | (footprint == 255)):
                continue

            safe_mask[y, x] = True

        if not np.any(safe_mask):
            self.get_logger().info("No traversible space")
            return None

        # ---- STEP 2: fixed grid subdivision (6 x 4 = 24) ----
        rows = 4
        cols = 6

        cell_h = h // rows
        cell_w = w // cols

        waypoints = []

        for r in range(rows):
            for c in range(cols):

                y0 = r * cell_h
                y1 = (r + 1) * cell_h if r < rows - 1 else h

                x0 = c * cell_w
                x1 = (c + 1) * cell_w if c < cols - 1 else w

                mask = safe_mask[y0:y1, x0:x1]
                pts = np.argwhere(mask)

                if len(pts) == 0:
                    continue

                # convert to global coords
                pts[:, 0] += y0
                pts[:, 1] += x0

                # centroid of traversible cells in this section
                cy = int(np.mean(pts[:, 0]))
                cx = int(np.mean(pts[:, 1]))

                waypoints.append((cy, cx))

        if len(waypoints) == 0:
            return None

        # ---- STEP 3: order waypoints (nearest neighbour) ----
        ordered = []
        remaining = waypoints.copy()

        current = remaining.pop(0)
        ordered.append(current)

        while remaining:
            dists = [
                (abs(current[0] - y) + abs(current[1] - x), (y, x))
                for (y, x) in remaining
            ]

            _, next_pt = min(dists, key=lambda x: x[0])

            ordered.append(next_pt)
            remaining.remove(next_pt)
            current = next_pt

        # ---- optional plot (ensure non-blocking implementation) ----
        #self.plot_region_plan_grid(safe_mask, ordered, cell_h, cell_w)

        self.sweep_cells = ordered
        self.publish_bcd_path()

        return ordered
    


    def plot_region_plan(self, safe_mask, regions, waypoints, block_size):

        import matplotlib.pyplot as plt
        import numpy as np

        # ---- init once ----
        if not hasattr(self, "plot_initialized"):
            plt.ion()  # interactive mode
            self.fig, self.ax = plt.subplots(figsize=(8, 8))
            self.plot_initialized = True

        self.ax.clear()

        # ---- traversible mask ----
        self.ax.imshow(safe_mask, origin='lower')

        # ---- regions ----
        for region in regions:
            region = np.array(region)
            self.ax.scatter(region[:, 1], region[:, 0], s=1)

        # ---- grid ----
        h, w = safe_mask.shape

        for y in range(0, h, block_size):
            self.ax.axhline(y, linewidth=0.5)

        for x in range(0, w, block_size):
            self.ax.axvline(x, linewidth=0.5)

        # ---- waypoints ----
        if len(waypoints) > 0:
            wp = np.array(waypoints)
            self.ax.scatter(wp[:, 1], wp[:, 0], s=30)
            self.ax.plot(wp[:, 1], wp[:, 0], linewidth=1)

        self.ax.set_title("Traversible Regions, Subdivisions, and Planned Path")
        self.ax.set_xlabel("X (cells)")
        self.ax.set_ylabel("Y (cells)")
        self.ax.invert_yaxis()

        # ---- non-blocking refresh ----
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def publish_bcd_path(self):

        if self.sweep_cells is None or self.map_info is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        res = self.map_info.resolution
        origin = self.map_info.origin.position

        for (y, x) in self.sweep_cells:

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = origin.x + x * res
            pose.pose.position.y = origin.y + y * res
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.bcd_path_pub.publish(path_msg)

    def get_next_sweep_cell(self):
        if self.sweep_cells is None:
            self.generate_region_tour()

        if self.sweep_cells is None:
            self.get_logger().info("No sweep cells available")
            return None

        if self.sweep_index >= len(self.sweep_cells):
            self.get_logger().info("Sweep complete")
            return None

        cell = self.sweep_cells[self.sweep_index]
        self.sweep_index += 1

        self.get_logger().info(f"Sweep index: {self.sweep_index}")

        return cell

    # ---------------- CALLBACKS ----------------

    def exploration_server_callback(self, request, response):

        self.exploring = request.data
        response.success = True

        if self.exploring:
            response.message = "Explorer running"
        else:
            response.message = "Explorer paused"

            if self.spinning:
                self.stop_spin()

            if self._current_goal_handle is not None:
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)

        return response

    def costmap_callback(self, msg):
        self.costmap_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.costmap_data = data.reshape((msg.info.height, msg.info.width))

        # ---- FIX 4: only recompute when idle ----
        if not self.goal_active:
            old_length = 0 if self.sweep_cells is None else len(self.sweep_cells)

            self.generate_region_tour()

            new_length = 0 if self.sweep_cells is None else len(self.sweep_cells)

            if old_length == 0 and new_length > 0:
                self.sweep_index = 0

    def map_callback(self, msg):

        self.map_info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        self.map_data = data.reshape((msg.info.height, msg.info.width))

        if (self.exploring and not self.goal_active and 
            not self.spinning and not self.waiting_for_costmap):

            frontier = self.pick_frontier()

            if frontier is not None:
                pose = self.frontier_to_pose(frontier)
                self.send_goal(pose)
            else:
                cell = self.get_next_sweep_cell()

                if cell is not None:
                    pose = self.frontier_to_pose(cell)
                    self.send_goal(pose)

    # ---------------- FRONTIER ----------------

    def has_sufficient_unknown(self, y, x, unknown, radius=2, min_unknown=3):

        h, w = unknown.shape
        count = 0

        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):

                ny = y + dy
                nx = x + dx

                if ny < 0 or nx < 0 or ny >= h or nx >= w:
                    continue

                if unknown[ny, nx]:
                    count += 1

        return count >= min_unknown

    def pick_frontier(self):
        if self.map_data is None or self.costmap_data is None:
            return None

        free = (self.map_data == 0)
        unknown = (self.map_data == -1)

        padded_unknown = np.pad(unknown, 1, mode='constant')

        frontier_mask = free & (
            padded_unknown[0:-2, 1:-1] |
            padded_unknown[2:, 1:-1] |
            padded_unknown[1:-1, 0:-2] |
            padded_unknown[1:-1, 2:]
        )

        points = np.argwhere(frontier_mask)

        if len(points) == 0:
            return None

        safe_frontiers = []
        h, w = self.costmap_data.shape

        res = self.costmap_info.resolution
        rover_width = int(np.ceil((448/1000) / res))
        rover_length = int(np.ceil((425/1000) / res))

        for y, x in points:

            if not self.has_sufficient_unknown(y, x, unknown):
                continue

            x0 = int(x - rover_width/2)
            x1 = int(x + rover_width/2)
            y0 = int(y - rover_length/2)
            y1 = int(y + rover_length/2)

            if x0 < 0 or y0 < 0 or x1 >= w or y1 >= h:
                continue

            footprint = self.costmap_data[y0:y1+1, x0:x1+1]

            if np.all((footprint < 20) | (footprint == 255)):
                safe_frontiers.append((y, x))

        if len(safe_frontiers) == 0:
            return None

        return safe_frontiers[np.random.choice(len(safe_frontiers))]

    def frontier_to_pose(self, cell):

        y, x = cell
        res = self.map_info.resolution
        origin = self.map_info.origin.position

        wx = origin.x + x * res
        wy = origin.y + y * res

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.w = 1.0

        return pose

    def send_goal(self, pose):

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

        self.goal_active = True

    def goal_response_callback(self, future):

        handle = future.result()
        self._current_goal_handle = handle

        if not handle.accepted:
            self.goal_active = False
            return

        handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        status = future.result().status

        if status == 4:
            self.goal_active = False

            self.waiting_for_costmap = True

            self.wait_timer = self.create_timer(
                self.wait_duration,
                self.costmap_wait_done
            )

        elif status == 6:
            self.goal_active = False

    def cancel_done_callback(self, future):
        self._current_goal_handle = None
        self.goal_active = False

    def costmap_wait_done(self):

        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

        self.waiting_for_costmap = False

    # ---------------- SPIN ----------------

    def start_spin(self, angular_speed=0.25, duration=30.0):

        self.spinning = True
        self.spin_speed = angular_speed
        self.spin_duration = duration
        self.spin_start_time = self.get_clock().now().nanoseconds / 1e9

        self.spin_timer = self.create_timer(0.05, self.spin_callback)

    def spin_callback(self):

        if not self.exploring:
            self.stop_spin()
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if (now - self.spin_start_time) >= self.spin_duration:
            self.stop_spin()
            return

        msg = Twist()
        msg.angular.z = self.spin_speed
        self.spin_pub.publish(msg)

    def stop_spin(self):

        msg = Twist()
        self.spin_pub.publish(msg)

        if self.spin_timer is not None:
            self.spin_timer.cancel()
            self.spin_timer = None

        self.spinning = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()