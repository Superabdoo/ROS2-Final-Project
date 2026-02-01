#!/usr/bin/env python3
# ============================================================================
# Name        : turtlesim_player.py
# Author      : Edges For Training
# ============================================================================

import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# ------------------- Helper Function -------------------
def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class TurtleCSVPathPlayer(Node):
    def __init__(self):
        super().__init__('turtlesim_csv_path_player')

        # ------------------- Parameters -------------------
        self.declare_parameter('csv_file', '/tmp/leader_path.csv')
        self.declare_parameter('skip_first_sec', 5.0)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('offset_x', 5.5)
        self.declare_parameter('offset_y', 5.5)

        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_lin', 2.0)
        self.declare_parameter('max_ang', 4.0)
        self.declare_parameter('kp_lin', 1.2)
        self.declare_parameter('kp_ang', 5.0)
        self.declare_parameter('rate_hz', 30.0)

        # Read parameters
        self.csv_file = self.get_parameter('csv_file').value
        self.skip_first_sec = float(self.get_parameter('skip_first_sec').value)
        self.scale = float(self.get_parameter('scale').value)
        self.offset_x = float(self.get_parameter('offset_x').value)
        self.offset_y = float(self.get_parameter('offset_y').value)

        self.tol = float(self.get_parameter('goal_tolerance').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.kp_lin = float(self.get_parameter('kp_lin').value)
        self.kp_ang = float(self.get_parameter('kp_ang').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # ------------------- Internal State -------------------
        self.pose = None          # current turtle pose
        self.idx = 0              # index of current target point

        # ============================================================
        # COMPLETED SECTION
        # ============================================================

        # 1) Create a publisher to send velocity commands to turtlesim
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10
        )

        # 2) Create a subscriber to read turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_cb,
            10
        )

        # 3) Load Path from CSV
        self.path = self.load_csv_path(self.csv_file, self.skip_first_sec)
        if not self.path:
            self.get_logger().error("No path points loaded. Check csv_file or skip_first_sec.")
            raise SystemExit

        self.get_logger().info(
            f"[Turtlesim Player] Loaded {len(self.path)} points after {self.skip_first_sec} seconds"
        )

        # 4) Create a timer that periodically calls self.tick()
        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.tick)

        # ============================================================

    # ------------------- CSV Loader -------------------
    def load_csv_path(self, filename, skip_sec):
        pts = []
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    t = float(row['t_sec'])
                    if t < skip_sec:
                        continue

                    x = float(row['x_m'])
                    y = float(row['y_m'])

                    # Convert robot coordinates to turtlesim coordinates
                    tx = self.offset_x + self.scale * x
                    ty = self.offset_y + self.scale * y
                    pts.append((tx, ty))
        except FileNotFoundError:
            self.get_logger().error(f"Could not open file: {filename}")
            return []
            
        return pts

    # ------------------- Pose Callback -------------------
    def pose_cb(self, msg: Pose):
        self.pose = msg

    # ------------------- Control Loop -------------------
    def tick(self):
        if self.pose is None:
            return

        if self.idx >= len(self.path):
            self.cmd_pub.publish(Twist()) # Stop
            return

        gx, gy = self.path[self.idx]
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        dist = math.hypot(dx, dy)

        if dist < self.tol:
            self.idx += 1
            return

        target_heading = math.atan2(dy, dx)
        heading_err = wrap_to_pi(target_heading - self.pose.theta)

        lin = min(self.max_lin, self.kp_lin * dist)
        ang = max(-self.max_ang, min(self.max_ang, self.kp_ang * heading_err))

        # Slow down if turning sharply
        if abs(heading_err) > 0.7:
            lin *= 0.3

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = TurtleCSVPathPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.cmd_pub.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
