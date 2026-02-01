#!/usr/bin/env python3
# ============================================================================
# Name        : path_to_csv_recorder.py
# Author      : Edges For Training
# ============================================================================

import math
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


# ------------------- Helper Function -------------------
def quat_to_yaw(x, y, z, w):
    """
    Convert quaternion orientation to yaw angle (rotation around Z axis).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathToCSVRecorder(Node):
    def __init__(self):
        super().__init__('path_to_csv_recorder')

        # ------------------- Parameters -------------------
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('csv_file', '/tmp/leader_path.csv')
        self.declare_parameter('sample_hz', 10.0)
        self.declare_parameter('min_dist', 0.02)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.csv_file = self.get_parameter('csv_file').value
        self.sample_hz = float(self.get_parameter('sample_hz').value)
        self.min_dist = float(self.get_parameter('min_dist').value)

        # ------------------- State Variables -------------------
        self.start_time = self.get_clock().now()
        self.latest_pose = None
        self.last_x = None
        self.last_y = None

        # ------------------- CSV File Setup -------------------
        self.f = open(self.csv_file, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['t_sec', 'x_m', 'y_m', 'yaw_rad'])
        self.f.flush()

        # ============================================================
        # COMPLETED SECTION
        # ============================================================
        
        # 1) Create a subscription to read Odometry from self.odom_topic
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10  # QoS depth
        )

        # 2) Create a timer that calls self.timer_cb at (1.0 / sample_hz)
        timer_period = 1.0 / self.sample_hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

        # ============================================================

        self.get_logger().info(f"[Recorder] Ready. Will record from: {self.odom_topic}")
        self.get_logger().info(f"[Recorder] Output CSV: {self.csv_file}")

    # ------------------- Subscriber Callback -------------------
    def odom_cb(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    # ------------------- Timer Callback -------------------
    def timer_cb(self):
        if self.latest_pose is None:
            return

        # Extract x, y position
        x = float(self.latest_pose.position.x)
        y = float(self.latest_pose.position.y)

        # Extract yaw from quaternion
        q = self.latest_pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Only save if moved enough
        if self.last_x is not None:
            if math.hypot(x - self.last_x, y - self.last_y) < self.min_dist:
                return

        # Time since start (seconds)
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Save a row in CSV
        self.writer.writerow([round(t, 4), round(x, 4), round(y, 4), round(yaw, 4)])
        self.f.flush()

        # Update last stored position
        self.last_x, self.last_y = x, y

    # ------------------- Cleanup -------------------
    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PathToCSVRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
