#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
import time

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        self.file_path = '/tmp/leader_path.csv'
        self.f = open(self.file_path, 'w')
        self.writer = csv.writer(self.f)
        
        # FIX: The player wants specific units in the header
        # Previous error wanted 't_sec', this error wants 'x_m'
        self.writer.writerow(['t_sec', 'x_m', 'y_m', 'theta_rad'])
        
        self.start_time = None
        self.get_logger().info(f'Recording path to {self.file_path}...')

    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
            
        current_time = self.get_clock().now().nanoseconds
        elapsed_sec = (current_time - self.start_time) / 1e9
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.writer.writerow([elapsed_sec, x, y, theta])

    def destroy_node(self):
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = PathRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
