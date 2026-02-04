#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
import csv
import time
import sys
import os

class PerfectReplay(Node):
    def __init__(self):
        super().__init__('perfect_replay')
        
        # 1. Setup the Teleport Client (The God Mode Command)
        self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TurtleSim...')
            
        # 2. Lift the Pen (So we don't draw the jump to start)
        self.set_pen(255, 255, 255, 3, 1) # Off
        
        # 3. Read the CSV
        csv_path = os.path.expanduser('~/robot_ws/path.csv')
        self.data = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Store: x, y, theta
                self.data.append({
                    'x': float(row['x_m']),
                    'y': float(row['y_m']),
                    'theta': float(row['theta_rad'])
                })
        
        self.get_logger().info(f'Loaded {len(self.data)} points. Starting Replay...')
        self.run_replay()

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r; req.g = g; req.b = b; req.width = width; req.off = off
        self.pen_cli.call_async(req)

    def run_replay(self):
        # Center Offset: Turtle starts at (5.5, 5.5). Robot starts at (0,0)
        # We must shift the robot data so it draws in the middle of the screen
        OFFSET_X = 5.544445
        OFFSET_Y = 5.544445

        # Teleport to Start
        start = self.data[0]
        self.teleport(start['x'] + OFFSET_X, start['y'] + OFFSET_Y, start['theta'])
        time.sleep(5.0)
        
        # Pen Down (Start Drawing)
        self.set_pen(255, 0, 0, 3, 0) # Red Line
        
        # Loop through points
        for i, point in enumerate(self.data):
            # Skip points to speed up drawing (draw every 5th point)
            if i % 5 == 0:
                self.teleport(point['x'] + OFFSET_X, point['y'] + OFFSET_Y, point['theta'])
                # Tiny sleep to make it look like animation
                time.sleep(0.01)
        
        self.get_logger().info('Replay Finished.')
        sys.exit()

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.cli.call_async(req)
        # We don't wait for result to keep it fast

def main(args=None):
    rclpy.init(args=args)
    node = PerfectReplay()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
