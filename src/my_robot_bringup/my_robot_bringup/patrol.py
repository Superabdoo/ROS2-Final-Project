import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class PatrolRobot(Node):
    def __init__(self):
        super().__init__('patrol_robot')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def go_to_pose(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # Face forward

        self._action_client.wait_for_server()
        self.get_logger().info(f'Navigating to: x={x}, y={y}...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for the robot to get there (Simple synchronous wait)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if send_goal_future.done():
                goal_handle = send_goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().error('Goal rejected :(')
                    return False
                
                result_future = goal_handle.get_result_async()
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if result_future.done():
                        status = result_future.result().status
                        self.get_logger().info(f'Goal Reached! Status: {status}')
                        return True

def main(args=None):
    rclpy.init(args=args)
    patrol = PatrolRobot()
    
    # --- CONFIGURATION ---
    # Set your two patrol points here
    location_A = {'x': 2.0, 'y': 0.0}
    location_B = {'x': -2.0, 'y': 0.0}
    # ---------------------

    while True: # Loop forever
        # Go to A
        patrol.go_to_pose(location_A['x'], location_A['y'])
        time.sleep(2) # Wait 2 seconds
        
        # Go to B
        patrol.go_to_pose(location_B['x'], location_B['y'])
        time.sleep(2) # Wait 2 seconds

if __name__ == '__main__':
    main()
