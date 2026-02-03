import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # Face forward

        self._action_client.wait_for_server()
        print(f"Sending goal: x={x}, y={y}...")
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        print('Goal reached! (or failed)')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    
    # CHANGE THESE NUMBERS to where you want the robot to go
    navigator.send_goal(2.0, 0.0) 
    
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
