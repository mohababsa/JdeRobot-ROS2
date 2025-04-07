import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = [
            (1.0, 0.0, 0.0),   # x, y, yaw
            (1.0, 1.0, 1.57),
            (-1.0, 1.0, 3.14),
        ]
        self.current_goal = 0

    def send_goal(self):
        if self.current_goal >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.waypoints[self.current_goal][0]
        pose.pose.position.y = self.waypoints[self.current_goal][1]
        pose.pose.orientation.z = self.waypoints[self.current_goal][2]
        goal_msg.pose = pose

        self.get_logger().info(f"Sending waypoint {self.current_goal + 1}: {self.waypoints[self.current_goal]}")
        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal {self.current_goal + 1} reached!")
        self.current_goal += 1
        self.send_goal()

    def feedback_callback(self, feedback_msg):
        pass  # Optional: log progress

def main():
    rclpy.init()
    navigator = WaypointNavigator()
    navigator.send_goal()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
