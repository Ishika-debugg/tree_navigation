#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import yaml
import os

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(2.0, self.send_waypoints)
        self.sent = False

    def send_waypoints(self):
        if self.sent or not self.client.wait_for_server(timeout_sec=5.0):
            return

        yaml_file = os.path.join(
            os.path.expanduser('~'),  # or specify absolute path
            'ros2_ws', 'src', 'tree_navigation', 'config', 'waypoints.yaml'
        )

        waypoints = self.load_waypoints_from_yaml(yaml_file)

        for i, pose in enumerate(waypoints):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            self.get_logger().info(f'Sending waypoint {i + 1}')
            future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info(f'Waypoint {i + 1} completed')

        self.sent = True

    def load_waypoints_from_yaml(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            return [
                self.create_pose(wp['x'], wp['y'], wp['theta'])
                for wp in data['waypoints']
            ]
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    def create_pose(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
