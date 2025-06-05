#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class TreeNavigationCoordinator(Node):
    def __init__(self):
        super().__init__('tree_navigation_coordinator')
        self.tree_status_sub = self.create_subscription(Bool, '/tree_detection_active', self.tree_status_callback, 10)
        self.nav_status_sub = self.create_subscription(Bool, '/navigation_active', self.nav_status_callback, 10)
        self.detected_trees_sub = self.create_subscription(String, '/detected_trees', self.detected_trees_callback, 10)
        self.coordination_status_pub = self.create_publisher(String, '/coordination_status', 10)
        self.tree_detection_active = False
        self.navigation_active = False
        self.detected_tree_count = 0
        self.timer = self.create_timer(2.0, self.publish_coordination_status)

    def tree_status_callback(self, msg):
        self.tree_detection_active = msg.data

    def nav_status_callback(self, msg):
        self.navigation_active = msg.data

    def detected_trees_callback(self, msg):
        if msg.data.startswith("trees_detected:"):
            self.detected_tree_count = int(msg.data.split(":")[1])

    def publish_coordination_status(self):
        status = f"Trees: {self.detected_tree_count}, Detection: {self.tree_detection_active}, Navigation: {self.navigation_active}"
        self.coordination_status_pub.publish(String(data=status))

def main(args=None):
    rclpy.init(args=args)
    node = TreeNavigationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
