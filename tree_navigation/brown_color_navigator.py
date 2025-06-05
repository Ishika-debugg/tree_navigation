#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BrownColorNavigator(Node):
    def __init__(self):
        super().__init__('brown_color_navigator')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'oak/rgb/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tree_status_sub = self.create_subscription(Bool, '/tree_detection_active', self.tree_status_callback, 10)
        self.debug_img_pub = self.create_publisher(Image, '/brown_navigator/debug_image', 10)
        self.brown_hsv_lower = np.array([7, 149, 107])
        self.brown_hsv_upper = np.array([17, 209, 167])
        self.forward_speed = 0.3
        self.turn_speed = 0.2
        self.min_color_area = 500
        self.center_tolerance = 50
        self.tree_detection_active = False

    def tree_status_callback(self, msg):
        self.tree_detection_active = msg.data

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.brown_hsv_lower, self.brown_hsv_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cmd = Twist()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > self.min_color_area:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    center_offset = cx - (image.shape[1] // 2)
                    cmd.linear.x = self.forward_speed
                    if abs(center_offset) > self.center_tolerance:
                        cmd.angular.z = -self.turn_speed if center_offset > 0 else self.turn_speed
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BrownColorNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
