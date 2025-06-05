#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs

class TreeDetector(Node):
    def __init__(self):
        super().__init__('tree_detector')
        self.bridge = CvBridge()
        self.scan = None
        self.image_sub = self.create_subscription(Image, 'oak/rgb/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.tree_pub = self.create_publisher(String, '/detected_trees', 10)
        self.tree_status_pub = self.create_publisher(Bool, '/tree_detection_active', 10)
        self.marker_pub = self.create_publisher(Marker, '/tree_markers', 10)
        self.debug_img_pub = self.create_publisher(Image, '/tree_detection/debug_image', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.brown_hsv_lower = np.array([7, 149, 107])
        self.brown_hsv_upper = np.array([17, 209, 167])
        self.min_area = 500
        self.detection_count = 0
        self.frame_count = 0
        self.last_detection_frame = 0
        self.last_no_detection_log = 0
        self.detected_trees_positions = []
        self.get_logger().info('Tree Detector initialized.')

    def scan_callback(self, msg):
        self.scan = msg

    def image_callback(self, msg):
        if self.scan is None:
            return
        try:
            self.frame_count += 1
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            trees = self.detect_all_trees_fast(cv_image)
            self.tree_status_pub.publish(Bool(data=len(trees) > 0))
            if trees:
                self.handle_trees_detected(cv_image, trees)
            else:
                self.handle_no_trees_detected()
            debug_img = self.create_debug_image(cv_image, trees)
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f'Error in tree detection: {str(e)}')

    def detect_all_trees_fast(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.brown_hsv_lower, self.brown_hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return [(x, y, w, h, cv2.contourArea(c)) for c in contours if (cv2.contourArea(c) > self.min_area) for x, y, w, h in [cv2.boundingRect(c)]]

    def handle_trees_detected(self, cv_image, trees):
        self.tree_pub.publish(String(data=f"trees_detected:{len(trees)}"))
        if self.frame_count - self.last_detection_frame > 20:
            self.detection_count += 1
            self.get_logger().info(f'Detected {len(trees)} tree(s)')
            self.last_detection_frame = self.frame_count

    def handle_no_trees_detected(self):
        if self.frame_count - self.last_no_detection_log > 60:
            self.get_logger().info('No trees detected')
            self.last_no_detection_log = self.frame_count

    def create_debug_image(self, image, trees):
        img = image.copy()
        for i, (x, y, w, h, _) in enumerate(trees):
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(img, f'TREE {i+1}', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return img

def main(args=None):
    rclpy.init(args=args)
    node = TreeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
