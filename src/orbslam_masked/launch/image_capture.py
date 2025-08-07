#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class QuickCapture(Node):
    def __init__(self):
        super().__init__('quick_capture')
        self.bridge = CvBridge()
        self.left_saved = False
        self.right_saved = False
        
        # Subscribe to both cameras
        self.left_sub = self.create_subscription(Image, '/zed/left/image_rect_color', self.save_left, 10)
        self.right_sub = self.create_subscription(Image, '/zed/right/image_rect_color', self.save_right, 10)
        
        print("Waiting for images")
    
    def save_left(self, msg):
        if not self.left_saved:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imwrite('left_camera.png', cv_image)
            print("Saved left_camera.png")
            self.left_saved = True
            self.check_complete()
    
    def save_right(self, msg):
        if not self.right_saved:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imwrite('right_camera.png', cv_image)
            print("Saved right_camera.png")
            self.right_saved = True
            self.check_complete()
    
    def check_complete(self):
        if self.left_saved and self.right_saved:
            print("Both images saved")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = QuickCapture()
    rclpy.spin(node)

if __name__ == '__main__':
    main()