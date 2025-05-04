# --- Imports ---
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import message_filters  # Sync RGB + depth
from std_msgs.msg import String

# --- Node Definition ---
class ConeDetector3D(Node):
    def __init__(self):
        super().__init__('cone_detector_3d_node')
        self.bridge = CvBridge()

        # Subscribe to RGB + depth images
        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed/left/image_rect_color')
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed/depth/image_raw')

        # Sync both image streams
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        self.get_logger().info("ConeDetector 3D node started.")
        self.publisher_ = self.create_publisher(String, 'detected_cones', 10)

    def image_callback(self, rgb_msg, depth_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # Cone colour (HSV) masks
        blue_mask = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
        yellow_mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([35, 255, 255]))
        orange_mask = cv2.inRange(hsv, np.array([5, 100, 100]), np.array([20, 255, 255]))

        # Combine all
        combined_mask = blue_mask | yellow_mask | orange_mask

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fx, fy = 525.0, 525.0
        cx, cy = rgb_image.shape[1] // 2, rgb_image.shape[0] // 2

        message_lines = []  # All cone lines to be published

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 20:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx_pixel = x + w // 2
            cy_pixel = y + h // 2

            depth = float(depth_image[cy_pixel, cx_pixel])
            if np.isnan(depth) or depth == 0.0:
                continue

            Z = depth
            X = (cx_pixel - cx) * Z / fx
            Y = (cy_pixel - cy) * Z / fy

            if blue_mask[cy_pixel, cx_pixel]:
                colour, label = "blue", 0
            elif yellow_mask[cy_pixel, cx_pixel]:
                colour, label = "yellow", 1
            elif orange_mask[cy_pixel, cx_pixel]:
                colour, label = "orange", 2
            else:
                continue  # not valid

            message_lines.append(f"{X:.2f},{Y:.2f},{Z:.2f},{label}")
            print(f"Detected cone: Colour={colour}  X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")

            cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(rgb_image, f"{colour} [{X:.1f}, {Y:.1f}, {Z:.1f}]", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Publish message
        if message_lines:
            msg = String()
            msg.data = "\n".join(message_lines)
            self.publisher_.publish(msg)

        cv2.imshow("Cone Detection 3D", rgb_image)
        cv2.waitKey(1)

# --- Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

