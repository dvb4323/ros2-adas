import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/carla/actor29/front_cam/image',  # <-- update this
            self.image_callback,
            10
        )

    def save_debug_images(self, frame, edges, cropped):
        cv2.imwrite('/tmp/frame.jpg', frame)
        cv2.imwrite('/tmp/edges.jpg', edges)
        cv2.imwrite('/tmp/roi.jpg', cropped)
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        lane_center_offset, edges, cropped_edges = self.process_frame(frame)

        self.get_logger().info(f'Offset: {lane_center_offset:.2f}')
        self.frame_id = getattr(self, 'frame_id', 0) + 1

        if self.frame_id % 20 == 0:  # save every 20 frames
            self.save_debug_images(frame, edges, cropped_edges)

    def process_frame(self, frame):
        edges = None
        cropped_edges = None
        
        height, width, _ = frame.shape

        # 1. Grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. Blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Canny
        edges = cv2.Canny(blur, 50, 150)

        # 4. ROI mask
        mask = np.zeros_like(edges)

        polygon = np.array([[
            (0, height),
            (width, height),
            (int(width * 0.6), int(height * 0.6)),
            (int(width * 0.4), int(height * 0.6)),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        # 5. Hough Transform
        lines = cv2.HoughLinesP(
            cropped_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=10,
            maxLineGap=5
        )

        if lines is None:
            return 0.0, edges, cropped_edges

        left_lines = []
        right_lines = []

        # 6. Separate lines
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1)

            if abs(slope) < 0.5:
                continue

            if slope < 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

        left_x = self.average_line_x(left_lines, height)
        right_x = self.average_line_x(right_lines, height)

        if left_x is None or right_x is None:
            return 0.0, edges, cropped_edges

        # 7. Compute offset
        lane_center = (left_x + right_x) / 2.0
        vehicle_center = width / 2.0

        offset = lane_center - vehicle_center

        return offset, edges, cropped_edges

    def average_line_x(self, lines, height):
        if len(lines) == 0:
            return None

        x_coords = []
        y_coords = []

        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]

        # Fit line: y = mx + b
        fit = np.polyfit(x_coords, y_coords, 1)
        m, b = fit

        # Compute x at bottom of image
        y = height
        x = (y - b) / m

        return int(x)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()