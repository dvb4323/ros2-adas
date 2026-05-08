from turtle import width

from matplotlib import lines
from networkx import edges

from std_msgs.msg import Float32

import rclpy
from rclpy.node import Node
from sensor_msgs import msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.bridge = CvBridge()
        self.previous_offset = 0.0

        self.subscription = self.create_subscription(
            Image,
            '/carla/actor147/front_cam/image',  # <-- update this
            self.image_callback,
            10
        )

        self.offset_publisher = self.create_publisher(
            Float32,
            '/lane_offset',
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

        if self.frame_id % 10 == 0:  # save every 10 frames
            self.save_debug_images(frame, edges, cropped_edges)
            
        msg = Float32()
        msg.data = float(lane_center_offset)
        self.offset_publisher.publish(msg)

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

        # polygon = np.array([[
        #     (width * 0.1, height),
        #     (width * 0.9, height),
        #     (int(width * 0.7), int(height * 0.55)),
        #     (int(width * 0.3), int(height * 0.55)),
        # ]], np.int32)
        
        # cv2.polylines(frame, [polygon], isClosed=True, color=(0, 255, 0), thickness=2)

        # cv2.fillPoly(mask, polygon, 255)
        
        # Create two separate masks

        # Define a polygon for the left side of the lane
        poly_left = np.array([[(int(width*0.1), height), (int(width*0.25), height), 
                    (int(width*0.35), int(height*0.55)), (int(width*0.25), int(height*0.55))]], np.int32)

        # Define a polygon for the right side of the lane
        poly_right = np.array([[(int(width*0.75), height), (int(width*0.9), height), 
                    (int(width*0.75), int(height*0.55)), (int(width*0.65), int(height*0.55))]], np.int32)

        cv2.polylines(frame, [poly_left], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(frame, [poly_right], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.fillPoly(mask, [poly_left, poly_right], 255)

        # Extract edges from both sides
        cropped_edges = cv2.bitwise_and(edges, mask)

        # 5. Hough Transform
        lines = cv2.HoughLinesP(
            cropped_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=30,
            maxLineGap=20
        )

        if lines is None:
            return self.previous_offset, edges, cropped_edges

        left_lines = []
        right_lines = []

        # 6. Separate lines
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 == x1:
                continue

            slope = (y2 - y1) / (x2 - x1) if (x2 != x1) else 999

            if abs(slope) < 0.1:
                continue

            mid_x = (x1 + x2) / 2

            if slope < 0:
                left_lines.append(line[0])
            else:
                right_lines.append(line[0])

        sample_rows = [
            int(height * 0.90),  # near
            int(height * 0.75), # middle
            int(height * 0.60),  # far
        ]

        offsets = []

        for y in sample_rows:

            left_x = self.average_line_x(left_lines, y)
            right_x = self.average_line_x(right_lines, y)
            
            self.get_logger().info(
                f'Row {y}: Left_x={left_x}, Right_x={right_x}'
            )
            
            if left_x is None or right_x is None:
                continue

            lane_width = abs(right_x - left_x)
            self.get_logger().info(f'Row {y}: Lane width={lane_width}')

            if lane_width < 100 or lane_width > 500:
                continue

            lane_center = (left_x + right_x) / 2.0
            vehicle_center = width / 2.0

            offset = lane_center - vehicle_center

            offsets.append(offset)
            
        if len(offsets) == 0:
            return self.previous_offset, edges, cropped_edges
        
        weights = [0.2, 0.3, 0.5][:len(offsets)]

        weighted_offset = sum(
            w * o for w, o in zip(weights, offsets)
        )    

        alpha = 0.2

        weighted_offset = (
            alpha * weighted_offset +
            (1 - alpha) * self.previous_offset
        )
        
        self.previous_offset = weighted_offset
        
        return weighted_offset, edges, cropped_edges

    def average_line_x(self, lines, target_y):
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

        if abs(m) < 1e-3:
            return None

        # Compute x at target_y
        x = (target_y - b) / m

        return int(x)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()