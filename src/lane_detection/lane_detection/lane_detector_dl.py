import torch
from torchvision import models
import torchvision.transforms as transforms
import time

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
        super().__init__('lane_detector_dl')

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
            '/lane_offset_dl',
            10
        )
        
        self.device=torch.device('cpu')
        
        self.model = models.segmentation.deeplabv3_mobilenet_v3_large(
            pretrained=True
        )

        self.model.to(self.device)
        self.model.eval()
        
        self.transform = transforms.Compose([
            transforms.ToTensor(),
        ])
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        lane_center_offset, mask = self.process_frame_dl(frame)
        
        mask_vis = (mask * 10).astype(np.uint8)

        cv2.imshow("Segmentation", mask_vis)
        cv2.waitKey(1)

        self.get_logger().info(f'Offset: {lane_center_offset:.2f}')
        self.frame_id = getattr(self, 'frame_id', 0) + 1
        msg = Float32()
        msg.data = float(lane_center_offset)
        self.offset_publisher.publish(msg)

    def process_frame_dl(self, frame):

        height, width, _ = frame.shape

        # Resize for CPU performance
        frame_small = cv2.resize(frame, (320, 180))

        rgb = cv2.cvtColor(frame_small, cv2.COLOR_BGR2RGB)

        tensor = self.transform(rgb).unsqueeze(0).to(self.device)

        start = time.time()

        with torch.no_grad():
            output = self.model(tensor)['out'][0]

        fps = 1.0 / (time.time() - start)

        self.get_logger().info(f'Inference FPS: {fps:.2f}')

        mask = output.argmax(0).byte().cpu().numpy()

        # Use lower half of image
        lower_half = mask[mask.shape[0]//2:, :]

        lane_pixels = np.where(lower_half > 0)

        if len(lane_pixels[1]) == 0:
            return self.previous_offset, mask

        lane_center = np.mean(lane_pixels[1])

        image_center = lower_half.shape[1] / 2

        offset = lane_center - image_center

        # smoothing
        alpha = 0.2

        offset = (
            alpha * offset +
            (1 - alpha) * self.previous_offset
        )

        self.previous_offset = offset

        return offset, mask


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()