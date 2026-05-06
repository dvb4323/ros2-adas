import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class LaneDetector(Node):

    def __init__(self):
        super().__init__('lane_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/carla/actor30/front_cam/image',  # ⚠️ change if actor id changes
            self.image_callback,
            10
        )

        self.get_logger().info("Lane Detector Node Started")

    def image_callback(self, msg):
        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Show image
        # cv2.imshow("Camera", frame)
        # cv2.waitKey(1)
        
        # No GUI
        self.get_logger().info(f"Frame received: {frame.shape}")


def main(args=None):
    rclpy.init(args=args)

    node = LaneDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
