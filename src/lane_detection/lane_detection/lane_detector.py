import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

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
            '/carla/actor147/front_cam/image',
            self.image_callback,
            10
        )

        self.offset_publisher = self.create_publisher(
            Float32,
            '/lane_offset',
            10
        )

        self.frame_id = 0

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        offset, binary, bird_eye, debug = self.process_frame(frame)

        self.get_logger().info(
            f'Offset: {offset:.2f}'
        )

        out_msg = Float32()
        out_msg.data = float(offset)

        self.offset_publisher.publish(out_msg)

        self.frame_id += 1

        if self.frame_id % 10 == 0:
            self.save_debug_images(
                frame,
                binary,
                bird_eye,
                debug
            )

    def save_debug_images(
        self,
        frame,
        binary,
        bird_eye,
        debug
    ):

        cv2.imwrite('/tmp/frame.jpg', frame)
        cv2.imwrite('/tmp/binary.jpg', binary)
        cv2.imwrite('/tmp/bird_eye.jpg', bird_eye)
        cv2.imwrite('/tmp/debug.jpg', debug)

    def process_frame(self, frame):

        height, width, _ = frame.shape

        ####################################################
        # 1. GRAYSCALE
        ####################################################

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        ####################################################
        # 2. BLUR
        ####################################################

        blur = cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

        ####################################################
        # 3. THRESHOLD
        ####################################################

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower_white = np.array([0, 0, 140])
        upper_white = np.array([180, 80, 255])

        binary = cv2.inRange(
            hsv,
            lower_white,
            upper_white
        )
        
        # Dilation
        kernel = np.ones((3,3), np.uint8)

        binary = cv2.dilate(
            binary,
            kernel,
            iterations=1
        )

        ####################################################
        # 4. PERSPECTIVE TRANSFORM
        ####################################################

        src = np.float32([
            [220, 240],   # top-left
            [420, 240],   # top-right
            [590, 470],   # bottom-right
            [50, 470]     # bottom-left
        ])

        dst = np.float32([
            [150, 0],     # top-left
            [490, 0],     # top-right
            [490, 479],   # bottom-right
            [150, 479]    # bottom-left
        ])

        matrix = cv2.getPerspectiveTransform(
            src,
            dst
        )
        
        debug_frame = frame.copy()
        src_int = src.astype(np.int32)
        
        cv2.polylines(
            debug_frame,
            [src_int],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2
        )
        
        cv2.imwrite('/tmp/src_debug.jpg', debug_frame)
        cv2.imwrite('/tmp/threshold.jpg', binary)

        bird_eye = cv2.warpPerspective(
            binary,
            matrix,
            (width, height)
        )

        ####################################################
        # 5. HISTOGRAM
        ####################################################

        histogram = np.sum(
            bird_eye[height // 2:, :],
            axis=0
        )

        # midpoint = width // 2

        # left_base = np.argmax(
        #     histogram[:midpoint]
        # )

        # right_base = np.argmax(
        #     histogram[midpoint:]
        # ) + midpoint

        left_search_region = histogram[100:320]
        if np.max(left_search_region) > 0:
            left_base = np.argmax(left_search_region) + 100
        else:
            left_base = 150 # Default fallback to your DST left edge

        # We search from x=320 to x=540 for the right lane
        right_search_region = histogram[320:540]
        if np.max(right_search_region) > 0:
            right_base = np.argmax(right_search_region) + 320
        else:
            right_base = 490 # Default fallback to your DST right edge

        ####################################################
        # 6. FIND NONZERO PIXELS
        ####################################################

        nonzero = bird_eye.nonzero()

        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        ####################################################
        # 7. SLIDING WINDOWS
        ####################################################

        nwindows = 8
        margin = 50
        minpix = 30

        window_height = height // nwindows

        left_current = left_base
        right_current = right_base

        left_lane_inds = []
        right_lane_inds = []

        debug = np.dstack((
            bird_eye,
            bird_eye,
            bird_eye
        ))

        for window in range(nwindows):

            win_y_low = height - (
                window + 1
            ) * window_height

            win_y_high = height - (
                window
            ) * window_height

            ################################################
            # LEFT WINDOW
            ################################################

            win_xleft_low = left_current - margin
            win_xleft_high = left_current + margin

            ################################################
            # RIGHT WINDOW
            ################################################

            win_xright_low = right_current - margin
            win_xright_high = right_current + margin

            ################################################
            # DRAW WINDOWS
            ################################################

            cv2.rectangle(
                debug,
                (win_xleft_low, win_y_low),
                (win_xleft_high, win_y_high),
                (0, 255, 0),
                2
            )

            cv2.rectangle(
                debug,
                (win_xright_low, win_y_low),
                (win_xright_high, win_y_high),
                (0, 255, 0),
                2
            )

            ################################################
            # IDENTIFY PIXELS
            ################################################

            good_left_inds = (
                (
                    nonzeroy >= win_y_low
                ) &
                (
                    nonzeroy < win_y_high
                ) &
                (
                    nonzerox >= win_xleft_low
                ) &
                (
                    nonzerox < win_xleft_high
                )
            ).nonzero()[0]

            good_right_inds = (
                (
                    nonzeroy >= win_y_low
                ) &
                (
                    nonzeroy < win_y_high
                ) &
                (
                    nonzerox >= win_xright_low
                ) &
                (
                    nonzerox < win_xright_high
                )
            ).nonzero()[0]

            left_lane_inds.append(
                good_left_inds
            )

            right_lane_inds.append(
                good_right_inds
            )

            ################################################
            # RECENTER WINDOWS
            ################################################

            if len(good_left_inds) > minpix:

                left_current = int(
                    np.mean(
                        nonzerox[good_left_inds]
                    )
                )

            if len(good_right_inds) > minpix:

                right_current = int(
                    np.mean(
                        nonzerox[good_right_inds]
                    )
                )

        ####################################################
        # 8. CONCATENATE INDICES
        ####################################################

        left_lane_inds = np.concatenate(
            left_lane_inds
        )

        right_lane_inds = np.concatenate(
            right_lane_inds
        )

        ####################################################
        # 9. EXTRACT PIXELS
        ####################################################

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]

        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        ####################################################
        # 10. VALIDATION
        ####################################################

        if (
            len(leftx) < 200 or
            len(rightx) < 200
        ):

            return (
                self.previous_offset,
                binary,
                bird_eye,
                debug
            )

        ####################################################
        # 11. POLYNOMIAL FIT
        ####################################################

        # left_fit = np.polyfit(
        #     lefty,
        #     leftx,
        #     2
        # )

        # right_fit = np.polyfit(
        #     righty,
        #     rightx,
        #     2
        # )
        LANE_WIDTH_PIXELS = 270 

        # Check if we have enough pixels for each side
        left_is_valid = len(leftx) > 200
        right_is_valid = len(rightx) > 200

        if left_is_valid and right_is_valid:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
        elif left_is_valid and not right_is_valid:
            left_fit = np.polyfit(lefty, leftx, 2)
            # Project right lane from left lane
            right_fit = left_fit.copy()
            right_fit[2] += LANE_WIDTH_PIXELS 
        elif right_is_valid and not left_is_valid:
            right_fit = np.polyfit(righty, rightx, 2)
            # Project left lane from right lane
            left_fit = right_fit.copy()
            left_fit[2] -= LANE_WIDTH_PIXELS
        else:
            # Use previous data if both are lost
            return self.previous_offset, binary, bird_eye, debug

        ####################################################
        # 12. COMPUTE LANE CENTER
        ####################################################

        y_eval = height - 1

        left_x = (
            left_fit[0] * y_eval ** 2 +
            left_fit[1] * y_eval +
            left_fit[2]
        )

        right_x = (
            right_fit[0] * y_eval ** 2 +
            right_fit[1] * y_eval +
            right_fit[2]
        )

        lane_center = (
            left_x + right_x
        ) / 2.0

        vehicle_center = width / 2.0

        offset = lane_center - vehicle_center

        ####################################################
        # 13. SMOOTHING
        ####################################################

        MAX_JUMP = 50

        delta = (
            offset - self.previous_offset
        )

        delta = np.clip(
            delta,
            -MAX_JUMP,
            MAX_JUMP
        )

        offset = (
            self.previous_offset + delta
        )

        alpha = 0.2

        offset = (
            alpha * offset +
            (1 - alpha) * self.previous_offset
        )

        self.previous_offset = offset

        ####################################################
        # 14. DRAW FITTED LINES
        ####################################################

        ploty = np.linspace(
            0,
            height - 1,
            height
        )

        left_fitx = (
            left_fit[0] * ploty ** 2 +
            left_fit[1] * ploty +
            left_fit[2]
        )

        right_fitx = (
            right_fit[0] * ploty ** 2 +
            right_fit[1] * ploty +
            right_fit[2]
        )

        for i in range(len(ploty)):

            cv2.circle(
                debug,
                (
                    int(left_fitx[i]),
                    int(ploty[i])
                ),
                2,
                (255, 0, 0),
                -1
            )

            cv2.circle(
                debug,
                (
                    int(right_fitx[i]),
                    int(ploty[i])
                ),
                2,
                (0, 0, 255),
                -1
            )
            
            target_left_x = right_fitx - 270 

            for k in range(0, len(ploty), 10): # Draw as a dashed yellow line for clarity
                cv2.circle(debug, (int(target_left_x[k]), int(ploty[k])), 2, (0, 255, 255), -1)

        ####################################################
        # 15. RETURN
        ####################################################

        return (
            offset,
            binary,
            bird_eye,
            debug
        )


def main(args=None):

    rclpy.init(args=args)

    node = LaneDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()