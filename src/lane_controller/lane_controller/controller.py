#!/usr/bin/env python3

from std_msgs.msg import Float32

import rclpy
from rclpy.node import Node

import carla
import time


class LaneController(Node):

    def __init__(self):
        super().__init__('lane_controller')

        self.get_logger().info('Connecting to CARLA...')

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)

        self.world = self.client.get_world()

        self.vehicle = None

        # Find hero vehicle
        self.find_hero_vehicle()

        # Timer: send control at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.lane_offset = 0.0

        self.subscription = self.create_subscription(
            Float32,
            '/lane_offset',
            self.offset_callback,
            10
        )
        
        # PID initialization
        self.error_integral = 0.0
        self.previous_error = 0.0
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001
        self.first_run = True

    def find_hero_vehicle(self):

        while self.vehicle is None:

            actors = self.world.get_actors().filter('vehicle.*')

            for actor in actors:
                if actor.attributes.get('role_name') == 'hero':
                    self.vehicle = actor
                    break

            if self.vehicle is None:
                self.get_logger().info('Waiting for hero vehicle...')
                time.sleep(1)

        self.get_logger().info(
            f'Found hero vehicle: {self.vehicle.id}'
        )
        
    def offset_callback(self, msg):
        self.lane_offset = msg.data    

    def control_loop(self):
        error = self.lane_offset
        
        if self.first_run:
            self.prev_error = error
            self.first_run = False
            return # Skip this frame to let values stabilize
        
        # Accumulate integral error
        self.error_integral += error
        # Limit integral to prevent windup
        self.error_integral = max(-5.0, min(5.0, self.error_integral))
        
        derivative = error - self.previous_error
        
        # PID formula
        # steer = (self.Kp * error)
        steer = (self.Kp * error) + (self.Ki * self.error_integral) + (self.Kd * derivative)
        steer = max(-0.8, min(0.8, steer))
        
        control = carla.VehicleControl()

        # Constant forward movement
        control.throttle = 0.15

        # Constant steering test
        # Kp = 0.01
        # steer = Kp * self.lane_offset
        # steer = max(-0.5, min(0.5, steer))  # clamp to [-0.5, 0.5]
        control.steer = float(steer)

        self.vehicle.apply_control(control)

        self.get_logger().info(
            f'Offset={self.lane_offset:.2f}, Steer={steer:.2f}'
        )
        
        self.previous_error = error


def main(args=None):

    rclpy.init(args=args)

    node = LaneController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()