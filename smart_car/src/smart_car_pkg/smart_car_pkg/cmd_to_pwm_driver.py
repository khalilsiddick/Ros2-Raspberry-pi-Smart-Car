#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
import time

class CmdToPWMNode(Node):
    def __init__(self):
        super().__init__('cmd_to_pwm_driver')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        self.left_motor = Motor(forward=22, backward=27, enable=18)
        self.right_motor = Motor(forward=25, backward=24, enable=23)

        self.target_speed = 0.0
        self.current_speed = 0.0
        self.acceleration = 0.1  # 调整加速度大小

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Twist: linear_x={msg.linear.x}, angular_z={msg.angular.z}")

        # Limit the speed to the range [-1, 1]
        speed = max(-1.0, min(1.0, msg.linear.x))
        turn = max(-1.0, min(1.0, msg.angular.z))

        # Calculate target speed and turning
        self.target_speed = speed
        self.turn = turn

        # Update motor speeds
        self.update_motors()

    def update_motors(self):
        # Calculate individual wheel speeds
        left_speed = self.target_speed - (self.turn * 0.5)
        right_speed = self.target_speed + (self.turn * 0.5)

        # Gradually change speed to avoid jerkiness
        self.current_speed = self.smooth_transition(self.current_speed, self.target_speed)

        # Ensure speeds are within the valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Forward motion
        if self.current_speed > 0:
            self.left_motor.forward(min(1.0, abs(left_speed)))
            self.right_motor.forward(min(1.0, abs(right_speed)))
            self.get_logger().info(f"Moving forward at speed: {self.current_speed}")
        # Backward motion
        elif self.current_speed < 0:
            self.left_motor.backward(min(1.0, abs(-self.current_speed)))
            self.right_motor.backward(min(1.0, abs(-self.current_speed)))
            self.get_logger().info(f"Moving backward at speed: {-self.current_speed}")
        # Stop
        else:
            self.left_motor.stop()
            self.right_motor.stop()
            self.get_logger().info("Motors stopped")

        # Handle turning
        if self.turn > 0:
            self.left_motor.forward(min(1.0, abs(left_speed)))
            self.right_motor.backward(min(1.0, abs(right_speed)))
            self.get_logger().info(f"Turning left at speed: {self.turn}")
        elif self.turn < 0:
            self.left_motor.backward(min(1.0, abs(left_speed)))
            self.right_motor.forward(min(1.0, abs(right_speed)))
            self.get_logger().info(f"Turning right at speed: {-self.turn}")

    def smooth_transition(self, current, target):
        if current < target:
            current += self.acceleration
            if current > target:
                current = target
        elif current > target:
            current -= self.acceleration
            if current < target:
                current = target
        return current

def main(args=None):
    rclpy.init(args=args)
    node = CmdToPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()