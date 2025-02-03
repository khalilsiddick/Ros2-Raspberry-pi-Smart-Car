import rclpy
from rclpy.node import Node
from gpiozero import Robot, Motor # type: ignore
import cv2
import numpy as np
from pyzbar import pyzbar # type: ignore
import sys
import signal
import time

class QRControlNode(Node):
    def __init__(self):
        super().__init__('qr_control_node')
        self.robot = Robot(left=Motor(forward=22, backward=27, enable=18), 
                           right=Motor(forward=25, backward=24, enable=23))
        self.camera = cv2.VideoCapture(0)
        self.get_logger().info("QR Control Node started.")
        
        # Debouncing mechanism
        self.last_command_time = time.time()
        self.command_interval = 1.0  # Minimum interval between commands (in seconds)
        
        # Register signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.shutdown_requested = False

    def detect_and_decode(self, frame):
        barcodes = pyzbar.decode(frame)
        for barcode in barcodes:
            barcodeData = barcode.data.decode("utf-8")
            if len(barcodeData) > 0:
                self.get_logger().info(f'QR Code detected: {barcodeData}')
                self.command_resolve(barcodeData)

    def command_resolve(self, command):
        current_time = time.time()
        if current_time - self.last_command_time < self.command_interval:
            self.get_logger().info("Ignoring repeated command due to debouncing.")
            return  # Ignore command if it's too soon after the last one
        
        self.last_command_time = current_time
        if "Run" in command:
            self.robot.forward(0.3)  # Increased speed for better response
            self.get_logger().info("Command: Run")
        elif "Back" in command:
            self.robot.backward(0.3)
            self.get_logger().info("Command: Back")
        elif "Left" in command:
            self.robot.left(0.3)
            self.get_logger().info("Command: Left")
        elif "Right" in command:
            self.robot.right(0.3)
            self.get_logger().info("Command: Right")
        elif "Stop" in command:
            self.robot.stop()
            self.get_logger().info("Command: Stop")

    def close_camera(self):
        if self.camera.isOpened():
            self.camera.release()
            self.get_logger().info("Camera released.")

    def signal_handler(self, sig, frame):
        """Signal handler to gracefully shut down the node."""
        self.get_logger().info("Signal received, shutting down...")
        self.shutdown_requested = True

    def run(self):
        """Main loop to capture frames from the camera and process them."""
        try:
            while rclpy.ok() and not self.shutdown_requested:
                ret, frame = self.camera.read()
                if not ret:
                    self.get_logger().error("Failed to capture frame from camera.")
                    break

                # Detect and decode QR codes
                self.detect_and_decode(frame)
        finally:
            self.close_camera()
            self.robot.stop()

def main(args=None):
    rclpy.init(args=args)
    node = QRControlNode()
    try:
        node.run()  # Start the main loop
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()