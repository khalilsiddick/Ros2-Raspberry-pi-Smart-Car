import rclpy
from rclpy.node import Node
import gpiozero # type: ignore
from gpiozero import Robot, Motor # type: ignore
import pylirc # type: ignore

class RemoteControlNode(Node):
    def __init__(self):
        super().__init__('remote_control_node')
        # Initialize motors
        self.motor_left = Motor(forward=17, backward=27, enable=22)
        self.motor_right = Motor(forward=23, backward=24, enable=25)
        self.robot = Robot(left=self.motor_left, right=self.motor_right)
        
        # Initialize LIRC
        pylirc.init("pylirc", "/etc/lirc/lircrc", blocking=False)
        
        # Create a timer to check IR commands periodically
        self.timer = self.create_timer(0.1, self.check_ir_command)  # Check every 100ms

    def control_motor(self, command):
        if command == "forward":
            self.robot.forward(0.5)
        elif command == "backward":
            self.robot.backward(0.5)
        elif command == "left":
            self.robot.left(0.5)
        elif command == "right":
            self.robot.right(0.5)
        elif command == "stop":
            self.robot.stop()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def check_ir_command(self):
        try:
            codes = pylirc.nextcode(1)  # Fetch up to 1 command
            while codes:
                for code in codes:
                    command = code["config"]  # Assumes LIRC maps to 'config' string
                    self.get_logger().info(f"Received command: {command}")
                    self.control_motor(command)
                codes = pylirc.nextcode(1)
        except Exception as e:
            self.get_logger().error(f"Error reading IR command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RemoteControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()