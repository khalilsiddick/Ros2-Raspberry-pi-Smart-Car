import rclpy
from rclpy.node import Node
from gpiozero import Robot, Motor, Button, LED  # type: ignore
from std_msgs.msg import String

class AvoidObstaclesNode(Node):
    def __init__(self):
        super().__init__('avoid_obstacles_node')
        
        # Hardware configuration (verify GPIO pins match your wiring)
        self.robot = Robot(
            left=Motor(forward=22, backward=27, enable=18),
            right=Motor(forward=25, backward=24, enable=23)
        )
        self.sensor_right = Button(16, pull_up=True)  # 0=obstacle
        self.sensor_left = Button(12, pull_up=True)   # 0=obstacle
        self.button = Button(19, pull_up=True)
        self.green_led = LED(5)
        self.red_led = LED(6)

        # State management
        self.key_flag = 0
        self.avoidance_step = 0
        self.avoidance_timer = None

        # Set up callbacks
        self.button.when_pressed = self.keyscan
        self.button.when_released = self.released
        
        # ROS2 setup
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def keyscan(self):
        self.get_logger().info('Button Pressed: Starting operation')
        self.red_led.on()
        self.green_led.off()
        self.key_flag = 1

    def released(self):
        self.get_logger().info('Button Released: Stopping operation')
        self.red_led.off()
        self.green_led.on()
        self.key_flag = 0
        self.robot.stop()
        self.reset_avoidance()

    def reset_avoidance(self):
        if self.avoidance_timer:
            self.avoidance_timer.cancel()
        self.avoidance_step = 0

    def start_avoidance(self):
        self.get_logger().info('Starting avoidance maneuver')
        self.avoidance_step = 1
        self.robot.backward(0.6)
        self.avoidance_timer = self.create_timer(0.8, self.avoidance_sequence)

    def avoidance_sequence(self):
        if self.avoidance_step == 1:
            self.get_logger().info('Turning left after backing up')
            self.robot.left(0.9)
            self.avoidance_step = 2
            self.avoidance_timer.cancel()
            self.avoidance_timer = self.create_timer(1.5, self.avoidance_sequence)
        elif self.avoidance_step == 2:
            self.get_logger().info('Resuming normal operation')
            self.robot.stop()
            self.reset_avoidance()

    def control_loop(self):
        if self.key_flag == 1:
            # Read raw sensor values (0=obstacle detected)
            left_obstacle = self.sensor_left.value == 0
            right_obstacle = self.sensor_right.value == 0

            self.get_logger().info(f'Sensors: L={left_obstacle} R={right_obstacle}')

            # Obstacle handling logic
            if not left_obstacle and not right_obstacle:
                self.robot.forward(0.7)
            elif left_obstacle and not right_obstacle:
                self.robot.right(0.85)
            elif right_obstacle and not left_obstacle:
                self.robot.left(0.85)
            else:
                self.robot.stop()
                self.start_avoidance()

        # Publish status
        status_msg = String()
        status_msg.data = f'Active: {self.key_flag}, Avoidance: {self.avoidance_step}'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AvoidObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()