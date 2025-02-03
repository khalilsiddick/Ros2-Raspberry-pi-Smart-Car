from gpiozero import Robot, Motor
from time import sleep

# 创建机器人对象，指定左右电机的引脚
robot = Robot(left=Motor(forward=22, backward=27, enable=18), right=Motor(forward=25, backward=24, enable=23))

if __name__ == "__main__":
    try:
        while True:
            # 前进
            robot.forward(0.3)
            sleep(1.5)
            robot.stop()
            sleep(1)  # 停止1秒

            # 后退
            robot.backward(0.3)
            sleep(1.5)
            robot.stop()
            sleep(1)  # 停止1秒

            # 左转
            robot.left(0.7)
            sleep(1.5)
            robot.stop()
            sleep(1)  # 停止1秒

            # 右转
            robot.right(0.7)
            sleep(1.5)
            robot.stop()
            sleep(1)  # 停止1秒
    except KeyboardInterrupt:
        # 按Ctrl+C退出时，停止机器人
        robot.stop()
