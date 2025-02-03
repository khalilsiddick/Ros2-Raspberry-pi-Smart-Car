import RPi.GPIO as GPIO
import time

# 设置GPIO模式
GPIO.setmode(GPIO.BCM)

# 设置PWM引脚（例如GPIO 18）
pwm_pin = 18
GPIO.setup(pwm_pin, GPIO.OUT)

# 创建PWM对象，设置频率为50Hz
pwm = GPIO.PWM(pwm_pin, 50)
pwm.start(0)

try:
    while True:
        for duty_cycle in range(0, 101, 10):  # 从0%到100%的占空比
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
        for duty_cycle in range(100, -1, -10):  # 从100%到0%的占空比
            pwm.ChangeDutyCycle(duty_cycle)
            time.sleep(0.1)
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()