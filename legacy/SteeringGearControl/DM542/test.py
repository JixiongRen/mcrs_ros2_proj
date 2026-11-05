import RPi.GPIO as GPIO
import time

# 设置GPIO模式为BCM
GPIO.setmode(GPIO.BCM)

# 定义步进电机的控制引脚
step_pin = 29
dir_pin = 31

# 设置引脚为输出
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)

# 定义步进电机控制参数
delay = 0.1  # 步进间隔时间，控制步进速度

# 设置方向，0为正向，1为反向
direction = 0

# 步进电机控制函数
def step():
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(step_pin, GPIO.LOW)
    time.sleep(delay)

try:
    # 设置方向
    GPIO.output(dir_pin, direction)

    # 步进电机运动一定步数，可以根据需要修改
    steps = 200
    for _ in range(steps):
        step()

except KeyboardInterrupt:
    pass

finally:
    # 清理GPIO设置
    GPIO.cleanup()
