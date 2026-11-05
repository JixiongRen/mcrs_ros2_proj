import RPi.GPIO as GPIO
import time


# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)

# 定义引脚
PUL = 29 # 步进脉冲
DIR = 31  # 步进方向

# 设置引脚为输出
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)


# 正转
GPIO.output(DIR, GPIO.LOW)  # 设置方向
for _ in range(round(25600*(30/17))):  # 发送足够的脉冲来完成一次正转
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(0.0002)

# # 反转
# GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转
# for _ in range(1600):  # 发送足够的脉冲来完成一次反转
#     GPIO.output(PUL, GPIO.HIGH)
#     time.sleep(0.002)
#     GPIO.output(PUL, GPIO.LOW)
#     time.sleep(0.002)

# # 清理GPIO设置
# GPIO.cleanup()
