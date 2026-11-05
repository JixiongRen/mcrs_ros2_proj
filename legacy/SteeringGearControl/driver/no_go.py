import motor
import time
import pygame
import RPi.GPIO as GPIO

# 舵机部分
motor.serial_port.port = '/dev/ttyUSB0'
motor.serial_port.baudrate = 57600
motor.serial_port.timeout = 1

while True:
    try:
        motor.serial_port.open()
        break
    except:
        print('串口错误！')

print('连接成功！')
time.sleep(2)
print('准备就绪，开始')

def use_wheel_mode(motor_id, flag, goal_RPM=20):
    motor.motor_id = motor_id
    motor.velocity_mode()
    if flag > 0.5:
        motor.send_goal_RPM((-1) * goal_RPM)
        value1 = motor.read_position()
        print(value1)
    elif -0.5 < flag < 0.5:
        motor.send_goal_RPM(0)
        value1 = motor.read_position()
        print(value1)
    elif flag < -0.5:
        motor.send_goal_RPM(goal_RPM)
        value1 = motor.read_position()
        print(value1)

# 步进电机部分
GPIO.setmode(GPIO.BOARD)
PUL = 29
DIR = 31
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

def set_motor_forward(flag):
    GPIO.output(DIR, GPIO.LOW)
    #while flag == 1:
    for _ in range(50):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)

def set_motor_reverse(flag):
    GPIO.output(DIR, GPIO.HIGH)
    for _ in range(50):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)

def stop_motor(flag):
    GPIO.output(DIR, GPIO.HIGH)
    for _ in range(0):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.002)

# 初始化pygame
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
clock = pygame.time.Clock()
frame_rate = 4800

# 游戏循环
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)

    joystick_value_1 = joystick.get_axis(1)
    joystick_value_3 = joystick.get_axis(3)

    if joystick_value_1 > 0.5:
        flag_1 = 1
    elif joystick_value_1 < -0.5:
        flag_1 = -1
    else:
        flag_1 = 0

    if joystick_value_3 > 0.5:
        flag_3 = 1
        set_motor_forward(flag_3)
        GPIO.cleanup()
    elif joystick_value_3 < -0.5:
        flag_3 = -1
        set_motor_reverse(flag_3)
        GPIO.cleanup()
    else:
        flag_3 = 0
        stop_motor(flag_3)
        GPIO.cleanup()

    use_wheel_mode(1, flag_1)

    clock.tick(frame_rate)
