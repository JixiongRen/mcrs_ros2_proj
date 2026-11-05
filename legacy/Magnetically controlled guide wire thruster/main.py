import motor
import time
import pygame
import threading
import RPi.GPIO as GPIO

def print_at_position(x, y, text):
    # 使用 ANSI 转义码将光标移动到指定位置
    print(f"\033[{y};{x}H{text}", end='', flush=True)
    
import os

def clear_screen():
    # 获取终端宽度
    _, columns = os.popen('stty size', 'r').read().split()
    # 将光标移动到第1行第6列
    move_cursor = f"\033[1;6H"
    # 清除从第6列到屏幕右侧的内容
    clear_content = "\033[J"
    # 打印清屏命令
    print("\033c", end='', flush=True)
    # 打印移动光标和清除内容的命令
    print(move_cursor + clear_content, end='', flush=True)




print("\033c", end='', flush=True)
"""
舵机部分
"""

'''
连接DYNAMIXEL
'''
while True:#Conecting Loop
    motor.serial_port.port = '/dev/ttyUSB1'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('串口错误！')

print('连接成功！')
time.sleep(2)
print("舵机清零中...")
motor.position_mode()
motor.move_pos(0)
print('清零完成，准备就绪，开始')

'''
控制舵机转动的函数
'''
def use_wheel_mode(motor_id, flag, goal_RPM=40):
    global current_pos
    motor.motor_id = motor_id
    motor.velocity_mode()
    if flag == 1:
        motor.send_goal_RPM((-1)*goal_RPM)
        current_pos = motor.read_position()
        # print(value1)
    if flag == 0:
        motor.send_goal_RPM(0)
        current_pos = motor.read_position()
        # print(value1)
    if flag == -1:
        motor.send_goal_RPM(goal_RPM)
        current_pos = motor.read_position()
        # print(value1)


# 定义引脚
PUL = 29  # 步进脉冲
DIR = 31  # 步进方向

# 设置GPIO模式
GPIO.setmode(GPIO.BOARD)

# 设置引脚为输出
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# 初始化pygame
pygame.init()


# 设置手柄
joystick = pygame.joystick.Joystick(0)
joystick.init()

flag_3 = 0
flag_1 = 0
current_pos = 0
exit_event = threading.Event()  # 用于通知线程退出的事件

def set_motor_forward():
    """
    设置电机正转的逻辑。
    """
    GPIO.output(DIR, GPIO.LOW)  # 设置方向
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(0.0002)

def set_motor_reverse():
    """
    设置电机反转的逻辑。
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(0.0002)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(0.0002)

def stop_motor():
    """
    停止电机的逻辑。
    """
    GPIO.output(DIR, GPIO.HIGH)  # 设置方向为反转


# 电机控制线程
def motor_control_thread():
    global flag_3
    pul_num = 0
    while not exit_event.is_set():
        joystick_value_3 = joystick.get_axis(3)
        flag_3 = 1 if joystick_value_3 > 0.5 else (-1 if joystick_value_3 < -0.5 else 0)
        if flag_3 != 0:
            if flag_3 > 0:
                set_motor_forward()
                pul_num += 1
            else:
                set_motor_reverse()
                pul_num -= 1
        else:
            stop_motor()
        rotation_angle = "{:.1f}".format(((pul_num/6400)*360)*(17/30))
        print_at_position(1, 6, f"旋转角度: {rotation_angle}°")
        time.sleep(0.001)
        
        
# 舵机控制线程
def steering_control_thread():
    global flag_1
    global current_pos
    flag_1_old = 0
    use_wheel_mode(1, flag_1_old)
    while not exit_event.is_set():
        joystick_value_1 = joystick.get_axis(1)
        flag_1 = 1 if joystick_value_1 > 0.5 else (-1 if joystick_value_1 < -0.5 else 0)
        if flag_1 != flag_1_old:
            flag_1_old = flag_1
            use_wheel_mode(1, flag_1_old)
        current_dis = "{:.2f}".format((current_pos/4096)*2*3.1415926*6)
        try:
            print_at_position(1, 7, f"进动距离：{current_dis}mm")
        except Exception as e:
            pass
        #time.sleep(0.001)
        
# 清屏线程
def clear_screen_thread():
        while not exit_event.is_set():
            clear_screen()
            time.sleep(0.1)

# 启动电机控制线程
motor_thread = threading.Thread(target=motor_control_thread)
motor_thread.start()

steering_thread = threading.Thread(target=steering_control_thread)
steering_thread.start()

clear_screen_thread = threading.Thread(target=clear_screen_thread)
clear_screen_thread.start()
try:
    while not exit_event.is_set():
        pygame.event.pump()  # 处理pygame事件
        time.sleep(0.000001)
except KeyboardInterrupt:
    pass
finally:
    exit_event.set()
    motor_thread.join()
    steering_thread.join()
    GPIO.cleanup()  # 清理GPIO资源
    pygame.joystick.quit()
    pygame.quit()
