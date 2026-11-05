import threading
import time
import RPi.GPIO as GPIO
import os
import spidev
import sys
import PIDclass
import tool
import motor

def setSetPoint():
    global setPoint
    while not exit_event.is_set():
        if setPoint is None:
            try:
                double_input = input("请输入磁控导丝目标推进量: ")
                if double_input == 'q':
                    break
                with data_lock:
                    setPoint = float(double_input)
            except ValueError:
                print("error: 输入无效")

def driveSteering():
    pid_controller = PIDclass.PIDController(Kp=0.1, Ki=0.01, Kd=0.05)
    global setPoint, radius
    while not exit_event.is_set():
        with data_lock:
            # 等待新数据输入
            if setPoint is not None:
                data = setPoint
                print("正在处理数据:", data)
                time.sleep(2)
                # 在这里进行数据处理
                target_distance = setPoint
                while True:
                    error = target_distance - feedback_distance
                    if abs(error) < 0.1:
                        print("舵机已达到目标位置")
                        break
                output = pid_controller.calculate(error)
                rotation = 4096 * (output / (2 * 3.1415926 * radius))
                tool.use_position_mode(motor_id, rotation)
                setPoint = None  # 处理完成后清空输入数据
        # time.sleep(1)  # 模拟处理时间

def distanceMeansure():
    """传感器工作线程
    """
    global channel
    global spi
    global feedback_distance
    spi.open(0,0)
    spi.max_speed_hz = 9600
    while not exit_event.is_set():
        with data_lock:
            data = tool.analogInput(channel, spi)
            if data > 1023:
                dis_exec = 0
            else:
                dis_exec = round(tool.Dis(data),2)
            feedback_distance = dis_exec

# def main():
#     # 连接至DYNAMIXEL并清零
#     while True:#Conecting Loop
#         motor.serial_port.port = '/dev/ttyUSB0'
#         motor.serial_port.baudrate = 57600
#         motor.serial_port.timeout = 1
#         try:
#             motor.serial_port.open()
#             break
#         except:
#             print('串口错误！')
#     print('连接成功！')
#     time.sleep(2)
#     print("舵机清零中...")
#     motor.position_mode()
#     motor.move_pos(0)
#     print('清零完成，准备就绪，开始')

#     setSetPoint_thread = threading.Thread(target=setSetPoint)
#     setSetPoint_thread.start()

#     driveSteering_thread = threading.Thread(target=driveSteering)
#     driveSteering_thread.start()

#     distance_meansure_thread = threading.Thread(target=distanceMeansure)
#     distance_meansure_thread.start()

#     setSetPoint_thread.join()
#     driveSteering_thread.join()
#     distance_meansure_thread.join()

# if __name__ == "__main__":
spi = spidev.SpiDev()
channel = 1
feedback_distance = 0
data_lock = threading.Lock()
exit_event = threading.Event()
setPoint = None
radius = 6  # mm
motor_id = 1

# 连接至DYNAMIXEL并清零
while True:#Conecting Loop
    motor.serial_port.port = '/dev/ttyUSB0'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 10
    try:
        motor.serial_port.open()
        break
    except:
        print('串口错误！')
print('连接成功！')
time.sleep(2)
print("舵机清零中...")
motor.position_mode()
motor.move_pos(1024)
print('清零完成，准备就绪，开始')

setSetPoint_thread = threading.Thread(target=setSetPoint)
setSetPoint_thread.start()

driveSteering_thread = threading.Thread(target=driveSteering)
driveSteering_thread.start()

distance_meansure_thread = threading.Thread(target=distanceMeansure)
distance_meansure_thread.start()

setSetPoint_thread.join()
driveSteering_thread.join()
distance_meansure_thread.join()
