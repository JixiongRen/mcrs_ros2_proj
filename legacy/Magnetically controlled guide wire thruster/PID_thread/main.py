import motor
import spidev
import Steering
import threading
import time
import queue

event = threading.Event()
lock = threading.Lock()
spi = spidev.SpiDev()
setpoint = None

def thread_steering():
    global setpoint
    global spi
    radius = 6  # mm
    motor.extended_position_mode()
    time.sleep(2)
    pid_controller = Steering.PIDController(Kp=0.6, Ki=0.00001, Kd=0.1)
    spi.open(0,0)
    spi.max_speed_hz = 9600

    if setpoint is None:
        setpoint = 0

    while True:
        with lock:
            error = float(setpoint) - Steering.distanceMeansure(0, spidev.SpiDev())
            while abs(error) > 1:
                print("导丝位移距离: ", Steering.distanceMeansure(0, spidev.SpiDev()))
                output = pid_controller.calculate(error)
                # print("output: ", output)
                rotation = 4096 * (output / (2 * 3.1415926 * radius))
                # print("舵机当前位置: ", motor.read_position())
                motor.move_pos(motor.read_position()+round(rotation))
                error = float(setpoint) - Steering.distanceMeansure(0, spidev.SpiDev())
            print("舵机已到达指定位置")
        event.set()


def thread_getinput():
    global setpoint
    while True:
        with lock:
            setpoint = input("请设置 setPoint: ")
        event.wait()


change_port_flag = False
# 连接至DYNAMIXEL并清零
while True:#Conecting Loop
    if change_port_flag:
        motor.serial_port.port = '/dev/ttyUSB1'
    motor.serial_port.port = '/dev/ttyUSB0'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 10
    try:
        motor.serial_port.open()
        break
    except:
        change_port_flag = True
        print('串口错误！')
print('连接成功！')
time.sleep(2)
print("舵机清零中...")
motor.position_mode()
motor.move_pos(0)
print('清零完成，准备就绪，开始')

# 初始化setpoint
setpoint = 0

threading.Thread(target=thread_getinput).start()
threading.Thread(target=thread_steering).start()