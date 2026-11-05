import threading
import queue
import time
import Steering
import spidev
import motor
import socket


# 舵机控制线程
def motor_control(target_queue):
    global spi
    radius = 6  # mm
    motor.extended_position_mode()
    time.sleep(2)
    pid_controller = Steering.PIDController(Kp=0.6, Ki=0.00001, Kd=0.1)  # 使用此参数无超调量
    spi.open(0,0)
    spi.max_speed_hz = 9600
    last_target = 0  # 电机的最新目标位置
    while True:
        # 从队列中获取最新的目标位置
        try:
            new_target = target_queue.get_nowait()  # 非阻塞获取
            if new_target is not None:
                last_target = new_target
        except queue.Empty:
            pass
        error = float(last_target) - Steering.distanceMeansure(0, spidev.SpiDev())
        print("导丝位移距离 --------> ", Steering.distanceMeansure(0, spidev.SpiDev()), " mm")
        output = pid_controller.calculate(error)
        rotation = 4096 * (output / (2 * 3.1415926 * radius))
        motor.move_pos(motor.read_position()+round(rotation))
        # error = float(last_target) - Steering.distanceMeansure(0, spidev.SpiDev())
        if abs(error) < 1:
        # 模拟电机动作，这里用简单的打印代替
            print(f"舵机已经到达 --------> {last_target} mm")
            print('----------------------------------\n')


def read_target_from_client(target_queue, client_socket):
    """
    从客户端读取目标位置
    target_queue: 用于存放目标位置的队列
    client_socket: 客户端socket
    """
    target = client_socket.recv(1024).decode()  # 从客户端接收目标位置
    if target.isdigit():
        target_queue.put(int(target))  # 将目标位置放入队列


def Initialize():
    """
    舵机初始化函数
    """
    change_port_flag = False  # 用于切换串口的标志
    # 连接至DYNAMIXEL并清零
    while True:
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
    print("舵机清零中...")
    motor.position_mode()
    motor.move_pos(0)
    print('清零完成，准备就绪，开始')


if __name__ == "__main__":
    # ==================初始化设备==================
    Initialize()  # 初始化舵机
    # 初始化server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))  # 绑定到所有可用的接口，端口为12345
    server_socket.listen(5)  # 最多允许5个连接等待
    print("树莓派服务器正在监听...")
    client_socket, addr = server_socket.accept()  # 接受连接
    print(f"连接来自 {addr}")

    #==================初始化变量==================
    target_queue = queue.Queue()  # 创建一个队列来共享目标位置
    spi = spidev.SpiDev()  # 创建spi对象

    # ==================创建线程==================
    motor_thread = threading.Thread(target=motor_control, args=(target_queue,))
    motor_thread.daemon = True  # 设置为守护线程，这样主程序退出时它也会退出
    motor_thread.start()
    try:
        while True:
            client_socket, addr = server_socket.accept()  # 接受连接
            keyboard_thread = threading.Thread(target=read_target_from_client, args=(target_queue,client_socket,))
            keyboard_thread.daemon = True  # 设置为守护线程
            keyboard_thread.start()
    except KeyboardInterrupt:
        print("\n================\n程序退出\n================\n")