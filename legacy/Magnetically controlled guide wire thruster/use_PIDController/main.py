import threading
import queue
import time
import PID
import motor
import socket
import toolfunc as tf
import RPi.GPIO as GPIO
import serial
import encoder_func
import stepper_func
import steering_func


# 舵机控制线程
def steering_control(target_queue_steering, event):
    """
    舵机控制线程
    Args:
        target_queue_steering: 存储目标位置的队列，来自上位机
    """
    global spi
    error = None
    flag_of_PID = True
    radiusSteering = 6  # mm
    radiusEncoder = ((22.76+22.77+22.74)/3)/2-0.4 # mm
    motor.extended_position_mode()
    time.sleep(2)
    #pid_controller = PID.PIDController(Kp=0.6, Ki=0.00001, Kd=0.1)  # 使用此参数无超调量
    pid_controller = PID.PIDController(Kp=0.99, Ki=0.0001, Kd=0.005)  # 使用此参数无超调量
    #spi.open(0,0)
    #spi.max_speed_hz = 9600
    last_target = -1  # 电机的最新目标位置
    print_flag = True
    error = None
    while not event.is_set():
        # 从队列中获取最新的目标位置
        try:
            new_target = target_queue_steering.get_nowait()  # 非阻塞获取
            if new_target is not None:
                last_target = new_target
                #print("进动目标位置 --------> ", last_target, "\n")
        except queue.Empty:
            if last_target == -1:
                if print_flag:
                    print("目标位置未设置")
                    print_flag = False
                continue
            pass
        print_flag = True
        time.sleep(0.05)
        error = float(last_target) - float(encoder_func.QUERY_ANGLE(ser))/360*2*3.1415926*radiusEncoder
        #error = float(last_target) - ((motor.read_position()/4096)*2*3.1415926*radius) # - 0.1*random.randint(0, 10 if error is None else round(10*error)))  # TODO: 测试代码，需要传感器运行时解除上一行的注释
        measure_precession = encoder_func.QUERY_ANGLE(ser)/360*2*3.1415926*radiusEncoder
        print("导丝位移距离 --------> ", measure_precession, " mm")
        #measure_precession = ((motor.read_position()/4096)*2*3.1415926*radius) # TODO: 测试代码，需要传感器运行时解除上2行的注释
        #print("导丝位移距离 --------> ", measure_precession, " mm")  # TODO: 测试代码，需要传感器运行时解除上2行的注释
        if flag_of_PID is True:
            output = pid_controller.calculate(error)
        else:
            output = 0
        rotation = 4096 * (output / (2 * 3.1415926 * radiusSteering))
        motor.move_pos(-(-motor.read_position()+round(rotation)))
        theoretical_precession = -((motor.read_position())/4096)*2*3.1415926*radiusSteering
        print("舵机位置    --------> ", theoretical_precession)
        with lock:
            array_theoretical_precession.append(theoretical_precession)
            array_measure_precession.append(measure_precession)
            array_setpoint.append(last_target)
            if len(array_theoretical_precession) > 10000:
                array_theoretical_precession.pop(0)
                array_measure_precession.pop(0)
        flag_of_PID = True
        if abs(error) < 0.1:
            print(f"舵机已经到达 --------> {last_target} mm")
            flag_of_PID = False
        #time.sleep(0.05)


def stepper_control(target_queue_stepper, event):
    """
    步进电机控制线程
    Args:
        target_queue_stepper: 存储步进电机目标位置的队列，来自上位机
    """
    # 定义引脚
    PUL = 29  # 步进脉冲
    DIR = 31  # 步进方向
    GPIO.setwarnings(False)
    # 设置GPIO模式
    GPIO.setmode(GPIO.BOARD)
    # 设置引脚为输出
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)

    last_target = None  # 电机的最新目标位置
    print_flag = True
    pulse_num = 0
    while not event.is_set():
        goal_pulse_num = None
        while pulse_num != goal_pulse_num or goal_pulse_num is None:
            try:
                new_target = target_queue_stepper.get_nowait()  # 非阻塞获取
                if new_target is not None:
                    last_target = new_target
                    #print("旋转目标位置 --------> ", last_target, "\n")
            except queue.Empty:
                if last_target is None:
                    if print_flag:
                        print("旋转目标位置未设置")
                        stepper_func.stop_motor(DIR, PUL)
                        print_flag = False
                    continue
                pass
            print_flag = True
            goal_pulse_num = stepper_func.get_pulse_number(last_target, 6400)
            if goal_pulse_num - pulse_num > 0:
                stepper_func.set_motor_forward(DIR, PUL)
                pulse_num += 1
            elif goal_pulse_num - pulse_num < 0:
                stepper_func.set_motor_reverse(DIR, PUL)
                pulse_num -= 1
            else:
                stepper_func.stop_motor(DIR, PUL)


def read_target_from_client(target_queue_steering, target_queue_stepper, client_socket):
    """
    从客户端读取目标位置
    Args:
        target_queue_steering: 存储舵机目标位置的队列，来自上位机
        target_queue_stepper: 存储步进电机目标位置的队列，来自上位机
        client_socket: 上位机socket
    """
    try:
        data = client_socket.recv(1024).decode().strip()  # 从客户端接收目标位置，并移除可能存在的空白字符
        if data:
            targets = data.split(',')
            if len(targets) == 2:
                if tf.is_numeric(targets[0]):
                    target_queue_steering.put(float(targets[0]))
                if tf.is_numeric(targets[1]):
                    target_queue_stepper.put(float(targets[1]))
        print("接收到目标位点 --------> ", targets[0], ", ", targets[1])
    except ConnectionResetError:
        # 处理客户端意外断开连接的情况
        print("客户端连接已断开")
    finally:
        # 确保客户端socket在函数结束时关闭
        client_socket.close()


if __name__ == "__main__":
    # ==================初始化设备==================
    steering_func.Initialize()  # 初始化舵机
    # 初始化server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    port = 10065
    tf.find_and_kill_process_on_port(port)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', port))  # 绑定到所有可用的接口，端口为12345
    server_socket.listen(5)  # 最多允许5个连接等待
    print("执行服务器正在监听...")
    # 编码器初始化
    # 配置串口参数，根据你的设备和需要进行调整
    SERIAL_PORT = '/dev/ttySC1'  # 树莓派上的串口设备路径
    BAUDRATE = 38400          # 波特率，应与编码器设置匹配
    TIMEOUT = 3                   # 读取超时时间
    # 创建串口对象
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    # if ser.is_open:
    #     ser.close()
    if not ser.isOpen():
        ser.open()
    time.sleep(0.5)
    encoder_func.SET_BAUDRATE('38400', ser)
    time.sleep(0.5)
    encoder_func.SET_ZERO_POSITION(ser)
    time.sleep(0.5)
    encoder_func.SET_DIRECTION('anticlockwise', ser)

    #==================初始化变量==================
    target_queue_steering = queue.Queue()  # 创建一个队列来共享目标位置
    target_queue_stepper = queue.Queue()  # 创建一个队列来共享目标位置

    array_theoretical_precession = []
    array_measure_precession = []
    array_setpoint = []

    event = threading.Event()
    lock = threading.Lock()

    # ==================创建线程==================
    steering_thread = threading.Thread(target=steering_control, args=(target_queue_steering, event,))
    Stepper_thread = threading.Thread(target=stepper_control, args=(target_queue_stepper, event,))

    steering_thread.daemon = True
    Stepper_thread.daemon = True

    steering_thread.start()
    Stepper_thread.start()

    try:
        while True:
            try:
                client_socket, addr = server_socket.accept()  # 接受连接
                # print(f"\n连接来自 --------> {addr}")
                # 为每个客户端连接创建一个新的线程来处理数据
                keyboard_thread = threading.Thread(target=read_target_from_client, args=(target_queue_steering, target_queue_stepper, client_socket))
                keyboard_thread.daemon = True  # 设置为守护线程
                keyboard_thread.start()
            except KeyboardInterrupt:
                print("\n捕获到结束信号...")
                break  # 退出循环
    finally:
        event.set()
        # 退出舵机线程
        steering_thread.join()
        print("正在退出舵机线程...")
        if not steering_thread.is_alive():
            print("舵机线程已退出")
        else:
            print("舵机线程退出失败")
        # 退出步进电机线程
        Stepper_thread.join()
        print("正在退出步进电机线程...")
        if not Stepper_thread.is_alive():
            print("步进电机线程已退出")
        else:
            print("步进电机线程退出失败")
        # 确保在程序退出时关闭服务器socket
        server_socket.close()
        print("正在关闭服务器socket...")
        if server_socket.fileno() == -1: # 检查socket是否已关闭
            print("服务器socket已关闭")
        else:
            print("服务器socket关闭失败")
        # 确保在程序退出时关闭串口
        ser.close()
        print("正在关闭串口...")
        if not ser.isOpen():
            print("串口已关闭")
        else:
            print("串口关闭失败")
        # 在循环退出后执行plot_data
        tf.plot_data(array_theoretical_precession, array_measure_precession, array_setpoint)
        print("\n================\n程序退出\n================\n")

