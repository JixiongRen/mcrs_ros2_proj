import motor
def Initialize():
    """
    舵机初始化函数
    """
    change_port_flag = False  # 用于切换串口的标志
    # 连接至DYNAMIXEL并清零
    while True:
        if change_port_flag:
            motor.serial_port.port = '/dev/ttyUSB0'
        else:
            motor.serial_port.port = '/dev/ttyUSB1'
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
    motor.send_goal_RPM(100)
    motor.move_pos(0)
    print('清零完成，准备就绪，开始')