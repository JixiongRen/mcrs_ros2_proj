import serial
import time

# 配置串口参数，根据你的设备和需要进行调整
SERIAL_PORT = '/dev/ttySC1'  # 树莓派上的串口设备路径
BAUDRATE = 38400          # 波特率，应与编码器设置匹配
TIMEOUT = 1                   # 读取超时时间

# 创建串口对象
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)

def SET_BAUDRATE(MODE):
    """
    设置编码器的波特率
    Args:
        MODE: 模式 0:9600 1: 19200 2: 38400 3: 57600 4: 115200
    """
    send_comm = bytes.fromhex('01 06 00 05 00 0' + str(MODE) + ' 18 0A')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            print('已设置波特率为模式'+ str(MODE))
    else:
        print('错误，未收到数据')

def SET_ZERO_POSITION():
    """
    设置编码器的零位
    """
    com_without_crc = bytes.fromhex('01 06 00 08 00 01')
    crc = crc_count(com_without_crc, len(com_without_crc))
    send_comm = com_without_crc + crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            print('已将零位设置为当前位置')
    else:
        print('错误，未收到数据')

def SET_DIRECTION(direction):
    dir_dic = {
        'clockwise': '00 00',
        'anticlockwise': '00 01'
    }
    com_without_crc = bytes.fromhex('01 06 00 09 ' + dir_dic[direction])
    crc = crc_count(com_without_crc, len(com_without_crc))
    send_comm = com_without_crc + crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            print('设置方向为 ' + direction)
    else:
        print('错误，未收到数据')

def QUERY_ANGLE():
    com_without_crc = bytes.fromhex('01 03 00 00 00 02')
    crc = crc_count(com_without_crc, len(com_without_crc))
    send_comm = com_without_crc + crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            # 提取recv_data的正数第7位14位的数据
            angle = int.from_bytes(recv_data[3:5], byteorder='big')
    else:
        print('错误，未收到数据')

def crc_count(pbuf, num):
    wcrc = 0xFFFF
    for i in range(num):
        wcrc ^= pbuf[i]
        for j in range(8):
            if wcrc & 0x0001:
                wcrc >>= 1
                wcrc ^= 0xA001
            else:
                wcrc >>= 1
    return wcrc

def crc_check(data_with_crc, crc_length=2):
    # 分离出有效数据和CRC校验码
    data = data_with_crc[:-crc_length]
    # data = [0x01, 0x06, 0x00, 0x08, 0x00, 0x01]
    received_crc = data_with_crc[-crc_length:]
    #print(data)
    #print(received_crc)
    # 计算有效数据的CRC值
    calculated_crc = crc_count(data, len(data))
    # 将计算出的CRC值转换为字节表示，以便与接收到的CRC值进行比较
    calculated_crc_bytes = calculated_crc.to_bytes((calculated_crc.bit_length() + 7) // 8, 'little')
    #print(calculated_crc_bytes)
    # 比较计算出的CRC值和接收到的CRC值
    if calculated_crc_bytes == received_crc:
        crc_flag = True
        print("CRC校验成功，数据帧有效。")
    else:
        crc_flag = False
        print("CRC校验失败，数据帧可能已损坏。")
    return crc_flag

if ser.is_open:
    ser.close()
ser.open()
time.sleep(0.1)
if ser.isOpen():
    SET_ZERO_POSITION()
    time.sleep(0.5)
    SET_DIRECTION('clockwise')
    time.sleep(0.5)
    while True:
        QUERY_ANGLE()