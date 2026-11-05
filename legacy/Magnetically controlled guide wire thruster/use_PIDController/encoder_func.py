import time


def SET_BAUDRATE(buadrate, ser):
    """
    设置编码器的波特率
    Args:
        MODE: 模式 0:9600 1: 19200 2: 38400 3: 57600 4: 115200
        ser: 串口对象
    """
    baudrate_dic = {
        '9600'  : '00 00',
        '19200' : '00 01',
        '38400' : '00 02',
        '57600' : '00 03',
        '115200': '00 04'
    }
    com_without_crc = bytes.fromhex('01 06 00 05 ' + baudrate_dic[buadrate])
    crc = crc_count(com_without_crc, len(com_without_crc))
    send_comm = com_without_crc + crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            print('已设置波特率为 '+ str(buadrate))
    else:
        print('错误，未收到数据')

def SET_ZERO_POSITION(ser):
    """
    设置编码器的零位
    Args:
        ser: 串口对象
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

def SET_DIRECTION(direction, ser):
    """
    设置编码器的方向
    Args:
        direction: 方向 clockwise: 顺时针 anticlockwise: 逆时针
        ser: 串口对象
    """
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

def QUERY_ANGLE(ser):
    """
    查询编码器的角度
    Args:
        ser: 串口对象
    Returns: 编码器角度信息
    """
    com_without_crc = bytes.fromhex('01 03 00 00 00 02')
    crc = crc_count(com_without_crc, len(com_without_crc))
    send_comm = com_without_crc + crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
    ser.write(send_comm)
    time.sleep(0.2)
    len_recv_data = ser.in_waiting # 获取接收缓冲区中的字节数
    if len_recv_data:
        recv_data = ser.read(len_recv_data)
        if crc_check(recv_data):
            # 原始16进制数据
            hex_data = recv_data
            # 读取第三字节作为n的值
            n = hex_data[2]  # 索引2对应第三字节
            # 将n从字节转换为整数
            n = int.from_bytes([n], byteorder='little')
            # 计算开始位置
            start_index = 3
            # 提取从第四字节开始的n个字节
            pos_bytes = hex_data[start_index:start_index + n]
            pos_int = int.from_bytes(pos_bytes, byteorder='big')
            # 如果pos_int大于900*4096，说明是负数，需要减去1800圈
            if pos_int > 900*4096:
                pos_int = pos_int - 4096*1800
            # 将结果存储在变量angle中
            angle = float(pos_int*360/4096)
            return float(angle)
            # print(angle)
    else:
        print('错误，未收到数据')

def crc_count(pbuf, num):
    """
    计算CRC校验码
    Args:
        pbuf: 不含校验码的缓冲区数据
        num: pbuf的长度
    Returns: 计算得到的CRC校验码
    """
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
    """
    CRC校验方法
    Args:
        data_with_crc: 含有CRC校验码的数据
        crc_length: CRC校验码的预计长度
    Returns: CRC校验结果
    """
    # 分离出有效数据和CRC校验码
    data = data_with_crc[:-crc_length]
    received_crc = data_with_crc[-crc_length:]
    # 计算有效数据的CRC值
    calculated_crc = crc_count(data, len(data))
    # 将计算出的CRC值转换为字节表示，以便与接收到的CRC值进行比较
    calculated_crc_bytes = calculated_crc.to_bytes((calculated_crc.bit_length() + 7) // 8, 'little')
    # 比较计算出的CRC值和接收到的CRC值
    if calculated_crc_bytes == received_crc:
        crc_flag = True
        print("CRC校验成功，数据帧有效")
    else:
        crc_flag = False
        print("CRC校验失败，数据帧可能已损坏")
    return crc_flag