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
    print(data)
    print(received_crc)

    # 计算有效数据的CRC值
    calculated_crc = crc_count(data, len(data))
    # 将计算出的CRC值转换为字节表示，以便与接收到的CRC值进行比较
    calculated_crc_bytes = calculated_crc.to_bytes((calculated_crc.bit_length() + 7) // 8, 'little')
    print(calculated_crc_bytes)
    # 比较计算出的CRC值和接收到的CRC值
    if calculated_crc_bytes == received_crc:
        print("CRC校验成功，数据帧有效。")
    else:
        print("CRC校验失败，数据帧可能已损坏。")



# # 示例数据帧，包括最后4位的CRC校验码
# data_frame = bytes.fromhex('01 06 00 08 00 01 C9 C8')
# # 执行CRC校验
# crc_check(data_frame)

array = bytes.fromhex('01 03 04 00 01 76 3B CC 40')
crc = crc_count(array, len(array))
print(crc)
crc_byte =  crc.to_bytes((crc.bit_length() + 7) // 8, 'little')
print(crc_byte)
array_with_crc = array + crc_byte
print(array_with_crc)
crc_check(array)

# 原始16进制数据
hex_data = b'\x01\x03\x04\x00\x01v;\xcc@'

# 读取第三字节作为n的值（注意Python中索引从0开始）
n = hex_data[2]  # 索引2对应第三字节

# 将n从字节转换为整数
n = int.from_bytes([n], byteorder='little')

# 计算开始位置（基于0的索引，所以第四字节的位置是3）
start_index = 3

# 提取从第四字节开始的n个字节
angle_bytes = hex_data[start_index:start_index + n]

# 如果需要，可以将字节串转换回16进制字符串或整数等
angle_hex_str = angle_bytes.hex()
angle_int = int.from_bytes(angle_bytes, byteorder='big')

# 将结果存储在变量angle中
angle = angle_int # 或者 angle_hex_str 或 angle_int，取决于您想要的数据类型

# 打印结果
print("n (read from third byte):", n)
print("Extracted bytes:", angle_bytes)
print("As hex string:", angle_hex_str)
print("As integer:", angle_int)
