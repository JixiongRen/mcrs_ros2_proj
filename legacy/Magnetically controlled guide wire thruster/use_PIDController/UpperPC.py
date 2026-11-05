# PC上的客户端代码
import socket

def send_data_to_pi():
    """
    发送数据到树莓派
    """
    while True:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('192.168.0.105', 10001))  # 连接到树莓派的服务器
        data = input('输入目标位置  --------> ')
        # if data.isdigit():
        data_to_send = data  # 要发送的数据
        client_socket.send(data_to_send.encode())  # 发送数据
        print('发送数据 --------> ', data_to_send)
        # else:
        #     print('输入错误, 重新输入')
        client_socket.close()  # 关闭连接


if __name__ == "__main__":
    send_data_to_pi()

