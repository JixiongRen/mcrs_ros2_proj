import socket
import queue
import threading
import toolfunc as tf

def read_target_from_client(target_queue_steering, target_queue_stepper, client_socket):
    """
    从客户端读取目标位置
    target_queue: 用于存放目标位置的队列
    client_socket: 客户端socket
    """
    try:
        data = client_socket.recv(1024).decode().strip()  # 从客户端接收目标位置，并移除可能存在的空白字符
        if data:
            targets = data.split(',')
            if len(targets) == 2:
                print(targets[0])
                if tf.is_numeric(targets[0]):
                    target_queue_steering.put(int(targets[0]))
                    print("接收到进退目标位点 --------> ", targets[0])
                else:
                    print("未能接收到进退目标位点")
                if tf.is_numeric(targets[1]):
                    target_queue_stepper.put(int(targets[1]))
                    print("接收到旋转目标位点 --------> ", targets[1])
                else:
                    print("未能接收到旋转目标位点")
    except ConnectionResetError:
        # 处理客户端意外断开连接的情况
        print("客户端连接已断开")
    finally:
        # 确保客户端socket在函数结束时关闭
        client_socket.close()

if __name__ == "__main__":
    target_queue_steering = queue.Queue()  # 创建一个队列来共享目标位置
    target_queue_stepper = queue.Queue()  # 创建一个队列来共享目标位置


    # ==================初始化设备==================
    # 初始化server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 10005))  # 绑定到所有可用的接口，端口为12345
    server_socket.listen(5)  # 最多允许5个连接等待
    print("执行服务器正在监听...")

    try:
        while True:
            client_socket, addr = server_socket.accept()  # 接受连接
            print(f"连接来自 --------> {addr}")
            # 为每个客户端连接创建一个新的线程来处理数据
            keyboard_thread = threading.Thread(target=read_target_from_client, args=(target_queue_steering, target_queue_stepper, client_socket))
            keyboard_thread.daemon = True  # 设置为守护线程
            keyboard_thread.start()
    except KeyboardInterrupt:
        print("\n================\n程序退出\n================\n")
    finally:
        # 确保在程序退出时关闭服务器socket
        server_socket.close()