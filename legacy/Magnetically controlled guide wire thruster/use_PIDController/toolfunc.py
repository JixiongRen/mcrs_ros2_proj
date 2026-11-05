import motor
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
import socket
import subprocess
import platform
import os

def is_numeric(s):
    """
    判断输入字符串是否为数字
    Args:
        s: 待判断的字符串
    Returns: 判断结果
    """
    if s != '':
        if s[0] == '-' and s[1:].replace('.', '', 1).isdigit():
            return True
        elif s.replace('.', '', 1).isdigit():
            return True
        return False
    return False



def plot_data(array_theoretical_precession, array_measure_precession, array_setpoint):
    """
    绘图函数
    Args:
        array_theoretical_precession: 磁控导丝理论位移序列
        array_measure_precession: 传感测量的导丝位移序列
        array_setpoint: 目标点序列
    """
    plt.figure()
    plt.plot(array_theoretical_precession, label='theoretical precession')
    plt.plot(array_measure_precession, label='measure precession')
    plt.plot(array_setpoint, label='set point', linestyle='--', color='green')
    plt.legend()
    plt.title('Comparison between theoretical and measure precession distance')
    plt.xlabel('Times / 1')
    plt.ylabel('Precession / mm')
    plt.show()
    plt.savefig('precession.png')


def find_and_kill_process_on_port(port):
    """
    查找并关闭指定端口上的进程
    Args:
        port: 需要关闭的端口号
    Returns:
    """
    # 创建一个socket对象
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 尝试连接到指定的端口
    try:
        s.connect(('localhost', port))
    except ConnectionRefusedError:
        print(f"端口 {port} 未被占用")
        return
    # 获取当前操作系统的信息
    system = platform.system()

    # 获取占用端口的进程ID
    if system == 'Windows':
        # Windows下使用netstat命令查找进程ID
        result = subprocess.run(['netstat', '-ano', '|', 'findstr', f':{port}'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        lines = result.stdout.split('\n')
        for line in lines:
            if line:
                pid = line.split()[-1]
                break
    elif system == 'Linux' or system == 'Darwin':
        # Linux和macOS下使用lsoft命令查找进程ID
        result = subprocess.run(['lsoft', '-i', f':{port}'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        lines = result.stdout.split('\n')
        for line in lines:
            if 'LISTEN' in line:
                pid = line.split()[1]
                break
    else:
        print(f"不支持的操作系统: {system}")
        return

        # 如果找到了进程ID，尝试关闭它
    if pid:
        try:
            # 使用taskkill命令关闭Windows上的进程
            if system == 'Windows':
                subprocess.run(['taskkill', '/F', '/PID', pid], check=True)
                # 使用kill命令关闭Linux或macOS上的进程
            elif system == 'Linux' or system == 'Darwin':
                subprocess.run(['kill', '-9', pid], check=True)
            print(f"在端口 {port} 上的进程 (PID: {pid}) 已被停止")
        except subprocess.CalledProcessError:
            print(f"消灭端口 {port} 上的进程 (PID: {pid}) 失败")
    else:
        print(f"未在端口 {port} 上找到进程")
        # 关闭socket对象
    s.close()
