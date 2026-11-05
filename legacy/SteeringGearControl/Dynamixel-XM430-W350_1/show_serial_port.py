import serial
import time
import serial.tools.list_ports

serial_port =  serial.Serial()
port_list = [x.device for x in serial.tools.list_ports.comports(include_links=False)]
print('Select COM. port')
for p in range(len(port_list)):
    print('[%d] %s'%(p,port_list[p]))