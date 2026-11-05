import motor
import time
while True:#Conecting Loop

    #motor.serial_port.port = '/dev/cu.wchusbserial1410'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('Serial error')



print('connected')
time.sleep(2)

print('start')


#default velocity limite is 350 or 80.15RPM
#reboot(1)
motor.motor_id = 1
motor.position_mode()




#motor.LED_control('on')
#time.sleep(1)
firmware_version = motor.read_int(motor.control_dict['version_of_firmware'],1)
print('firmware_version = %d'%firmware_version)
motor.send_profile_rpm(30)
print(motor.read_uint(9,1))
while True:

    motor.LED_control(0)
    motor.move_pos(0)
    time.sleep(0.5)
    motor.LED_control(1)
    motor.move_pos(4095)
    time.sleep(0.5)





