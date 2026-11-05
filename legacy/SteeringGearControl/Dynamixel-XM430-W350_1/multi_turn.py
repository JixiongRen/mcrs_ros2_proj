import motor
import time
while True:#Conecting Loop

    motor.serial_port.port = 'COM8'
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
motor.extended_position_mode()




motor.LED_control('on')
time.sleep(1)
firmware_version = motor.read_int(motor.control_dict['version_of_firmware'],1)
print('firmware_version = %d'%firmware_version)
motor.send_profile_rpm(60)
while True:

    motor.LED_control(0)
    motor.send_goal_degree(90+360*2)
    time.sleep(10)
    motor.LED_control( 1)
    motor.send_goal_degree(90-360*2)
    time.sleep(10)





