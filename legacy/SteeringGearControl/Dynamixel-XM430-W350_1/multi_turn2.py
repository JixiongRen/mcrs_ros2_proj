import motor
import time
while True:#Conecting Loop

    #motor.serial_port.port = 'COM6'
    motor.serial_port.baudrate = 57600
    motor.serial_port.timeout = 1
    try:
        motor.serial_port.open()
        break
    except:
        print('Serial error')



print('connected')
motor.reboot(1)
time.sleep(1)

print('start')


motor.motor_id = 1
motor.set_homing_offset(motor.most_left_pos)
motor.extended_position_mode()




motor.LED_control('on')
firmware_version = motor.read_int(motor.control_dict['version_of_firmware'],1)
print('firmware_version = %d'%firmware_version)
motor.send_profile_rpm(60)
print('homing...')
motor.move_pos(motor.most_left_pos)
time.sleep(1)
print('moving...')
print(motor.read_position())
motor.send_goal_pos(motor.most_left_pos+4095*15)


while True:
    time.sleep(1)
    print(motor.read_position())




