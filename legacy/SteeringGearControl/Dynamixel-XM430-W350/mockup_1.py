import motor
import time
import atexit

def exit_handler():

    motor.motor_off()
    print('Stop program..')

atexit.register(exit_handler)
while True:#Conecting Loop


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
motor.set_homing_offset(0)
motor.extended_position_mode()




motor.LED_control('on')
firmware_version = motor.read_int(motor.control_dict['version_of_firmware'],1)
print('firmware_version = %d'%firmware_version)
motor.send_profile_rpm(60)
print('homing...')

offset = motor.read_position()
goal_pos = offset-4096*22
#goal_pos = offset+26175
motor.send_goal_pos(goal_pos)


while True:
    current = motor.read_position()-offset
    print('turn = ',current/4096,'value = ',current)

    time.sleep(0.5)




