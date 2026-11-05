import serial
import time
import serial.tools.list_ports

serial_port =  serial.Serial()
serial_port.port = '/dev/cu.wchusbserial1410'
motor_id = 1
most_left_pos = -1044479
most_right_pos = 1044479
#######Protocol section#######
operating_mode_dict = {
    'current_control':b'\x00',
    'velovity_control':b'\x01',
    'position_control':b'\x03',
    'extended_position_control':b'\x04',
    'current_besed_position_control':b'\x05',
    'pwm_control':b'\x10'
}
inst_dict = {
    'ping':b'\x01',
    'read':b'\x02',
    'write':b'\x03',
    'reg_write':b'\x04',
    'action':b'\x05',
    'factory_reset':b'\x06',
    'reboot':b'\x08',
    'clear':b'\x10',
    'status':b'\x55',
    'sync_read':b'\x82',
    'sync_write':b'\x83',
    'bulk_read':b'\x92',
    'bulk_write':b'\x93'
}
control_dict = {

    'led' : 65,
    'goal_velocity' : 104,
    'present_velocity' : 128,
    'present_position' : 132,
    'homing_offset' : 20,

    'goal_pos' : 116,
    'torque_enable' : 64,
    'operating_mode' : 11,
    'present_current' : 126,
    'version_of_firmware' : 6,
    'profile_velocity' : 112
}
def hex_print (b_data):
    print(' '.join([hex(b) for b in b_data]))

def update_crc( crc_accum, pack):
    crc_table = [
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    ]
    pack_len =  (pack[6]<<8)| pack[5]
    for j in range(5+pack_len):
        i = ((crc_accum >> 8) ^ pack[j]) & 0xFF
        crc_accum = ((crc_accum << 8)&0xFFFF) ^ crc_table[i]
    return bytes([crc_accum&0xFF]) + bytes([crc_accum >> 8])
    #return (crc_accum>>8),(crc_accum&0xFF)

    #return bytes([crc_accum])

def packet_builder(id, inst, param):
    if type(id) is int:
        id = bytes([id])
    packet_header = b'\xff\xff\xfd\x00'
    length = len(param) + 3
    lenght_1 = bytes([length & 0xff])
    lenght_2 = bytes([(length >> 8) & 0xff])
    msg = packet_header + id+lenght_1 + lenght_2 + inst + param

    packet = msg + update_crc(0, msg)
    return packet

def transceive_packer(send_packet):
    #serial_port.flushInput()
    #time.sleep(0.1)
    serial_port.write(send_packet)
    recieve_data = serial_port.read(7)
    try:
        length = recieve_data[5] |(recieve_data[6]<<8)
        recieve_data += serial_port.read(length)
    except  Exception as e:
        print('Serial Error,transceive_packet ,'+str(e))
        return None
    return recieve_data
def return_pack_analyze(pack):
    packet_header = b'\xff\xff\xfd\x00'
    if pack[0:4] != packet_header:
        print('Packet Header Error')
    print('ID = ',pack[4])
    length = pack[5] | (pack[6] << 8)

    if pack[7] == 0x55:
        print('INST = return pack')
    else:
        print('error ,INST  = ',hex(pack[7]))
    pack_crc = pack[length+7-2:length+7]
    print('ERR = ',pack[8])
    calc_crc = update_crc(0,pack)
    if pack_crc != calc_crc:
        print('CRC Error , pack crc = ',pack_crc,'cals crc = ',calc_crc)
def packet_error_check(pack):
    packet_header = b'\xff\xff\xfd\x00'
    if pack == None:
        print('err,packet_error_check ,pack == None')
        return True
    if pack[0:4] != packet_header:
        print('err,packet_error_check ,pack[0:4] != packet_header')
        return True

    length = pack[5] | (pack[6] << 8)


    pack_crc = pack[length+7-2:length+7]
    err = pack[8]
    if err != 0:
        print('err,packet_error_check ,recieved err status')
        return True
    calc_crc = update_crc(0,pack)
    if pack_crc != calc_crc:
        print('CRC Error , pack crc = ',pack_crc,'cals crc = ',calc_crc)
        return True
    return False
def send_write(id,addr,data):

    addr_L = bytes([addr & 0xff])
    addr_H = bytes([(addr >> 8) & 0xff])
    pack = packet_builder(id=id, inst=inst_dict['write'], param=addr_L + addr_H + data)
    return transceive_packer(pack)
def reboot(id):


    pack = packet_builder(id=id, inst=inst_dict['reboot'], param=b'')
    return transceive_packer(pack)
def send_read(id,addr,length):

    addr_L = bytes([addr & 0xff])
    addr_H = bytes([(addr >> 8) & 0xff])
    length_L = bytes([length & 0xff])
    length_H = bytes([(length >> 8) & 0xff])
    pack = packet_builder(id=id, inst=inst_dict['read'], param=addr_L + addr_H + length_L+length_H)
    return transceive_packer(pack)

### Wrap up section ###
def read_reg(addr,length):
    status_pack = send_read(motor_id,addr,length)
    if packet_error_check(status_pack) == True:
        print('err,read_reg')
        print(status_pack)
        return None
    else:



        return status_pack[9:9+length]

def read_int(addr,length,retry = 0):
    data = read_reg(addr,length)
    if data == None:
        if retry>3:
            return None
        else:
            print('err,read_int, retry = ',retry)
            time.sleep(0.5)
            return read_int(addr,length,retry+1)
    else:
        return int.from_bytes(data, byteorder='little', signed=True)
def read_uint(addr,length,retry = 0):
    data = read_reg(addr,length)
    if data == None:
        if retry>3:
            return None
        else:
            print('err,read_uint, retry = ',retry)
            time.sleep(0.5)
            return read_uint(addr,length,retry+1)
    else:
        return int.from_bytes(data, byteorder='little', signed=False)
def LED_control(value):
    id = motor_id
    if type(id) is int:
        id = bytes([id])
    if type(value) is str:
        if value == 'on':
            value = b'\x01'
        else:
            value = b'\x00'
    elif type(value) is int:
        value = bytes([value])
    addr_L = bytes([control_dict['led'] & 0xff])
    addr_H = bytes([(control_dict['led'] >> 8) & 0xff])


    return  send_write(id,control_dict['led'],value)
def send_goal_pos(value):
    id = motor_id
    if type(id) is int:
        id = bytes([id])

    if type(value) is int:
        b_value = b''
        for i in range(4):
            b_value += bytes([(value>>(8*i))&0xff])
        value = b_value
    return send_write(id, control_dict['goal_pos'], value)
def send_homing_offset(value):#need to turn off motor before send this command
    id = motor_id
    if type(id) is int:
        id = bytes([id])

    if type(value) is int:
        b_value = b''
        for i in range(4):
            b_value += bytes([(value>>(8*i))&0xff])
        value = b_value
    return send_write(id, control_dict['homing_offset'], value)
def set_homing_offset(value):#need to turn off motor before send this command
    motor_off()
    send_homing_offset(value)
def send_goal_velocity(value):
    id = motor_id
    if type(id) is int:
        id = bytes([id])

    if type(value) is int:

        b_value = b''
        for i in range(4):
            b_value += bytes([(value>>(8*i))&0xff])
        value = b_value
    return send_write(id, control_dict['goal_velocity'], value)


def send_goal_RPM(RPM):
    value = rpm_to_velocity(RPM)
    return send_goal_velocity(value)
def send_profile_velocity(value):
    id = motor_id
    if type(id) is int:
        id = bytes([id])

    if type(value) is int:

        b_value = b''
        for i in range(4):
            b_value += bytes([(value>>(8*i))&0xff])
        value = b_value
    return send_write(id, control_dict['profile_velocity'], value)

def send_profile_rpm(value):
    return send_profile_velocity(rpm_to_velocity(value))
def velocity_mode():
    send_write(motor_id, control_dict['torque_enable'], b'\x00')
    send_write(motor_id, control_dict['operating_mode'], operating_mode_dict['velovity_control'])
    send_write(motor_id, control_dict['torque_enable'], b'\x01')
def position_mode():
    send_write(motor_id, control_dict['torque_enable'], b'\x00')
    send_write(motor_id, control_dict['operating_mode'], operating_mode_dict['position_control'])
    send_write(motor_id, control_dict['torque_enable'], b'\x01')
def extended_position_mode():
    send_write(motor_id, control_dict['torque_enable'], b'\x00')
    send_write(motor_id, control_dict['operating_mode'], operating_mode_dict['extended_position_control'])
    send_write(motor_id, control_dict['torque_enable'], b'\x01')
def motor_off():
    send_write(motor_id, control_dict['torque_enable'], b'\x00')
def read_rpm():
    return velocity_to_rpm(read_int(control_dict['present_velocity'], 4))
def read_position():
    return read_int(control_dict['present_position'], 4)
def read_current():#raw
    value = read_int(control_dict['present_current'],2)
    if value == None:
        return None
    return value
def read_current_milliamp():#mAh
    value = read_int(control_dict['present_current'], 2)
    if value == None:
        return None
    return read_current()*2.69

def send_goal_degree(deg):
    send_goal_pos(degree_to_position(deg))
### conversion ###
def rpm_to_velocity(rpm):
    return int(rpm/0.229)
def velocity_to_rpm(vel):
    return vel*0.229
def degree_to_position(deg):
    return int(deg/0.088)
def move_pos(goal_pos):
    tolerant = 2
    send_goal_pos(goal_pos)
    crr_pos = read_position()
    if crr_pos == None:
        print('err,move_pos,1')
        return None
    while abs(crr_pos-goal_pos) > tolerant:
        crr_pos = read_position()
        if crr_pos == None:
            print('err,move_pos,2')
            return None
        #time.sleep(0.01)
