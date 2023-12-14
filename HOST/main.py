import serial
import math
import time
import os

import cmsisdsp as dsp
import numpy as np
from scipy import signal

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

SOT='FFFFFFFF\n'
HDR='55555555\n'
EOT='AAAAAAAA\n'
datablock = []

BLOCKS_FOR_CHIP = 16384
mcu_port : serial.Serial = None
#  crc16("123456789".encode('UTF-8') should return 0xBB3D
def crc16(data: bytes, poly=0xa001, init_crc=0x0000, xor_out=0x0000):
    """
    CRC16 0x8005 (x^16 + x^15 + x^2 + 1)
    """
    data = bytearray(data)
    crc = init_crc
    for b in data:
        cur_byte = 0xFF & b
        for _ in range(0, 8):
            if (crc & 0x0001) ^ (cur_byte & 0x0001):
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            cur_byte >>= 1

    return (crc & 0xFFFF) ^ xor_out

#   init_crc = 0xffff --- CRC-16(modbus)
#   init_crc = 0      --- CRC-16
#   get_crc("123456789".encode('UTF-8') should return 0xBB3D for init_crc = 0 (CRC-16)
#   get_crc("123456789".encode('UTF-8') should return 0x4B37 for init_crc = 0xFFFF (CRC-16(modbus))
def get_crc(buf: bytes, poly=0xa001, init_crc=0) : # 0xffff):
    """
    """
    data = bytearray(buf)
    crc = init_crc
    for b in data:
        crc = crc ^ b
        for _ in range(0, 8):
            a = crc
            carry_flag = a & 0x0001
            crc = crc >> 1
            if 1 == carry_flag:
                crc = crc ^ poly
    return crc


def getStellarisData(dataport):
    datablock.clear()
    dataport.flushInput()
    dataport.write('S'.encode('utf-8'))
    while True :
        buff = dataport.readline().decode('utf-8')
        if SOT == buff:
            buff = dataport.readline().decode('utf-8')
            if HDR == buff:
                while EOT != buff:
                    buff = dataport.readline().decode('utf-8')
                    if EOT != buff:
                        datablock.append(buff)
                    else:
                        return len(datablock)

def processData():
    nb = len(datablock)
    rfftf32 = dsp.arm_rfft_fast_instance_f32()
    status = dsp.arm_rfft_fast_init_f32(rfftf32, nb)
    dtb = np.asarray(datablock, dtype=float)
    result = dsp.arm_rfft_fast_f32(rfftf32, dtb, 0)
    vres = np.arange((len(result)/2), dtype=float)
    vres[0:len(vres)] = 0
    #   prepare magnitude
    for i  in range(0, len(result), 2):
        vres[i // 2] = math.sqrt(result[i]*result[i]+result[i+1]*result[i+1])
    with open('stg.text', 'w') as f:
        for val in vres:
            f.write(str(val))
            f.write('\n')

    print(f'result length {len(result)}')

def readAsciiHexData(dataport):
    getStellarisData(dataport)
    print(f'datablock length {len(datablock)}')
    with open('stellaris_data.csv', 'w') as f:
        for s in datablock:
            f.write(s)
            f.flush()
    with open('stellaris_data.bin', 'wb') as g:
        for s in datablock:
            g.write(int(s).to_bytes(4, byteorder='big'))
            g.flush()

#
#  Request record - 'S' - give next record
#                   'Q' - stop the data transmission
#
#  record format :
#    ASCII HEX data - single line as HH HH HH HH ....
#
#  timeout, attempts
#
#   data will be placed to the datablock
#
BLOCKSIZE = 256
def receiveDataBlock(dataport) -> int :
    datablock.clear()
    dataport.flushInput()
    dataport.flush()
    dataport.write('S'.encode('utf-8'))
    buff = dataport.readline().split()
    bytes_received = 0
    for v in buff:
        datablock.append(int(v, 16))
    if (BLOCKSIZE+2) == len(datablock) :
        blockbuffer = bytes(datablock)
        vcrc = get_crc(blockbuffer[0:256])
        crcpattern = ((blockbuffer[-1] << 8) + blockbuffer[-2]) & 0xFFFF
        if vcrc != crcpattern :
            print(f'crc mismatch : vcrc = {vcrc:04X}  crcpattern = {crcpattern:04X}')
            return 0
        bytes_received = len(blockbuffer)-2
    return bytes_received

def readWholeMemory(dataport):
    memory_chip_data = []
    datablock.clear()
    dataport.flushInput()
    dataport.flushOutput()
    dataport.flush()
    # rewind to the first memory block
    dataport.write('C'.encode('utf-8'))
    is_record = True
    blocks_in = 0
    while is_record :
        if 0 != receiveDataBlock(dataport) :
            del(datablock[-1])
            del(datablock[-1])
            if BLOCKSIZE == len(datablock) :
                for v in datablock :
                    memory_chip_data.append(v)
            else :
                print(f'---> Strange block length : {len(datablock)}')
            blocks_in = blocks_in + 1
            if 0 == blocks_in % 100:
                print(f'blocks : {blocks_in}\r')
        else :
            is_record = False
    print("")
    print(f'---> Total bytes read : {len(memory_chip_data)}')
    with open('wt588d_data.bin', 'wb') as g:
        g.write(bytes(memory_chip_data))
        g.flush()

MAX_TIMEOUT = 2 # seconds
RECORD_LENGTH = 258 # 256 data bytes + 2 bytes CRC16

# receive fixed length record (data or answer to command)
#   host             client
#   'G'(ive) ->
#            <-       data block + 2 bytes CRC16
#   'ACK'    ->
#   'NAK'
#
# buffer size - 256 bytes or more
def receiveRecord(buffer : bytearray) -> bool :
    global mcu_port
    attempts = 5
    for i in range(0,256):
        buffer[i] = 0
    if mcu_port is not None and mcu_port.isOpen():
        while 0 < attempts:
            mcu_port.reset_input_buffer()
            mcu_port.reset_output_buffer()
            mcu_port.write('G'.encode('UTF-8'))
            t0 = time.time()
            bytesinbuffer = 0
            while RECORD_LENGTH > bytesinbuffer :
                bytesinbuffer = mcu_port.in_waiting
                if MAX_TIMEOUT < (time.time()-t0):
                    attempts -= 1
                    break
            if RECORD_LENGTH <= bytesinbuffer:
                tmp_buffer = mcu_port.read(RECORD_LENGTH)
                crcvalue = get_crc(tmp_buffer[0:256])
                crcpattern = ((tmp_buffer[-1] << 8) + tmp_buffer[-2]) & 0xFFFF
                if crcpattern != crcvalue:
                    print(f'crc mismatch : crcvalue = {crcvalue:04X}  crcpattern = {crcpattern:04X}')
                    attempts -= 1
                    mcu_port.write('NAK'.encode('UTF-8'))
                else:
                    for i in range(0,256):
                        buffer[i] = tmp_buffer[i]
                    mcu_port.write('ACK'.encode('UTF-8'))
                    return True
    return False

# send fixed length record (command or data)
#   host             client
#    'S'     ->
#            <-        'R' ready to receive
#                      'Q' quit receive
#  data block
#  + 2 bytes CRC16 ->
#                      'ACK' if record OK
#                      'NAK' if problem(s)
def sendRecord(cmdbuffer) -> bool :
    global mcu_port
    attempts = 5
    if mcu_port is not None and mcu_port.isOpen():
        while 0 < attempts:
            mcu_port.reset_input_buffer()
            mcu_port.reset_output_buffer()
            mcu_port.write('S'.encode('UTF-8'))
            t0 = time.time()
            bytesinbuffer = 0
            while 0 == bytesinbuffer:
                bytesinbuffer = mcu_port.in_waiting
                if MAX_TIMEOUT < (time.time() - t0) :
                    print('...timeout 234')
                    attempts -= 1
                    break
            if 0 < bytesinbuffer:
                answer = mcu_port.read_all().decode('UTF-8')
                if 'R' == answer[0]:
                    crcvalue = get_crc(cmdbuffer[0:256])
                    cmdbuffer[RECORD_LENGTH-2] = crcvalue & 0xFF
                    cmdbuffer[RECORD_LENGTH-1] = (crcvalue >> 8) & 0xFF
                    mcu_port.reset_input_buffer()
                    mcu_port.write(cmdbuffer)
                    # wait for an answer
                    t0 = time.time()
                    bytesinbuffer = 0
                    while 4 > bytesinbuffer :
                        bytesinbuffer = mcu_port.in_waiting
                        if MAX_TIMEOUT < (time.time() - t0) :
                            print('...timeout')
                            attempts -= 1
                            break
                    if 0 < bytesinbuffer:
                        answer = mcu_port.read(mcu_port.in_waiting).decode('UTF-8')
                        if 'ACK\n' == answer:
                            return True
                        else:
                            attempts -= 1
    return False

menu_entries = {
    'R': 'Read whole flash chip',
    'G': 'Get several blocks',
    'W': 'Write blocks ',
    'C': 'Check empty flash',
    'E': 'Erase whole flash',
    'I': 'Select serial port',
    'S': 'Start selected sound program',
    'F': 'Finish playing sound',
    'V': 'Change volume',
    'P': 'Prepare serial port',
    'Q': 'Exit'
}

tty_name = '/dev/ttyUSB0'
def draw_screen():
    print('\nActions:')
    s = ''
    for key, value in menu_entries.items():
        print(f'{key}:{value}')

def main():
    global mcu_port
    if mcu_port is not None and mcu_port.isOpen():
        mcu_port.close()
    port_name = tty_name
    cmdbuffer = bytearray([0]*RECORD_LENGTH)
    try:
        mcu_port = serial.Serial(port_name, baudrate=115200, bytesize=8,
                                 parity='N', stopbits=1, xonxoff=0, rtscts=0, timeout=5)
        mcu_port.dtr = False
    except(ValueError, serial.SerialException) as e:
        print('serial port initialization error : ')
        s = ' '+e.strerror
        sidx = s.find(':')
        oln = s[:sidx]+'\n'+s[sidx:]
        print(f'{oln}')
    while True:
        draw_screen()
        tmp_s = input('Enter : ')
        action_string = tmp_s.upper()
        if 1 == len(action_string):
            if 'Q' == action_string :
                break
            if 'I' == action_string :
                tmp_s = input(f'Device interface ['+tty_name+'] (Y/n) : ')
                if 0 != len(tmp_s) and 'N' == tmp_s.upper() :
                    tmp_s = input('tty device interface name : ')
                    if 0 != len(tmp_s) :
                        port_name = tmp_s
            if 'C' == action_string :
                # send check empty flash command
                cmdbuffer[1] = 0
                cmdbuffer[0] = 'C'.encode('UTF-8')[0]
                if not sendRecord(cmdbuffer) :
                    print('problem sending check empty chip command')
                    continue
                else:
                    print('check command sent OK')
                # receive progress info till end of checking
                print('')
                while True:
                    if receiveRecord(cmdbuffer) :
                        if 0x42 == cmdbuffer[0]:
                            # next progress step
                            blocks_checked = cmdbuffer[1]+(cmdbuffer[2] << 8)
                            error_blocks = cmdbuffer[3]+(cmdbuffer[4] << 8)
                            print(f'Blocks checked : {blocks_checked}\t\terror blocks : {error_blocks}\r', end=' ')
                        else:
                            break;
                    else:
                        print('The progress record receive problem')
                        break;
                #
                print('')
                if 0x4F == cmdbuffer[1]:
                    if 0x31 == cmdbuffer[0]:
                        print('The chip is empty')
                    else:
                        print('The chip is not empty')
                        nonemptyblocks = (cmdbuffer[2] + (cmdbuffer[3] << 8))
                        print(f'Non-empty blocks : {nonemptyblocks}')
                else:
                    print('problem receiving operation status ')
            if 'V' == action_string :
                # ask for playing volume
                tmps = input('Playing volume [0..7] : ')
                next_volume = int(tmps)
                if (0 <= next_volume) and ( 8 > next_volume):
                    cmdbuffer[1] = 0
                    cmdbuffer[0] = 'V'.encode('UTF-8')[0]
                    cmdbuffer[2] = next_volume
                    if not sendRecord(cmdbuffer):
                        print('problem sending volume set command')
                        continue
                    else:
                        print('volume set command sent OK')
                else:
                    print('Values should be in range 0..7')
            if 'S' == action_string :
                # ask for program number to "play"
                tmps = input('Program number to play : ')
                program_to_play = int(tmps)
                if (0 <= program_to_play) and (220 > program_to_play):
                    cmdbuffer[1] = 0
                    cmdbuffer[0] = 'S'.encode('UTF-8')[0]
                    cmdbuffer[2] = program_to_play
                    if not sendRecord(cmdbuffer) :
                        print('problem sending play command')
                        continue
                    else:
                        print('play command sent OK')
                else:
                    print('Values should be in range 0..219')
            if 'F' == action_string :
                cmdbuffer[1] = 0
                cmdbuffer[0] = 'F'.encode('UTF-8')[0]
                if not sendRecord(cmdbuffer) :
                    print('problem sending stop play command')
                    continue
                else:
                    print('stop play command sent OK')
            if 'E' == action_string :
                # send erase whole chip command
                cmdbuffer[1] = 0
                cmdbuffer[0] = 'E'.encode('UTF-8')[0]
                if not sendRecord(cmdbuffer) :
                    print('problem sending chip erase command')
                    continue
                else:
                    print('chip erase command sent OK')
                # receive progress info till end of checking
                print('')
                waitsymbols = ['\\','|','/','-']
                offset = 0;
                while True:
                    if receiveRecord(cmdbuffer) :
                        if 0x42 == cmdbuffer[0]:
                            # next progress step
                            print(f'Waiting {waitsymbols[offset % 4]}\r', end=' ')
                        else:
                            break;
                    else:
                        print('The progress record receive problem')
                        break;
                if 0x4F == cmdbuffer[1]:
                    if 0x46 == cmdbuffer[0]:
                        print('Chip erasing complete')
            if 'G' == action_string :
                # read arbiterary number of blocks
                number_of_blocks_string = input('Read blocks : ')
                number_of_blocks = int(number_of_blocks_string)
                cmdbuffer[3] = (number_of_blocks >> 8) & 0xFF
                cmdbuffer[2] = number_of_blocks & 0xFF
                cmdbuffer[1] = 0
                cmdbuffer[0] = 'R'.encode('UTF-8')[0]
                if not sendRecord(cmdbuffer):
                    print('problem sending chip read command')
                    break
                else:
                    print('command sent OK')
                # read blocks till end
                print('')
                data_blocks = []
                isreceived = True
                for i in range(0, number_of_blocks) :
                    if receiveRecord(cmdbuffer):
                        data_blocks.append(cmdbuffer[0:256])
                        if 0 == (i % 100):
                            print(f'Blocks in : {i}\r', end=' ')
                    else:
                        isreceived = False
                        print(f'problem receiving block : {i}')
                        break
                if isreceived:
                    print(f'Blocks in : {number_of_blocks}')
                    with open('chip_a.bin', 'wb') as f :
                        for i in range(0, number_of_blocks) :
                            f.write(data_blocks[i])
                print('')
            if 'R' == action_string :
                # read whole chip contents
                cmdbuffer[3] = (BLOCKS_FOR_CHIP >> 8) & 0xFF
                cmdbuffer[2] = BLOCKS_FOR_CHIP & 0xFF
                cmdbuffer[1] = 0
                cmdbuffer[0] = 'R'.encode('UTF-8')[0]
                if not sendRecord(cmdbuffer):
                    print('problem sending chip read command')
                    break
                else:
                    print('command sent OK')
                # read blocks till end
                print('')
                data_blocks = []
                isreceived = True
                for i in range(0, BLOCKS_FOR_CHIP) :
                    if receiveRecord(cmdbuffer):
                        data_blocks.append(cmdbuffer[0:256])
                        if 0 == (i % 100):
                            print(f'Blocks in : {i}\r', end=' ')
                    else:
                        isreceived = False
                        print(f'problem receiving block : {i}')
                        break
                if isreceived:
                    print(f'Blocks in : {BLOCKS_FOR_CHIP}')
                    with open('chip.bin', 'wb') as f :
                        for i in range(0, BLOCKS_FOR_CHIP) :
                            f.write(data_blocks[i])
                print('')
            if 'W' == action_string :
                # write data blocks (256 bytes pages) to the flash
                data_file_name = input('Binary data file : ')
                if os.path.isfile(data_file_name) :
                    with open(data_file_name, 'rb') as f :
                        blocks_to_write = f.read()
                    blocks_count = int(len(blocks_to_write)/256)
                    print(f'Total blocks : {blocks_count}')
                    from_block_string = input('Write from block : ')
                    from_block = int(from_block_string)
                    cmdbuffer[0] = 'W'.encode('UTF-8')[0]
                    cmdbuffer[1] = 0
                    cmdbuffer[2] = from_block & 0xFF
                    cmdbuffer[3] = (from_block >> 8) & 0xFF
                    cmdbuffer[4] = blocks_count & 0xFF;
                    cmdbuffer[5] = (blocks_count >> 8) & 0xFF;
                    if not sendRecord(cmdbuffer) :
                        print('problem sending write blocks command')
                        break
                    else:
                        print('command sent OK')
                        # sending binary data blocks
                        for i in range(0, blocks_count):
                            cmdbuffer[0:256] = blocks_to_write[i*256:i*256+256]
                            if not sendRecord(cmdbuffer) :
                                print(f'problem sending block #{i}')
                                break
                            else:
                                print(f'blocks sent : {i}\r')
                            if not receiveRecord(cmdbuffer):
                                print(f'problem receiving block write status for block {i}')
                            else:
                                if 0x31 == cmdbuffer[0]:
                                    print(f'problem during write block {i}')
                else:
                    print(f'No data file : {data_file_name}')
            if 'P' == action_string :
                try:
                    mcu_port = serial.Serial(port=port_name, baudrate=115200, bytesize=8,
                                             parity='N', stopbits=1, xonxoff=0, rtscts=0, timeout=5)
                except(ValueError, serial.SerialException) as e:
                    print(f'serial port initialization error {e}')

        print(f'_____ Port name set to : {port_name}')
    #  open dat file if exists
    # with open('stellaris_data.csv', 'r') as f:
    #    for ln in f:
    #        datablock.append(ln)
    # processData()
    #  readWholeMemory(mcu_port)
    if mcu_port is not None and mcu_port.isOpen():
        mcu_port.close()
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()
    # w = crc16("123456789".encode('UTF-8'))
    # print(f'CRC16 = {w} : {w:02x}')
    # w = get_crc("123456789".encode('UTF-8'))
    # print(f'get_crc = {w} : {w:02x}')



# See PyCharm help at https://www.jetbrains.com/help/pycharm/
