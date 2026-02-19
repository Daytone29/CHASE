#!/usr/bin/env python3

import serial
import struct

MSP_RC = 105

def get_checksum(msp_command_id, payload):
    checksum = 0
    length = len(payload)
    for byte in bytes([length, msp_command_id]) + payload:
        checksum ^= byte
    return checksum & 0xFF

def send_msp_request(serial_port, msp_command_id):
    header = b'$M<'
    length = 0
    checksum = get_checksum(msp_command_id, bytes([]))
    msp_package = header + struct.pack('<BB', length, msp_command_id) + bytes([checksum])
    serial_port.write(msp_package)

def read_msp_response(serial_port):
    data = serial_port.read(100)
    if len(data) >= 6 and data[:3] == b'$M>':
        length = data[3]
        msp_command_id = data[4]
        payload = data[5:5 + length]
        return msp_command_id, payload
    return None, None

def track_right_stick():
    ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
    
    msp_request = b'$M<\x00\x69\x69'
    
    while True:
        ser.write(msp_request)
        
        cmd_id, payload = read_msp_response(ser)
        
        if cmd_id == MSP_RC and payload and len(payload) >= 8:
            roll, pitch = struct.unpack('<HH', payload[0:4])
            
            x = max(0, min(100, (roll - 1000) // 10))
            y = max(0, min(100, (pitch - 1000) // 10))
            
            print(f"{x} {y}")

if __name__ == "__main__":
    track_right_stick()