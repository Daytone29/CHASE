#!/usr/bin/env python3

import serial
import struct

ser = serial.Serial('/dev/serial0', 115200, timeout=0.5)

while True:
    ser.write(b'$M<\x00\x69\x69')
    data = ser.read(100)
    if len(data) >= 19:
        print(struct.unpack('<7H', data[5:19])[6])