#!/usr/bin/env python3
import serial
import time
from enum import IntEnum

CRSF_SYNC = 0xC8

# FIXED PORTS
RX = '/dev/ttyUSB0'
TX = '/dev/serial0'
BAUDRATE = 420000


class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A


def crc8_dvb_s2(crc, a) -> int:
    crc ^= a
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc <<= 1
    return crc & 0xFF


def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc


def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]


def handleCrsfPacket(ptype, data):
    if ptype == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], "big", signed=True) / 1e7
        lon = int.from_bytes(data[7:11], "big", signed=True) / 1e7
        print(f"GPS: {lat} {lon}")

    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = data[3] - 256 if data[3] >= 128 else data[3]
        lq = data[5]
        print(f"RSSI={rssi1} LQ={lq}")

    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], "big", signed=True) / 10.0
        print(f"Battery: {vbat:.2f}V")


# OPEN SERIAL PORTS
ser = serial.Serial(RX, BAUDRATE, timeout=2)
ser_out = serial.Serial(TX, BAUDRATE, timeout=0)

buffer = bytearray()

while True:
    if ser.in_waiting:
        buffer.extend(ser.read(ser.in_waiting))
    else:
        time.sleep(0.005)

    while len(buffer) > 2:
        if buffer[0] != CRSF_SYNC:
            buffer.pop(0)
            continue

        frame_len = buffer[1] + 2

        if frame_len < 4 or frame_len > 64:
            buffer.clear()
            break

        if len(buffer) < frame_len:
            break

        frame = buffer[:frame_len]
        buffer = buffer[frame_len:]

        if not crsf_validate_frame(frame):
            continue

        
        ser_out.write(frame)

        handleCrsfPacket(frame[2], frame)