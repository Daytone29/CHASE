#!/usr/bin/env python3
import serial
import time
import argparse
from enum import IntEnum

CRSF_SYNC = 0xC8

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

def signed_byte(b):
    return b - 256 if b >= 128 else b

def handleCrsfPacket(ptype, data):
    if ptype == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
        lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
        gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
        hdg = int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
        alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
        sats = data[17]
        print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
    elif ptype == PacketsTypes.VARIO:
        vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        print(f"VSpd: {vspd:0.1f}m/s")
    elif ptype == PacketsTypes.ATTITUDE:
        pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
        roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
        yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
        print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
    elif ptype == PacketsTypes.BARO_ALT:
        alt = int.from_bytes(data[3:7], byteorder='big', signed=True) / 100.0
        print(f"Baro Altitude: {alt}m")
    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = signed_byte(data[3])
        rssi2 = signed_byte(data[4])
        lq = data[5]
        snr = signed_byte(data[6])
        print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03}")
    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
        mah = data[7] << 16 | data[8] << 7 | data[9]
        pct = data[10]
        print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
    elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
        # Розпаковка каналів (11 біт на канал)
        channels = []
        bits = 0
        bit_buffer = 0
        for b in data[2:]:  # байти після типу пакета
            bit_buffer |= b << bits
            bits += 8
            while bits >= 11:
                channels.append(bit_buffer & 0x7FF)
                bit_buffer >>= 11
                bits -= 11
        # Виводимо всі канали і AUX (5-8)
        aux_channels = channels[4:8]
        print(f"RC Channels: {channels}")
        print(f"AUX Channels: {aux_channels}")

# Аргументи командного рядка
parser = argparse.ArgumentParser()
parser.add_argument('-P', '--port', default='/dev/ttyUSB0', required=False, help='Serial port to read from')
parser.add_argument('-b', '--baud', default=420000, required=False, help='Baud rate for the serial port')
args = parser.parse_args()

# Основний цикл читання серійного порту
with serial.Serial(args.port, args.baud, timeout=2) as ser:
    input_buffer = bytearray()
    while True:
        if ser.in_waiting > 0:
            input_buffer.extend(ser.read(ser.in_waiting))
        else:
            time.sleep(0.010)

        if len(input_buffer) > 2:
            expected_len = input_buffer[1] + 2
            if expected_len > 64 or expected_len < 4:
                input_buffer = bytearray()
            elif len(input_buffer) >= expected_len:
                single_packet = input_buffer[:expected_len]
                input_buffer = input_buffer[expected_len:]

                if not crsf_validate_frame(single_packet):
                    packet = ' '.join(map(hex, single_packet))
                    print(f"CRC error: {packet}")
                else:
                    handleCrsfPacket(single_packet[2], single_packet)