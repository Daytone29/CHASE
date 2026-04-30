#!/usr/bin/env python3
import serial
import time
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

CRSF_SYNC = 0xC8

RX_PORT = "/dev/ttyUSB0"
TX_PORT = "/dev/serial0"
BAUDRATE = 420000

CRSF_MAX_FRAME_SIZE = 64

# RC_CHANNELS_PACKED:
# sync + len + type + 22 payload + crc = 26 bytes total
CRSF_RC_FRAME_TOTAL_LEN = 26
CRSF_RC_LENGTH_FIELD = 0x18
CRSF_RC_PAYLOAD_LEN = 22
CRSF_RC_CHANNELS_COUNT = 16

# AUX7 у CRSF channel index:
# ch1 index 0
# ch2 index 1
# ...
# ch7 index 6
AUX7_INDEX = 7

# Діапазон, при якому активується HOLD
HOLD_MIN = 900
HOLD_MAX = 1050


# ================= PACKET TYPES =================
class PacketType(IntEnum):
    RC_CHANNELS_PACKED = 0x16
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    LINK_STATISTICS = 0x14


# ================= CRC =================
def crc8_dvb_s2_byte(crc: int, byte: int) -> int:
    crc ^= byte
    for _ in range(8):
        if crc & 0x80:
            crc = (crc << 1) ^ 0xD5
        else:
            crc <<= 1
    return crc & 0xFF


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = crc8_dvb_s2_byte(crc, b)
    return crc


def is_valid_frame(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    return crc8(frame[2:-1]) == frame[-1]


# ================= CHANNEL ENCODING =================
def decode_channels(payload: bytes) -> list[int]:
    channels = []
    buffer = 0
    bits = 0

    for byte in payload:
        buffer |= byte << bits
        bits += 8

        while bits >= 11:
            channels.append(buffer & 0x7FF)
            buffer >>= 11
            bits -= 11

    return channels


def encode_channels(channels: list[int]) -> bytearray:
    out = bytearray()
    buffer = 0
    bits = 0

    for ch in channels:
        buffer |= (ch & 0x7FF) << bits
        bits += 11

        while bits >= 8:
            out.append(buffer & 0xFF)
            buffer >>= 8
            bits -= 8

    if bits:
        out.append(buffer & 0xFF)

    return out


# ================= STATE =================
@dataclass
class HoldState:
    enabled: bool = False
    saved_channels: Optional[list[int]] = None


# ================= SERIAL BRIDGE =================
class SerialBridge:
    def __init__(self, rx_port: str, tx_port: str, baudrate: int):
        self.rx = serial.Serial(rx_port, baudrate, timeout=0)
        self.tx = serial.Serial(tx_port, baudrate, timeout=0)

        self.buffer = bytearray()
        self.state = HoldState()

        self.last_debug_ts = 0.0

    def read_rx(self):
        available = self.rx.in_waiting

        if available:
            self.buffer.extend(self.rx.read(available))
        else:
            time.sleep(0.001)

    def extract_frame(self) -> Optional[bytearray]:
        while len(self.buffer) > 2:
            if self.buffer[0] != CRSF_SYNC:
                del self.buffer[0]
                continue

            frame_len = self.buffer[1] + 2

            if frame_len < 4 or frame_len > CRSF_MAX_FRAME_SIZE:
                del self.buffer[0]
                continue

            if len(self.buffer) < frame_len:
                return None

            frame = self.buffer[:frame_len]
            del self.buffer[:frame_len]

            return frame

        return None

    def is_strict_rc_frame(self, frame: bytes) -> bool:
        if len(frame) != CRSF_RC_FRAME_TOTAL_LEN:
            return False

        if frame[0] != CRSF_SYNC:
            return False

        if frame[1] != CRSF_RC_LENGTH_FIELD:
            return False

        if frame[2] != PacketType.RC_CHANNELS_PACKED:
            return False

        payload = frame[3:-1]

        if len(payload) != CRSF_RC_PAYLOAD_LEN:
            return False

        return True

    def build_rc_frame(self, channels: list[int]) -> Optional[bytearray]:
        if len(channels) != CRSF_RC_CHANNELS_COUNT:
            return None

        payload = encode_channels(channels)

        if len(payload) != CRSF_RC_PAYLOAD_LEN:
            return None

        frame = bytearray()
        frame.append(CRSF_SYNC)
        frame.append(CRSF_RC_LENGTH_FIELD)
        frame.append(PacketType.RC_CHANNELS_PACKED)
        frame.extend(payload)
        frame.append(crc8(frame[2:]))

        if len(frame) != CRSF_RC_FRAME_TOTAL_LEN:
            return None

        return frame

    def process_rc_frame(self, frame: bytearray):
        # Якщо RC frame не строго стандартний — не чіпаємо його.
        # Forward оригіналу без перекодування.
        if not self.is_strict_rc_frame(frame):
            self.forward(frame)
            return

        payload = frame[3:-1]
        channels = decode_channels(payload)

        if len(channels) != CRSF_RC_CHANNELS_COUNT:
            self.forward(frame)
            return

        aux7 = channels[AUX7_INDEX]

        hold_requested = HOLD_MIN < aux7 < HOLD_MAX

        if hold_requested:
            if not self.state.enabled:
                # Зберігаємо останній реальний стан основних RC-каналів:
                # roll, pitch, throttle, yaw
                self.state.saved_channels = channels[:4].copy()
                self.state.enabled = True
                print(">>> HOLD ON")

            if self.state.saved_channels is not None and len(self.state.saved_channels) == 4:
                channels[:4] = self.state.saved_channels
            else:
                self.forward(frame)
                return

        else:
            if self.state.enabled:
                print(">>> HOLD OFF")

            self.state.enabled = False
            self.state.saved_channels = None

        new_frame = self.build_rc_frame(channels)

        # Якщо з якоїсь причини новий frame не зібрався правильно —
        # передаємо оригінальний, щоб не створити обрив.
        if new_frame is None:
            self.forward(frame)
            return

        self.tx.write(new_frame)

    def forward(self, frame: bytes):
        self.tx.write(frame)


# ================= MAIN LOOP =================
def main():
    bridge = SerialBridge(RX_PORT, TX_PORT, BAUDRATE)

    while True:
        bridge.read_rx()

        while True:
            frame = bridge.extract_frame()

            if frame is None:
                break

            if not is_valid_frame(frame):
                continue

            packet_type = frame[2]

            if packet_type == PacketType.RC_CHANNELS_PACKED:
                bridge.process_rc_frame(frame)
            else:
                bridge.forward(frame)


if __name__ == "__main__":
    main()