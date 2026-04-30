#!/usr/bin/env python3
import errno
import fcntl
import os
import select
import serial
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional


CRSF_SYNC = 0xC8

RX_PORT = "/dev/ttyUSB0"
TX_PORT = "/dev/serial0"
BAUDRATE = 420000

CRSF_MAX_FRAME_SIZE = 64
RX_BUFFER_LIMIT = 512
TX_BUFFER_LIMIT = 2048

CRSF_RC_FRAME_TOTAL_LEN = 26
CRSF_RC_LENGTH_FIELD = 0x18
CRSF_RC_PAYLOAD_LEN = 22
CRSF_RC_CHANNELS_COUNT = 16

AUX7_INDEX = 7

HOLD_MIN = 900
HOLD_MAX = 1050

SELECT_TIMEOUT_SEC = 0.001

STATE_FILE = "/tmp/crsf_control_state"
STATE_MANUAL = "MANUAL"
STATE_TARGET = "TARGET"


class PacketType(IntEnum):
    RC_CHANNELS_PACKED = 0x16
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    LINK_STATISTICS = 0x14


@dataclass
class HoldState:
    enabled: bool = False
    saved_channels: Optional[list[int]] = None
    mode: str = STATE_MANUAL


def write_state(value: str) -> None:
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(value)
    os.replace(tmp, STATE_FILE)


def crc8_dvb_s2_byte(crc: int, byte: int) -> int:
    crc ^= byte

    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) if crc & 0x80 else (crc << 1)

    return crc & 0xFF


def crc8(data: bytes) -> int:
    crc = 0

    for b in data:
        crc = crc8_dvb_s2_byte(crc, b)

    return crc


def is_valid_frame(frame: bytes) -> bool:
    return len(frame) >= 4 and crc8(frame[2:-1]) == frame[-1]


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


def set_nonblocking(fd: int) -> None:
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)


class SerialBridge:
    def __init__(self, rx_port: str, tx_port: str, baudrate: int):
        self.rx = serial.Serial(rx_port, baudrate, timeout=0, write_timeout=0, exclusive=True)
        self.tx = serial.Serial(tx_port, baudrate, timeout=0, write_timeout=0, exclusive=True)

        self.rx_fd = self.rx.fileno()
        self.tx_fd = self.tx.fileno()

        set_nonblocking(self.rx_fd)
        set_nonblocking(self.tx_fd)

        self.rx.reset_input_buffer()
        self.tx.reset_output_buffer()

        self.rx_buffer = bytearray()
        self.tx_buffer = bytearray()

        self.state = HoldState()
        write_state(STATE_MANUAL)

    def set_mode(self, mode: str) -> None:
        if self.state.mode == mode:
            return

        self.state.mode = mode
        write_state(mode)
        print(f">>> MODE {mode}", flush=True)

    def read_rx(self) -> None:
        while True:
            try:
                data = os.read(self.rx_fd, 256)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                raise

            if not data:
                return

            self.rx_buffer.extend(data)

            if len(self.rx_buffer) > RX_BUFFER_LIMIT:
                del self.rx_buffer[:-CRSF_MAX_FRAME_SIZE]

    def flush_tx(self) -> None:
        while self.tx_buffer:
            try:
                written = os.write(self.tx_fd, self.tx_buffer)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                raise

            if written <= 0:
                return

            del self.tx_buffer[:written]

    def send(self, frame: bytes) -> None:
        if len(self.tx_buffer) + len(frame) > TX_BUFFER_LIMIT:
            self.tx_buffer.clear()
            self.tx.reset_output_buffer()

        self.tx_buffer.extend(frame)
        self.flush_tx()

    def extract_frame(self) -> Optional[bytearray]:
        while len(self.rx_buffer) > 2:
            if self.rx_buffer[0] != CRSF_SYNC:
                del self.rx_buffer[0]
                continue

            frame_len = self.rx_buffer[1] + 2

            if frame_len < 4 or frame_len > CRSF_MAX_FRAME_SIZE:
                del self.rx_buffer[0]
                continue

            if len(self.rx_buffer) < frame_len:
                return None

            frame = self.rx_buffer[:frame_len]
            del self.rx_buffer[:frame_len]
            return frame

        return None

    def is_strict_rc_frame(self, frame: bytes) -> bool:
        return (
            len(frame) == CRSF_RC_FRAME_TOTAL_LEN
            and frame[0] == CRSF_SYNC
            and frame[1] == CRSF_RC_LENGTH_FIELD
            and frame[2] == PacketType.RC_CHANNELS_PACKED
            and len(frame[3:-1]) == CRSF_RC_PAYLOAD_LEN
        )

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

        return frame if len(frame) == CRSF_RC_FRAME_TOTAL_LEN else None

    def process_rc_frame(self, frame: bytearray) -> None:
        if not self.is_strict_rc_frame(frame):
            self.forward(frame)
            return

        channels = decode_channels(frame[3:-1])

        if len(channels) != CRSF_RC_CHANNELS_COUNT:
            self.forward(frame)
            return

        hold_requested = HOLD_MIN < channels[AUX7_INDEX] < HOLD_MAX

        if hold_requested:
            if not self.state.enabled:
                self.state.saved_channels = channels[:4].copy()
                self.state.enabled = True
                self.set_mode(STATE_TARGET)

            if self.state.saved_channels is None:
                self.forward(frame)
                return

            channels[:4] = self.state.saved_channels

        else:
            if self.state.enabled:
                self.state.enabled = False
                self.state.saved_channels = None
                self.set_mode(STATE_MANUAL)

        new_frame = self.build_rc_frame(channels)
        self.send(new_frame if new_frame is not None else frame)

    def forward(self, frame: bytes) -> None:
        self.send(frame)

    def process_frames(self) -> None:
        while True:
            frame = self.extract_frame()

            if frame is None:
                return

            if not is_valid_frame(frame):
                continue

            if frame[2] == PacketType.RC_CHANNELS_PACKED:
                self.process_rc_frame(frame)
            else:
                self.forward(frame)

    def run(self) -> None:
        while True:
            readable, writable, _ = select.select(
                [self.rx_fd],
                [self.tx_fd] if self.tx_buffer else [],
                [],
                SELECT_TIMEOUT_SEC,
            )

            if self.rx_fd in readable:
                self.read_rx()
                self.process_frames()

            if self.tx_fd in writable:
                self.flush_tx()


def main() -> None:
    SerialBridge(RX_PORT, TX_PORT, BAUDRATE).run()


if __name__ == "__main__":
    main()