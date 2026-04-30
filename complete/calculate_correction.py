#!/usr/bin/env python3
import math
import mmap
import os
import signal
import struct
import time
from dataclasses import dataclass


SHM_PATH = "/dev/shm/crsf_tracking_correction_v1"

TARGET_HZ = 50.0
LOOP_PERIOD_SEC = 1.0 / TARGET_HZ

TARGET_TIMEOUT_SEC = 0.20
CORRECTION_TIMEOUT_SEC = 0.20

YAW_CHANNEL_DELTA_SCALE = 400.0
THROTTLE_CHANNEL_DELTA_SCALE = 300.0

TARGET_PAYLOAD_FORMAT = "<ddIId"
CORRECTION_PAYLOAD_FORMAT = "<ddd"
SEQ_FORMAT = "<Q"
SEQ_SIZE = struct.calcsize(SEQ_FORMAT)

TARGET_BLOCK_OFFSET = 0
TARGET_BLOCK_SIZE = SEQ_SIZE + struct.calcsize(TARGET_PAYLOAD_FORMAT)

CORRECTION_BLOCK_OFFSET = TARGET_BLOCK_OFFSET + TARGET_BLOCK_SIZE
CORRECTION_BLOCK_SIZE = SEQ_SIZE + struct.calcsize(CORRECTION_PAYLOAD_FORMAT)

SHM_SIZE = TARGET_BLOCK_SIZE + CORRECTION_BLOCK_SIZE

running = True


@dataclass(frozen=True)
class TargetSample:
    target_x: float | None
    target_y: float | None
    frame_w: int
    frame_h: int
    timestamp: float


@dataclass(frozen=True)
class CorrectionSample:
    yaw_delta: float
    throttle_delta: float
    timestamp: float


def stop_handler(signum, frame) -> None:
    global running
    running = False


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def is_fresh(timestamp: float, now: float, timeout_sec: float) -> bool:
    age = now - timestamp
    return 0.0 <= age <= timeout_sec


class TrackingCorrectionMemory:
    def __init__(self, path: str = SHM_PATH):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
        os.ftruncate(self.fd, SHM_SIZE)
        self.mm = mmap.mmap(self.fd, SHM_SIZE, access=mmap.ACCESS_WRITE)

    def close(self) -> None:
        self.mm.close()
        os.close(self.fd)

    def _write_block(self, offset: int, payload_format: str, values: tuple) -> None:
        seq = struct.unpack_from(SEQ_FORMAT, self.mm, offset)[0]
        odd_seq = seq + 1 if seq % 2 == 0 else seq + 2
        even_seq = odd_seq + 1

        struct.pack_into(SEQ_FORMAT, self.mm, offset, odd_seq)
        struct.pack_into(payload_format, self.mm, offset + SEQ_SIZE, *values)
        struct.pack_into(SEQ_FORMAT, self.mm, offset, even_seq)

    def _read_block(self, offset: int, payload_format: str) -> tuple | None:
        for _ in range(3):
            seq_before = struct.unpack_from(SEQ_FORMAT, self.mm, offset)[0]

            if seq_before % 2:
                continue

            values = struct.unpack_from(payload_format, self.mm, offset + SEQ_SIZE)
            seq_after = struct.unpack_from(SEQ_FORMAT, self.mm, offset)[0]

            if seq_before == seq_after and seq_after % 2 == 0:
                return values

        return None

    def write_target(
        self,
        target_x: float | None,
        target_y: float | None,
        frame_w: int,
        frame_h: int,
        timestamp: float | None = None,
    ) -> None:
        ts = time.monotonic() if timestamp is None else timestamp
        x = math.nan if target_x is None else float(target_x)
        y = math.nan if target_y is None else float(target_y)

        self._write_block(
            TARGET_BLOCK_OFFSET,
            TARGET_PAYLOAD_FORMAT,
            (x, y, int(frame_w), int(frame_h), ts),
        )

    def read_target(self) -> TargetSample:
        values = self._read_block(TARGET_BLOCK_OFFSET, TARGET_PAYLOAD_FORMAT)

        if values is None:
            return TargetSample(None, None, 0, 0, 0.0)

        target_x, target_y, frame_w, frame_h, timestamp = values

        if math.isnan(target_x) or math.isnan(target_y):
            return TargetSample(None, None, frame_w, frame_h, timestamp)

        return TargetSample(target_x, target_y, frame_w, frame_h, timestamp)

    def write_correction(
        self,
        yaw_delta: float,
        throttle_delta: float,
        timestamp: float | None = None,
    ) -> None:
        ts = time.monotonic() if timestamp is None else timestamp

        self._write_block(
            CORRECTION_BLOCK_OFFSET,
            CORRECTION_PAYLOAD_FORMAT,
            (float(yaw_delta), float(throttle_delta), ts),
        )

    def read_correction(self, timeout_sec: float = CORRECTION_TIMEOUT_SEC) -> CorrectionSample:
        values = self._read_block(CORRECTION_BLOCK_OFFSET, CORRECTION_PAYLOAD_FORMAT)

        if values is None:
            return CorrectionSample(0.0, 0.0, 0.0)

        yaw_delta, throttle_delta, timestamp = values
        now = time.monotonic()

        if not is_fresh(timestamp, now, timeout_sec):
            return CorrectionSample(0.0, 0.0, timestamp)

        return CorrectionSample(yaw_delta, throttle_delta, timestamp)


class TrackingPDController:
    def __init__(self):
        self.previous_yaw_error = 0.0
        self.previous_throttle_error = 0.0
        self.previous_yaw_delta = 0.0
        self.previous_throttle_delta = 0.0

    def reset(self) -> None:
        self.previous_yaw_error = 0.0
        self.previous_throttle_error = 0.0
        self.previous_yaw_delta = 0.0
        self.previous_throttle_delta = 0.0

    def update(
        self,
        target_x: float | None,
        target_y: float | None,
        frame_w: int,
        frame_h: int,
        dt: float,
    ) -> tuple[float, float]:
        if (
            target_x is None
            or target_y is None
            or frame_w <= 0
            or frame_h <= 0
            or dt <= 0.0
        ):
            self.reset()
            return 0.0, 0.0

        center_x = frame_w / 2.0
        center_y = frame_h / 2.0

        yaw_error = clamp((target_x - center_x) / center_x, -1.0, 1.0)
        throttle_error = clamp((center_y - target_y) / center_y, -1.0, 1.0)

        if abs(yaw_error) < 0.04:
            yaw_error = 0.0

        if abs(throttle_error) < 0.05:
            throttle_error = 0.0

        yaw_delta = self._pd_step(
            error=yaw_error,
            previous_error=self.previous_yaw_error,
            previous_value=self.previous_yaw_delta,
            kp=0.22,
            kd=0.05,
            max_delta=0.15,
            rate_limit=0.40,
            dt=dt,
        )
        throttle_delta = self._pd_step(
            error=throttle_error,
            previous_error=self.previous_throttle_error,
            previous_value=self.previous_throttle_delta,
            kp=0.15,
            kd=0.04,
            max_delta=0.10,
            rate_limit=0.30,
            dt=dt,
        )

        self.previous_yaw_error = yaw_error
        self.previous_throttle_error = throttle_error
        self.previous_yaw_delta = yaw_delta
        self.previous_throttle_delta = throttle_delta

        return yaw_delta, throttle_delta

    @staticmethod
    def _pd_step(
        error: float,
        previous_error: float,
        previous_value: float,
        kp: float,
        kd: float,
        max_delta: float,
        rate_limit: float,
        dt: float,
    ) -> float:
        requested = kp * error + kd * ((error - previous_error) / dt)
        requested = clamp(requested, -max_delta, max_delta)

        max_step = rate_limit * dt
        return clamp(requested, previous_value - max_step, previous_value + max_step)


def scaled_correction(
    controller: TrackingPDController,
    target_x: float | None,
    target_y: float | None,
    frame_w: int,
    frame_h: int,
    dt: float,
) -> tuple[float, float]:
    yaw_delta, throttle_delta = controller.update(target_x, target_y, frame_w, frame_h, dt)

    return (
        yaw_delta * YAW_CHANNEL_DELTA_SCALE,
        throttle_delta * THROTTLE_CHANNEL_DELTA_SCALE,
    )


def run_worker() -> None:
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    shared = TrackingCorrectionMemory()
    controller = TrackingPDController()
    previous_time = time.monotonic()

    try:
        while running:
            loop_start = time.monotonic()
            dt = loop_start - previous_time
            previous_time = loop_start

            sample = shared.read_target()

            if is_fresh(sample.timestamp, loop_start, TARGET_TIMEOUT_SEC):
                yaw_delta, throttle_delta = scaled_correction(
                    controller,
                    sample.target_x,
                    sample.target_y,
                    sample.frame_w,
                    sample.frame_h,
                    dt,
                )
            else:
                controller.reset()
                yaw_delta = 0.0
                throttle_delta = 0.0

            shared.write_correction(yaw_delta, throttle_delta, loop_start)

            sleep_time = LOOP_PERIOD_SEC - (time.monotonic() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    finally:
        shared.write_correction(0.0, 0.0)
        shared.close()


def main() -> None:
    run_worker()


if __name__ == "__main__":
    main()