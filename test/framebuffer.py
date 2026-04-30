#!/usr/bin/env python3
import os
import signal
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2


WIDTH = 720
HEIGHT = 576
FPS = 25.0
FRAME_PERIOD = 1.0 / FPS

BBOX_W = 100
BBOX_H = 100
BOX_THICKNESS = 2

STATE_POLL_PERIOD = 0.5
STATE_FILE = Path("/tmp/crsf_control_state")

STATE_MANUAL = "MANUAL"
STATE_TARGET = "TARGET"
STATE_LOST = "LOST"

VALID_STATES = {
    STATE_MANUAL,
    STATE_TARGET,
    STATE_LOST,
}

RGB_GREEN = (0, 255, 0)
RGB_RED = (255, 0, 0)

running = True


def stop_handler(signum, frame):
    global running
    running = False


def read_state(default: str = STATE_MANUAL) -> str:
    if not STATE_FILE.exists():
        return default

    state = STATE_FILE.read_text(encoding="ascii").strip()
    return state if state in VALID_STATES else default


def center_bbox() -> tuple[int, int, int, int]:
    return (
        (WIDTH - BBOX_W) // 2,
        (HEIGHT - BBOX_H) // 2,
        BBOX_W,
        BBOX_H,
    )


def create_tracker(frame: np.ndarray, bbox: tuple[int, int, int, int]):
    tracker = cv2.legacy.TrackerMOSSE_create()
    tracker.init(frame, bbox)
    return tracker


def draw_box(frame: np.ndarray, bbox: tuple[int, int, int, int], label: str) -> None:
    x, y, w, h = map(int, bbox)

    cv2.rectangle(
        frame,
        (x, y),
        (x + w, y + h),
        RGB_GREEN,
        BOX_THICKNESS,
    )

    cv2.putText(
        frame,
        label,
        (x, max(20, y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        RGB_GREEN,
        2,
    )


def draw_lost(frame: np.ndarray) -> None:
    cv2.putText(
        frame,
        STATE_LOST,
        (50, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        RGB_RED,
        2,
    )


def rgb888_to_rgb565(frame: np.ndarray) -> np.ndarray:
    r = (frame[:, :, 0] >> 3).astype(np.uint16)
    g = (frame[:, :, 1] >> 2).astype(np.uint16)
    b = (frame[:, :, 2] >> 3).astype(np.uint16)

    return (r << 11) | (g << 5) | b


def main() -> None:
    global running

    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    os.system("TERM=linux setterm -cursor off >/dev/tty0")

    framebuffer = np.memmap(
        "/dev/fb0",
        dtype=np.uint16,
        mode="w+",
        shape=(HEIGHT, WIDTH),
    )

    camera = Picamera2()
    camera.configure(
        camera.create_video_configuration(
            main={
                "size": (WIDTH, HEIGHT),
                "format": "RGB888",
            }
        )
    )
    camera.start()

    state = STATE_MANUAL
    last_state = STATE_MANUAL
    next_state_read = 0.0

    tracker = None
    bbox = center_bbox()

    while running:
        loop_start = time.monotonic()

        if loop_start >= next_state_read:
            state = read_state(state)
            next_state_read = loop_start + STATE_POLL_PERIOD

        frame = camera.capture_array()

        if state == STATE_TARGET:
            if tracker is None or last_state != STATE_TARGET:
                bbox = center_bbox()
                tracker = create_tracker(frame, bbox)

            success, tracked_bbox = tracker.update(frame)

            if success:
                bbox = tuple(map(int, tracked_bbox))
                draw_box(frame, bbox, STATE_TARGET)
            else:
                tracker = None
                draw_lost(frame)

        elif state == STATE_LOST:
            tracker = None
            draw_lost(frame)

        else:
            tracker = None
            bbox = center_bbox()
            draw_box(frame, bbox, STATE_MANUAL)

        last_state = state
        framebuffer[:] = rgb888_to_rgb565(frame)

        sleep_time = FRAME_PERIOD - (time.monotonic() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    camera.stop()


if __name__ == "__main__":
    main()