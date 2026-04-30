#!/usr/bin/env python3
import os
import signal
import time
from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2
from PIL import Image, ImageDraw, ImageFont

from calculate_correction import TrackingCorrectionMemory


WIDTH = 720
HEIGHT = 576
FPS = 25.0
FRAME_PERIOD = 1.0 / FPS

BBOX_W = 100
BBOX_H = 100

BOX_THICKNESS = 1
CROSSHAIR_THICKNESS = 1

STATE_POLL_PERIOD = 0.5
STATE_FILE = Path("/tmp/crsf_control_state")

STATE_MANUAL = "MANUAL"
STATE_TARGET = "TARGET"
STATE_ATACK = "ATACK"
STATE_LOST = "LOST"

VALID_STATES = {
    STATE_MANUAL,
    STATE_TARGET,
    STATE_ATACK,
    STATE_LOST,
}

TRACKING_STATES = {
    STATE_TARGET,
    STATE_ATACK,
}

# Кадр у форматі RGB888, тому кольори задаються як RGB, не BGR.
RGB_WHITE_GRAY = (180, 180, 180)
RGB_RED = (255, 0, 0)

FONT_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
)

running = True
pil_font = None


def stop_handler(signum, frame):
    global running
    running = False


def load_pil_font(size: int = 22):
    for font_path in FONT_PATHS:
        if Path(font_path).exists():
            return ImageFont.truetype(font_path, size)

    return ImageFont.load_default()


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


def fixed_bbox(tracked_bbox: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
    x, y = tracked_bbox[:2]
    return int(round(x)), int(round(y)), BBOX_W, BBOX_H


def bbox_center(bbox: tuple[int, int, int, int]) -> tuple[float, float]:
    x, y, _, _ = bbox
    return x + BBOX_W / 2.0, y + BBOX_H / 2.0


def create_tracker(frame: np.ndarray, bbox: tuple[int, int, int, int]):
    tracker = cv2.legacy.TrackerMOSSE_create()
    tracker.init(frame, bbox)
    return tracker


def state_label(state: str) -> str:
    if state == STATE_TARGET:
        return "ЦІЛЬ"
    if state == STATE_ATACK:
        return "АТАКА"
    return state


def draw_unicode_text(
    frame: np.ndarray,
    text: str,
    position: tuple[int, int],
    color: tuple[int, int, int] = RGB_WHITE_GRAY,
) -> None:
    global pil_font

    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    draw.text(position, text, font=pil_font, fill=color)
    frame[:] = np.asarray(image)


def draw_box(frame: np.ndarray, bbox: tuple[int, int, int, int], label: str | None = None) -> None:
    x, y, w, h = map(int, bbox)

    cv2.rectangle(
        frame,
        (x, y),
        (x + w, y + h),
        RGB_WHITE_GRAY,
        BOX_THICKNESS,
    )

    if label:
        text_x = x
        text_y = max(2, y - 28)

        draw_unicode_text(
            frame,
            label,
            (text_x, text_y),
            RGB_WHITE_GRAY,
        )


def draw_manual_crosshair(frame: np.ndarray) -> None:
    cx = WIDTH // 2
    cy = HEIGHT // 2

    outer_w = 120
    outer_h = 120
    corner_len = 22

    center_gap = 18
    center_len = 16

    left = cx - outer_w // 2
    right = cx + outer_w // 2
    top = cy - outer_h // 2
    bottom = cy + outer_h // 2

    color = RGB_WHITE_GRAY
    thickness = CROSSHAIR_THICKNESS

    cv2.line(frame, (left, top), (left + corner_len, top), color, thickness)
    cv2.line(frame, (left, top), (left, top + corner_len), color, thickness)

    cv2.line(frame, (right, top), (right - corner_len, top), color, thickness)
    cv2.line(frame, (right, top), (right, top + corner_len), color, thickness)

    cv2.line(frame, (left, bottom), (left + corner_len, bottom), color, thickness)
    cv2.line(frame, (left, bottom), (left, bottom - corner_len), color, thickness)

    cv2.line(frame, (right, bottom), (right - corner_len, bottom), color, thickness)
    cv2.line(frame, (right, bottom), (right, bottom - corner_len), color, thickness)

    cv2.line(
        frame,
        (cx - center_gap - center_len, cy),
        (cx - center_gap, cy),
        color,
        thickness,
    )
    cv2.line(
        frame,
        (cx + center_gap, cy),
        (cx + center_gap + center_len, cy),
        color,
        thickness,
    )
    cv2.line(
        frame,
        (cx, cy - center_gap - center_len),
        (cx, cy - center_gap),
        color,
        thickness,
    )
    cv2.line(
        frame,
        (cx, cy + center_gap),
        (cx, cy + center_gap + center_len),
        color,
        thickness,
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
    global pil_font

    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    os.system("TERM=linux setterm -cursor off >/dev/tty0")

    pil_font = load_pil_font(size=22)

    framebuffer = np.memmap(
        "/dev/fb0",
        dtype=np.uint16,
        mode="w+",
        shape=(HEIGHT, WIDTH),
    )

    target_memory = TrackingCorrectionMemory()
    camera = Picamera2()
    camera_started = False

    try:
        camera.configure(
            camera.create_video_configuration(
                main={
                    "size": (WIDTH, HEIGHT),
                    "format": "RGB888",
                }
            )
        )
        camera.start()
        camera_started = True

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

            if state in TRACKING_STATES:
                if tracker is None or last_state not in TRACKING_STATES:
                    bbox = center_bbox()
                    tracker = create_tracker(frame, bbox)

                success, tracked_bbox = tracker.update(frame)

                if success:
                    bbox = fixed_bbox(tracked_bbox)
                    target_x, target_y = bbox_center(bbox)

                    target_memory.write_target(target_x, target_y, WIDTH, HEIGHT)
                    draw_box(frame, bbox, state_label(state))
                else:
                    tracker = None
                    target_memory.write_target(None, None, WIDTH, HEIGHT)
                    draw_lost(frame)

            elif state == STATE_LOST:
                tracker = None
                target_memory.write_target(None, None, WIDTH, HEIGHT)
                draw_lost(frame)

            else:
                tracker = None
                bbox = center_bbox()

                target_memory.write_target(None, None, WIDTH, HEIGHT)
                draw_manual_crosshair(frame)

            last_state = state
            framebuffer[:] = rgb888_to_rgb565(frame)

            sleep_time = FRAME_PERIOD - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        target_memory.write_target(None, None, WIDTH, HEIGHT)
        target_memory.close()

        if camera_started:
            camera.stop()


if __name__ == "__main__":
    main()