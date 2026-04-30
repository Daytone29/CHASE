import numpy as np
import os
import cv2
from picamera2 import Picamera2

os.system("TERM=linux setterm -cursor off >/dev/tty0")

WIDTH, HEIGHT = 720, 576

fb = np.memmap('/dev/fb0', dtype='uint16', mode='w+', shape=(HEIGHT, WIDTH))

picam2 = Picamera2()
picam2.configure(
    picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
    )
)
picam2.start()


bbox = (300, 200, 100, 100)  

tracker = cv2.TrackerCSRT_create()
frame = picam2.capture_array()
frame = cv2.resize(frame, (WIDTH, HEIGHT))
tracker.init(frame, bbox)

def rgb888_to_rgb565(img):
    b = (img[:, :, 0] >> 3).astype(np.uint16)
    g = (img[:, :, 1] >> 2).astype(np.uint16)
    r = (img[:, :, 2] >> 3).astype(np.uint16)
    return (r << 11) | (g << 5) | b

while True:
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (WIDTH, HEIGHT))

    success, box = tracker.update(frame)

    if success:
        x, y, w, h = [int(v) for v in box]

        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, "TRACKING", (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    else:
        cv2.putText(frame, "LOST", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    fb[:] = rgb888_to_rgb565(frame)