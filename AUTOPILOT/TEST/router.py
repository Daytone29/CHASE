#!/usr/bin/env python3

import serial
import struct
import time
import threading
from off import RCReader
from caught import CaughtMode

# Ініціалізація
ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
serial_lock = threading.Lock()
rc_reader = RCReader(ser, serial_lock)
caught_mode = CaughtMode(ser, serial_lock)

current_mode = 'OFF'
last_rc_values = None


def get_mode_from_channel():
    """Зчитує режим з 7-го каналу"""
    with serial_lock:
        ser.reset_input_buffer()
        ser.write(b'$M<\x00\x69\x69')
        data = ser.read(22)
    
    if len(data) >= 19:
        ch7 = struct.unpack('<7H', data[5:19])[6]
        
        if abs(ch7 - 1000) < 100:
            return 'OFF'
        elif abs(ch7 - 1500) < 100:
            return 'CAUGHT'
        elif abs(ch7 - 2000) < 100:
            return 'KILL'
    
    return current_mode


def handle_mode_change(new_mode):
    """Обробка зміни режиму"""
    global current_mode, last_rc_values
    
    if new_mode == current_mode:
        return
    
    # Зупинка поточного режиму
    if current_mode == 'OFF':
        last_rc_values = rc_reader.stop()
        print(f"[MODE] Збережено RC значення: {last_rc_values}")
    elif current_mode == 'CAUGHT':
        caught_mode.stop()
    
    current_mode = new_mode
    print(f"\n[MODE] Перехід на режим: {current_mode}")
    
    # Запуск нового режиму
    if new_mode == 'OFF':
        rc_reader.start()
    elif new_mode == 'CAUGHT':
        if last_rc_values and len(last_rc_values) >= 4:
            print(f"[MODE] Використання збережених RC значень: {last_rc_values}")
            caught_mode.set_rc_values(last_rc_values)
        else:
            print(f"[MODE] УВАГА: Немає збережених RC значень, використовуємо нейтральні")
            caught_mode.set_rc_values([1500, 1500, 1500, 1500])
        
        caught_mode.start()


# Основний цикл
print("[ROUTER] Запуск системи")
rc_reader.start()

try:
    while True:
        new_mode = get_mode_from_channel()
        handle_mode_change(new_mode)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n[ROUTER] Зупинка системи")
    if current_mode == 'OFF':
        rc_reader.stop()
    elif current_mode == 'CAUGHT':
        caught_mode.stop()