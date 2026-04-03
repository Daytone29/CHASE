#!/usr/bin/env python3
"""
Тестовий скрипт для визначення RC каналів
Показує значення всіх каналів з MSP_RC
"""

import serial
import time
import struct
import sys
import os

# Додаємо шлях до CV для імпорту
sys.path.append('/home/obriy/CHASE/CV')

# Імпорт MSP helper
sys.path.append(os.path.dirname(__file__))
import msp_helper

# Налаштування
SERIAL_PORT = '/dev/serial0'  # або інший порт
BAUD_RATE = 115200

def connect_serial():
    """Підключення до серійного порту"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"✅ Підключено до {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"❌ Помилка підключення: {e}")
        return None

def get_rc_channels(ser):
    """Отримання та розпакування RC каналів"""
    try:
        # Надсилаємо запит MSP_RC
        msp_helper.send_msp_request(ser, msp_helper.MSP_RC)
        time.sleep(0.1)  # Чекаємо відповідь

        # Читаємо відповідь
        response = msp_helper.read_msp_response(ser)
        if response:
            msp_id, payload = response
            if msp_id == msp_helper.MSP_RC and payload:
                # Розпаковуємо всі канали як unsigned short (H)
                num_channels = len(payload) // 2
                channels = struct.unpack('<' + 'H' * num_channels, payload)
                return channels
        return None
    except Exception as e:
        print(f"❌ Помилка читання RC: {e}")
        return None

def main():
    print("🧪 Тест RC каналів")
    print("Перемикайте тумблери та дивіться які канали змінюються")
    print("Ctrl+C для виходу\n")

    ser = connect_serial()
    if not ser:
        return

    try:
        while True:
            channels = get_rc_channels(ser)
            if channels:
                print(f"Канали: {[f'CH{i+1}={val}' for i, val in enumerate(channels)]}")
            else:
                print("❌ Не вдалося отримати канали")

            time.sleep(0.5)  # Оновлення кожні 0.5 сек

    except KeyboardInterrupt:
        print("\n👋 Вихід")
    finally:
        ser.close()

if __name__ == "__main__":
    main()