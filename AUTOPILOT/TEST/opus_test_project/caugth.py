#!/usr/bin/env python3
"""
CaughtMode - Режим "заморожування" RC значень.
Передає останні RC значення з OFF режиму на польотний контролер через MSP override.

Без модифікацій від трекінгу - просто фіксує Roll, Pitch, Yaw, Throttle.
"""

import struct
import time
import threading
from typing import Optional, List


class CaughtMode:
    """
    Режим CAUGHT - передача збережених RC значень на польотний контролер.
    
    При переході OFF → CAUGHT:
    1. Зберігаються останні RC значення [Roll, Pitch, Yaw, Throttle]
    2. Ці значення передаються через MSP_SET_RAW_RC (cmd 200) з частотою 50Hz
    3. Дрон "заморожує" своє положення на момент перемикання
    """
    
    def __init__(self, serial_port, serial_lock):
        """
        Ініціалізація CaughtMode.
        
        Args:
            serial_port: Відкритий serial.Serial об'єкт
            serial_lock: threading.Lock для синхронізації доступу
        """
        self.ser = serial_port
        self.lock = serial_lock
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # RC значення [Roll, Pitch, Yaw, Throttle]
        self.rc_values = [1500, 1500, 1500, 1500]
        
        # Лічильник для періодичного виводу
        self.packet_counter = 0
        
    def set_rc_values(self, rc_data):
        """
        Встановлення RC значень з OFF режиму.
        
        Args:
            rc_data: список [roll, pitch, yaw, throttle] або dict
        """
        if not rc_data:
            return
            
        # Конвертація dict → list
        if isinstance(rc_data, dict):
            rc_array = [
                rc_data.get('roll', 1500),
                rc_data.get('pitch', 1500),
                rc_data.get('yaw', 1500),
                rc_data.get('throttle', 1500)
            ]
        else:
            rc_array = list(rc_data)
        
        # Перевірка та обмеження значень (1000-2000)
        if len(rc_array) >= 4:
            self.rc_values = [max(1000, min(2000, int(val))) for val in rc_array[:4]]
            
            print(f"\n{'='*60}")
            print(f"[CAUGHT] RC значення збережено з OFF режиму:")
            print(f"[CAUGHT]   Roll:     {self.rc_values[0]}")
            print(f"[CAUGHT]   Pitch:    {self.rc_values[1]}")
            print(f"[CAUGHT]   Yaw:      {self.rc_values[2]}")
            print(f"[CAUGHT]   Throttle: {self.rc_values[3]}")
            print(f"{'='*60}\n")
    
    def _build_msp_packet(self) -> bytes:
        """
        Формування MSP_SET_RAW_RC пакету.
        
        Структура MSP v1:
        $M< [length] [cmd] [payload...] [checksum]
        
        cmd = 200 (MSP_SET_RAW_RC)
        payload = 4 x uint16 (Roll, Pitch, Yaw, Throttle)
        """
        payload = struct.pack('<4H', *self.rc_values)
        cmd = 200  # MSP_SET_RAW_RC
        
        # Checksum = XOR всіх байтів (length, cmd, payload)
        checksum = len(payload) ^ cmd
        for byte in payload:
            checksum ^= byte
            
        return b'$M<' + bytes([len(payload), cmd]) + payload + bytes([checksum])
    
    def _send_override(self) -> bool:
        """
        Відправка MSP Override команди.
        
        Returns:
            True якщо успішно, False при помилці
        """
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                packet = self._build_msp_packet()
                self.ser.write(packet)
                time.sleep(0.001)  # Невелика затримка для стабільності
                return True
            except Exception as e:
                print(f"[CAUGHT] Помилка MSP: {e}")
                return False
    
    def _override_loop(self):
        """
        Циклічна відправка override команд (50Hz).
        Виводить значення кожну секунду.
        """
        print(f"\n[CAUGHT] Старт override loop @ 50Hz")
        print(f"[CAUGHT] Передаю на FC: R:{self.rc_values[0]} P:{self.rc_values[1]} "
              f"Y:{self.rc_values[2]} T:{self.rc_values[3]}")
        
        self.packet_counter = 0
        start_time = time.time()
        
        while self.running:
            success = self._send_override()
            self.packet_counter += 1
            
            # Вивід кожну секунду (50 пакетів)
            if self.packet_counter % 50 == 0:
                elapsed = time.time() - start_time
                rate = self.packet_counter / elapsed if elapsed > 0 else 0
                
                print(f"[CAUGHT] TX #{self.packet_counter}: "
                      f"R:{self.rc_values[0]} P:{self.rc_values[1]} "
                      f"Y:{self.rc_values[2]} T:{self.rc_values[3]} "
                      f"({rate:.1f} Hz)")
            
            time.sleep(0.02)  # 50Hz = 20ms
        
        # Фінальна статистика
        total_time = time.time() - start_time
        print(f"\n[CAUGHT] Статистика:")
        print(f"[CAUGHT]   Відправлено пакетів: {self.packet_counter}")
        print(f"[CAUGHT]   Час роботи: {total_time:.1f}с")
        print(f"[CAUGHT]   Середня частота: {self.packet_counter/total_time:.1f} Hz")
    
    def start(self):
        """Запуск режиму CAUGHT"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._override_loop, daemon=True)
            self.thread.start()
            print("[CAUGHT] Режим активовано")
    
    def stop(self):
        """Зупинка режиму CAUGHT"""
        if self.running:
            print("\n[CAUGHT] Зупинка...")
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            print("[CAUGHT] Режим зупинено")


class CaughtModeV2:
    """
    Версія для використання з SharedSerialManager.
    Використовується в app.py.
    """
    
    def __init__(self, serial_manager):
        """
        Args:
            serial_manager: SharedSerialManager instance
        """
        self.serial_manager = serial_manager
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # RC значення [Roll, Pitch, Yaw, Throttle]
        self.rc_values = [1500, 1500, 1500, 1500]
        
        # Лічильник пакетів
        self.packet_counter = 0
    
    def set_base_rc(self, rc_values: List[int]):
        """Встановлення RC значень"""
        if rc_values and len(rc_values) >= 4:
            self.rc_values = [max(1000, min(2000, int(v))) for v in rc_values[:4]]
            
            print(f"\n{'='*60}")
            print(f"[CAUGHT] RC значення збережено:")
            print(f"[CAUGHT]   Roll:     {self.rc_values[0]}")
            print(f"[CAUGHT]   Pitch:    {self.rc_values[1]}")
            print(f"[CAUGHT]   Yaw:      {self.rc_values[2]}")
            print(f"[CAUGHT]   Throttle: {self.rc_values[3]}")
            print(f"{'='*60}\n")
    
    def _loop(self):
        """Головний цикл передачі RC"""
        print(f"[CAUGHT] Старт override @ 50Hz")
        
        self.packet_counter = 0
        start_time = time.time()
        
        while self.running:
            self.serial_manager.send_rc_override(self.rc_values)
            self.packet_counter += 1
            
            # Вивід кожну секунду
            if self.packet_counter % 50 == 0:
                elapsed = time.time() - start_time
                rate = self.packet_counter / elapsed if elapsed > 0 else 0
                
                print(f"[CAUGHT] #{self.packet_counter}: "
                      f"R:{self.rc_values[0]} P:{self.rc_values[1]} "
                      f"Y:{self.rc_values[2]} T:{self.rc_values[3]} "
                      f"({rate:.1f} Hz)")
            
            time.sleep(0.02)
        
        # Статистика
        total_time = time.time() - start_time
        if total_time > 0:
            print(f"\n[CAUGHT] Відправлено: {self.packet_counter} пакетів за {total_time:.1f}с")
    
    def start(self):
        """Запуск"""
        if not self.running:
            self.running = True
            self.packet_counter = 0
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            print("[CAUGHT] Активовано")
    
    def stop(self):
        """Зупинка"""
        if self.running:
            print("[CAUGHT] Зупинка...")
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            print("[CAUGHT] Зупинено")


# =============================================================================
# Тестування
# =============================================================================

if __name__ == "__main__":
    import serial
    
    print("="*60)
    print("CaughtMode - Тест")
    print("="*60)
    
    try:
        ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
        lock = threading.Lock()
        
        caught = CaughtMode(ser, lock)
        
        # Симуляція RC значень з OFF режиму
        test_rc = [1500, 1550, 1500, 1400]  # Трохи pitch forward, throttle down
        caught.set_rc_values(test_rc)
        
        print("\n[TEST] Запуск на 5 секунд...")
        caught.start()
        
        time.sleep(5)
        
        caught.stop()
        ser.close()
        
        print("\n[TEST] Тест завершено")
        
    except Exception as e:
        print(f"\n[TEST] Помилка: {e}")