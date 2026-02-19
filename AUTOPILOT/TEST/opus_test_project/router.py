#!/usr/bin/env python3
"""
ModeRouter - Керування режимами на основі RC каналу.
Версія для standalone використання (без app.py) або як модуль.

Режими:
- OFF (~1000): Тільки читання RC
- CAUGHT (~1500): Активний трекінг + MSP override
- KILL (~2000): Аварійна зупинка
"""

import serial
import struct
import time
import threading
from enum import Enum
from typing import Optional, Callable, List
from dataclasses import dataclass


class Mode(Enum):
    OFF = "OFF"
    CAUGHT = "CAUGHT"
    KILL = "KILL"


@dataclass
class ModeChangeEvent:
    """Подія зміни режиму"""
    old_mode: Mode
    new_mode: Mode
    timestamp: float
    last_rc_values: List[int]


class RCReader:
    """Читач RC каналів через MSP"""
    
    def __init__(self, serial_port, serial_lock):
        self.ser = serial_port
        self.lock = serial_lock
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Останні прочитані значення
        self.rc_values = [1500] * 16
        self.rc_lock = threading.Lock()
        
    def _read_loop(self):
        """Цикл читання RC"""
        while self.running:
            with self.lock:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.write(b'$M<\x00\x69\x69')  # MSP_RC
                    data = self.ser.read(50)
                    
                    if len(data) >= 23 and data[:3] == b'$M>':
                        length = data[3]
                        payload = data[5:5 + length]
                        
                        if len(payload) >= 22:
                            channels = list(struct.unpack('<11H', payload[:22]))
                            with self.rc_lock:
                                self.rc_values[:11] = channels
                                
                except Exception as e:
                    pass
            
            time.sleep(0.02)  # 50Hz
    
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print("[RC_READER] Запущено")
    
    def stop(self) -> List[int]:
        """Зупинка та повернення останніх RC значень"""
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            print("[RC_READER] Зупинено")
        
        with self.rc_lock:
            return self.rc_values[:4].copy()
    
    def get_rc_values(self) -> List[int]:
        """Отримання поточних RC значень"""
        with self.rc_lock:
            return self.rc_values.copy()
    
    def get_channel(self, channel: int) -> int:
        """Отримання значення конкретного каналу (1-based)"""
        with self.rc_lock:
            if 1 <= channel <= len(self.rc_values):
                return self.rc_values[channel - 1]
            return 1500


class ModeRouter:
    """
    Роутер режимів на основі CH7.
    Може працювати автономно або інтегруватися з app.py.
    """
    
    def __init__(self, serial_port, serial_lock,
                 mode_channel: int = 7,
                 on_mode_change: Optional[Callable[[ModeChangeEvent], None]] = None):
        """
        Args:
            serial_port: serial.Serial об'єкт
            serial_lock: threading.Lock
            mode_channel: Канал для визначення режиму (1-based)
            on_mode_change: Callback при зміні режиму
        """
        self.ser = serial_port
        self.lock = serial_lock
        self.mode_channel = mode_channel
        self.on_mode_change = on_mode_change
        
        self.current_mode = Mode.OFF
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # RC Reader для збереження останніх значень
        self.rc_reader = RCReader(serial_port, serial_lock)
        
        # Гістерезис
        self.mode_history: List[Mode] = []
        self.history_size = 3
        
        # Thresholds
        self.thresholds = {
            Mode.OFF: (900, 1100),      # ~1000
            Mode.CAUGHT: (1400, 1600),  # ~1500
            Mode.KILL: (1900, 2100),    # ~2000
        }
    
    def _get_mode_from_value(self, value: int) -> Mode:
        """Визначення режиму за значенням каналу"""
        for mode, (low, high) in self.thresholds.items():
            if low <= value <= high:
                return mode
        return self.current_mode
    
    def _read_mode_channel(self) -> int:
        """Читання значення каналу режиму"""
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b'$M<\x00\x69\x69')
                data = self.ser.read(50)
                
                if len(data) >= 19 and data[:3] == b'$M>':
                    payload = data[5:19]
                    if len(payload) >= 14:
                        channels = struct.unpack('<7H', payload)
                        return channels[self.mode_channel - 1]
            except:
                pass
        return 1500
    
    def _get_stable_mode(self) -> Mode:
        """Отримання стабільного режиму з гістерезисом"""
        value = self._read_mode_channel()
        mode = self._get_mode_from_value(value)
        
        self.mode_history.append(mode)
        if len(self.mode_history) > self.history_size:
            self.mode_history.pop(0)
        
        # Повертаємо тільки якщо всі значення однакові
        if (len(self.mode_history) >= self.history_size and
            all(m == self.mode_history[0] for m in self.mode_history)):
            return self.mode_history[0]
        
        return self.current_mode
    
    def _handle_mode_change(self, new_mode: Mode):
        """Обробка зміни режиму"""
        if new_mode == self.current_mode:
            return
        
        old_mode = self.current_mode
        last_rc = self.rc_reader.get_rc_values()[:4]
        
        print(f"\n{'='*50}")
        print(f"[ROUTER] Зміна режиму: {old_mode.value} → {new_mode.value}")
        print(f"[ROUTER] Останні RC: {last_rc}")
        print(f"{'='*50}\n")
        
        self.current_mode = new_mode
        
        # Виклик callback
        if self.on_mode_change:
            event = ModeChangeEvent(
                old_mode=old_mode,
                new_mode=new_mode,
                timestamp=time.time(),
                last_rc_values=last_rc
            )
            self.on_mode_change(event)
    
    def _routing_loop(self):
        """Головний цикл роутингу"""
        while self.running:
            new_mode = self._get_stable_mode()
            self._handle_mode_change(new_mode)
            time.sleep(0.05)  # 20Hz
    
    def start(self):
        """Запуск роутера"""
        if not self.running:
            self.running = True
            self.rc_reader.start()
            self.thread = threading.Thread(target=self._routing_loop, daemon=True)
            self.thread.start()
            print("[ROUTER] Запущено")
    
    def stop(self):
        """Зупинка роутера"""
        if self.running:
            self.running = False
            self.rc_reader.stop()
            if self.thread:
                self.thread.join(timeout=1.0)
            print("[ROUTER] Зупинено")
    
    def get_current_mode(self) -> Mode:
        """Отримання поточного режиму"""
        return self.current_mode
    
    def get_last_rc_values(self) -> List[int]:
        """Отримання останніх RC значень"""
        return self.rc_reader.get_rc_values()[:4]


# ============================================================================
# Standalone режим (для тестування без app.py)
# ============================================================================

def main():
    """Standalone запуск роутера"""
    from caught import CaughtMode
    
    # Ініціалізація
    ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)
    serial_lock = threading.Lock()
    
    # CaughtMode
    caught_mode = CaughtMode(ser, serial_lock)
    
    # Останні RC
    last_rc_values = None
    
    def on_mode_change(event: ModeChangeEvent):
        nonlocal last_rc_values
        
        # Зупинка попереднього режиму
        if event.old_mode == Mode.CAUGHT:
            caught_mode.stop()
        
        # Збереження RC при виході з OFF
        if event.old_mode == Mode.OFF:
            last_rc_values = event.last_rc_values
        
        # Запуск нового режиму
        if event.new_mode == Mode.CAUGHT:
            if last_rc_values:
                caught_mode.set_rc_values(last_rc_values)
            else:
                caught_mode.set_rc_values([1500, 1500, 1500, 1500])
            caught_mode.start()
        elif event.new_mode == Mode.KILL:
            print("*** KILL MODE ACTIVATED ***")
    
    # Створення роутера
    router = ModeRouter(ser, serial_lock, on_mode_change=on_mode_change)
    
    print("[MAIN] Запуск системи")
    print("[MAIN] CH7: ~1000=OFF, ~1500=CAUGHT, ~2000=KILL")
    print("[MAIN] Ctrl+C для виходу\n")
    
    router.start()
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[MAIN] Завершення...")
        router.stop()
        if router.current_mode == Mode.CAUGHT:
            caught_mode.stop()
        ser.close()
        print("[MAIN] Готово")


if __name__ == "__main__":
    main()