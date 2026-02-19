#!/usr/bin/env python3
"""
UAV Auto-Tracking System - Main Application
Інтеграція ScopeController (трекінг) з CaughtMode (MSP override)

Режими роботи (CH7):
- OFF (~1000): Тільки читання RC, без трекінгу
- CAUGHT (~1500): Активний трекінг + MSP override на основі позиції об'єкта
- KILL (~2000): Аварійна зупинка

Архітектура: Shared Serial Port з Thread-safe доступом
"""

import cv2
import serial
import struct
import time
import threading
import numpy as np
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

# Імпорт трекера (припускаємо що tracker.py існує)
try:
    from tracker import MILTracker
except ImportError:
    print("[WARN] tracker.py не знайдено, використовуємо заглушку")
    class MILTracker:
        def init(self, frame, bbox): pass
        def update(self, frame):
            class Result:
                bbox = None
            return Result()

# Перевірка picamera2
try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("[WARN] picamera2 недоступна, буде використано OpenCV камеру")


class Mode(Enum):
    OFF = "OFF"
    CAUGHT = "CAUGHT"
    KILL = "KILL"


@dataclass
class TrackingResult:
    """Результат трекінгу для передачі в CaughtMode"""
    bbox: Optional[Tuple[int, int, int, int]] = None
    center_offset: Tuple[float, float] = (0.0, 0.0)  # Нормалізований offset від центру (-1..1)
    confidence: float = 0.0
    timestamp: float = 0.0


class SharedSerialManager:
    """
    Менеджер спільного serial порту з thread-safe доступом.
    Всі компоненти системи використовують один serial port.
    """
    
    def __init__(self, port: str = '/dev/serial0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        
    def connect(self) -> bool:
        """Підключення до serial порту"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[SERIAL] Підключено до {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"[SERIAL] Помилка підключення: {e}")
            return False
    
    def disconnect(self):
        """Відключення serial порту"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[SERIAL] Відключено")
    
    def read_rc_channels(self) -> Optional[list]:
        """
        Читання RC каналів через MSP_RC (105/0x69)
        Повертає список з 16 каналів або None при помилці
        """
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b'$M<\x00\x69\x69')
                data = self.ser.read(100)
                
                if len(data) >= 23 and data[:3] == b'$M>':
                    length = data[3]
                    payload = data[5:5 + length]
                    
                    if len(payload) >= 22:
                        # Розпаковуємо 11 каналів (22 байти)
                        channels = list(struct.unpack('<11H', payload[:22]))
                        return channels
            except Exception as e:
                pass
            return None
    
    def send_rc_override(self, rc_values: list) -> bool:
        """
        Відправка MSP_SET_RAW_RC (200) override команди
        rc_values: [Roll, Pitch, Yaw, Throttle, AUX1-AUX12...]
        """
        with self.lock:
            try:
                # Мінімум 4 канали, максимум 16
                values = rc_values[:16] if len(rc_values) > 16 else rc_values
                
                # Формування MSP пакету
                payload = struct.pack(f'<{len(values)}H', *values)
                cmd = 200
                checksum = len(payload) ^ cmd
                for byte in payload:
                    checksum ^= byte
                
                packet = b'$M<' + bytes([len(payload), cmd]) + payload + bytes([checksum])
                
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                time.sleep(0.001)
                return True
            except Exception as e:
                print(f"[SERIAL] Помилка MSP override: {e}")
                return False


class CaughtModeController:
    """
    Режим CAUGHT з інтеграцією трекінгу.
    Конвертує позицію об'єкта в RC override команди.
    """
    
    def __init__(self, serial_manager: SharedSerialManager):
        self.serial_manager = serial_manager
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Базові RC значення (зберігаються з OFF режиму)
        self.base_rc_values = [1500, 1500, 1500, 1500]  # Roll, Pitch, Yaw, Throttle
        
        # Поточний результат трекінгу (оновлюється з ScopeController)
        self.tracking_result: Optional[TrackingResult] = None
        self.tracking_lock = threading.Lock()
        
        # PID/Пропорційні коефіцієнти для конвертації offset → RC
        self.gain_roll = 300   # Максимальне відхилення Roll (1500 ± 300)
        self.gain_pitch = 300  # Максимальне відхилення Pitch
        
        # Deadzone для ігнорування малих відхилень
        self.deadzone = 0.05  # 5% від центру
        
    def set_base_rc(self, rc_values: list):
        """Встановлення базових RC значень з OFF режиму"""
        if rc_values and len(rc_values) >= 4:
            self.base_rc_values = [max(1000, min(2000, int(v))) for v in rc_values[:4]]
            print(f"[CAUGHT] Базові RC встановлено та будуть використані для override:")
            print(f"[CAUGHT]   base_rc_values = {self.base_rc_values}")
    
    def update_tracking(self, result: TrackingResult):
        """Оновлення результату трекінгу (викликається з ScopeController)"""
        with self.tracking_lock:
            self.tracking_result = result
    
    def _calculate_rc_override(self) -> list:
        """
        Розрахунок RC override на основі позиції об'єкта.
        Повертає [Roll, Pitch, Yaw, Throttle]
        """
        with self.tracking_lock:
            result = self.tracking_result
        
        if result is None or result.bbox is None:
            # Немає трекінгу - повертаємо базові значення
            return self.base_rc_values.copy()
        
        offset_x, offset_y = result.center_offset
        
        # Застосовуємо deadzone
        if abs(offset_x) < self.deadzone:
            offset_x = 0
        if abs(offset_y) < self.deadzone:
            offset_y = 0
        
        # Конвертація offset в RC команди
        # offset_x > 0 → об'єкт справа → Roll вправо (збільшуємо)
        # offset_y > 0 → об'єкт знизу → Pitch вперед (для компенсації)
        roll_delta = int(offset_x * self.gain_roll)
        pitch_delta = int(-offset_y * self.gain_pitch)  # Інвертуємо для правильного напрямку
        
        rc_override = [
            max(1000, min(2000, self.base_rc_values[0] + roll_delta)),   # Roll
            max(1000, min(2000, self.base_rc_values[1] + pitch_delta)),  # Pitch
            self.base_rc_values[2],  # Yaw (без змін)
            self.base_rc_values[3],  # Throttle (без змін)
        ]
        
        return rc_override
    
    def _override_loop(self):
        """Головний цикл відправки RC override (50Hz)"""
        print("[CAUGHT] Запуск override loop (50Hz)")
        
        # Перший вивід - показуємо що саме відправляємо
        first_rc = self._calculate_rc_override()
        print(f"[CAUGHT] Перші RC override значення:")
        print(f"[CAUGHT]   Roll:     {first_rc[0]} (base: {self.base_rc_values[0]})")
        print(f"[CAUGHT]   Pitch:    {first_rc[1]} (base: {self.base_rc_values[1]})")
        print(f"[CAUGHT]   Yaw:      {first_rc[2]} (base: {self.base_rc_values[2]})")
        print(f"[CAUGHT]   Throttle: {first_rc[3]} (base: {self.base_rc_values[3]})")
        
        iteration = 0
        while self.running:
            rc_values = self._calculate_rc_override()
            self.serial_manager.send_rc_override(rc_values)
            
            # Періодичний вивід кожні 2 секунди (100 ітерацій * 0.02с)
            iteration += 1
            if iteration % 100 == 0:
                with self.tracking_lock:
                    has_tracking = self.tracking_result is not None and self.tracking_result.bbox is not None
                
                if has_tracking:
                    offset = self.tracking_result.center_offset
                    print(f"[CAUGHT] RC: R:{rc_values[0]} P:{rc_values[1]} | "
                          f"Tracking offset: ({offset[0]:+.2f}, {offset[1]:+.2f})")
                else:
                    print(f"[CAUGHT] RC: R:{rc_values[0]} P:{rc_values[1]} Y:{rc_values[2]} T:{rc_values[3]} | No tracking")
            
            time.sleep(0.02)  # 50Hz
    
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
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)
            print("[CAUGHT] Режим зупинено")


class ScopeController:
    """
    Контролер трекінгу з оптичним зумом.
    Рефакторинг для використання SharedSerialManager.
    """
    
    def __init__(self, serial_manager: SharedSerialManager,
                 resolution: Tuple[int, int] = (640, 480),
                 bbox_size: Tuple[int, int] = (100, 100)):
        
        self.serial_manager = serial_manager
        self.resolution = resolution
        self.initial_bbox_size = bbox_size
        self.current_bbox_size = list(bbox_size)
        
        # Ініціалізація камери
        self.camera = None
        self._init_camera()
        
        # Трекер
        self.tracker = MILTracker()
        
        # Стан
        self.running = False
        self.rc_stick = (50, 50)
        self.aux7_value = 1500
        self.deadzone = 3
        
        # Параметри масштабування bbox
        self.bbox_min_size = (17, 17)
        self.bbox_max_size = (100, 100)
        
        # Розмір мініатюри
        self.preview_size = (150, 150)
        
        # Callback для передачі результатів трекінгу
        self.tracking_callback = None
        
        # RC reader thread
        self.rc_thread: Optional[threading.Thread] = None
        
        # Поточні RC значення для збереження при переході в CAUGHT
        self.last_rc_values = [1500, 1500, 1500, 1500]
        
    def _init_camera(self):
        """Ініціалізація камери"""
        if PICAMERA_AVAILABLE:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                raw={"size": (1640, 1232)},
                main={"format": "RGB888", "size": self.resolution}
            )
            self.camera.configure(config)
            print("[SCOPE] Picamera2 ініціалізовано")
        else:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            print("[SCOPE] OpenCV камера ініціалізовано")
    
    def _capture_frame(self) -> Optional[np.ndarray]:
        """Захоплення кадру з камери"""
        if PICAMERA_AVAILABLE:
            return self.camera.capture_array()
        else:
            ret, frame = self.camera.read()
            if ret:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return None
    
    def _start_camera(self):
        """Запуск камери"""
        if PICAMERA_AVAILABLE:
            self.camera.start()
        print("[SCOPE] Камера запущена")
    
    def _stop_camera(self):
        """Зупинка камери"""
        if PICAMERA_AVAILABLE:
            self.camera.stop()
            self.camera.close()
        else:
            self.camera.release()
        print("[SCOPE] Камера зупинена")
    
    def _rc_reader_thread(self):
        """Потік читання RC каналів"""
        while self.running:
            channels = self.serial_manager.read_rc_channels()
            
            if channels and len(channels) >= 11:
                # Roll і Pitch для керування offset
                roll, pitch = channels[0], channels[1]
                
                x = max(0, min(100, (roll - 1000) // 10))
                y = max(0, min(100, 100 - (pitch - 1000) // 10))  # Реверс
                
                self.rc_stick = (x, y)
                
                # AUX7 (11 канал, індекс 10) для розміру bbox
                self.aux7_value = channels[10]
                self._update_bbox_size()
                
                # Зберігаємо RC значення
                self.last_rc_values = channels[:4]
            
            time.sleep(0.02)  # 50Hz
    
    def _update_bbox_size(self):
        """Оновлення розміру bbox на основі AUX7"""
        aux7_normalized = max(0, min(1, (self.aux7_value - 1000) / 1000))
        
        new_w = int(self.bbox_min_size[0] + 
                   (self.bbox_max_size[0] - self.bbox_min_size[0]) * aux7_normalized)
        new_h = int(self.bbox_min_size[1] + 
                   (self.bbox_max_size[1] - self.bbox_min_size[1]) * aux7_normalized)
        
        self.current_bbox_size = [new_w, new_h]
    
    def _get_offset(self) -> Tuple[int, int]:
        """Отримання offset від RC стіків"""
        x, y = self.rc_stick
        
        if abs(x - 50) <= self.deadzone and abs(y - 50) <= self.deadzone:
            return (0, 0)
        
        offset_x = int((x - 50) * 5 / 3)
        offset_y = int((y - 50) * 5 / 3)
        
        return (offset_x, offset_y)
    
    def _calculate_center_offset(self, bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        """
        Розрахунок нормалізованого offset центру bbox від центру кадру.
        Повертає (-1..1, -1..1)
        """
        x, y, w, h = bbox
        center_x = x + w / 2
        center_y = y + h / 2
        
        frame_center_x = self.resolution[0] / 2
        frame_center_y = self.resolution[1] / 2
        
        offset_x = (center_x - frame_center_x) / frame_center_x  # -1..1
        offset_y = (center_y - frame_center_y) / frame_center_y  # -1..1
        
        return (offset_x, offset_y)
    
    def _draw_bbox_preview(self, frame_original, bbox):
        """Відображення мініатюри bbox"""
        x, y, w, h = bbox
        
        x = max(0, min(x, frame_original.shape[1] - 1))
        y = max(0, min(y, frame_original.shape[0] - 1))
        w = max(1, min(w, frame_original.shape[1] - x))
        h = max(1, min(h, frame_original.shape[0] - y))
        
        roi = frame_original[y:y+h, x:x+w].copy()
        
        if roi.size == 0:
            return None
        
        preview = cv2.resize(roi, self.preview_size, interpolation=cv2.INTER_LINEAR)
        return preview
    
    def set_tracking_callback(self, callback):
        """Встановлення callback для результатів трекінгу"""
        self.tracking_callback = callback
    
    def get_last_rc_values(self) -> list:
        """Отримання останніх RC значень"""
        return self.last_rc_values.copy()
    
    def run_single_frame(self, frame: np.ndarray) -> Optional[TrackingResult]:
        """
        Обробка одного кадру (для інтеграції з зовнішнім циклом)
        Повертає TrackingResult
        """
        result = self.tracker.update(frame)
        
        if result.bbox:
            offset_x, offset_y = self._get_offset()
            
            bbox_size_changed = (self.current_bbox_size[0] != self._prev_bbox_size[0] or
                                self.current_bbox_size[1] != self._prev_bbox_size[1])
            
            if offset_x != 0 or offset_y != 0 or bbox_size_changed:
                x, y, w, h = result.bbox
                new_w, new_h = self.current_bbox_size
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                new_x = center_x - new_w // 2 + offset_x
                new_y = center_y - new_h // 2 + offset_y
                
                new_x = max(0, min(new_x, self.resolution[0] - new_w))
                new_y = max(0, min(new_y, self.resolution[1] - new_h))
                
                new_bbox = (new_x, new_y, new_w, new_h)
                self.tracker.init(frame, new_bbox)
                display_bbox = new_bbox
                
                self._prev_bbox_size = self.current_bbox_size.copy()
            else:
                display_bbox = result.bbox
            
            center_offset = self._calculate_center_offset(display_bbox)
            
            return TrackingResult(
                bbox=display_bbox,
                center_offset=center_offset,
                confidence=1.0,
                timestamp=time.time()
            )
        
        return TrackingResult(timestamp=time.time())
    
    def run(self, headless: bool = False):
        """
        Головний цикл трекінгу.
        headless=True для роботи без GUI
        """
        self._start_camera()
        self.running = True
        self._prev_bbox_size = self.current_bbox_size.copy()
        
        if not headless:
            cv2.namedWindow("Scope Tracker")
        
        # Запуск RC reader
        self.rc_thread = threading.Thread(target=self._rc_reader_thread, daemon=True)
        self.rc_thread.start()
        
        # Ініціалізація трекера
        frame = self._capture_frame()
        if frame is None:
            print("[SCOPE] Помилка захоплення кадру")
            return
            
        w, h = 33, 33
        x = (self.resolution[0] - w) // 2
        y = (self.resolution[1] - h) // 2
        self.tracker.init(frame, (x, y, w, h))
        
        try:
            while self.running:
                frame = self._capture_frame()
                if frame is None:
                    continue
                    
                frame_clean = frame.copy()
                
                tracking_result = self.run_single_frame(frame)
                
                # Виклик callback для передачі результату
                if self.tracking_callback and tracking_result:
                    self.tracking_callback(tracking_result)
                
                if not headless and tracking_result and tracking_result.bbox:
                    # Візуалізація
                    display_bbox = tracking_result.bbox
                    preview = self._draw_bbox_preview(frame_clean, display_bbox)
                    
                    x, y, w, h = display_bbox
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    if preview is not None:
                        preview_x = frame.shape[1] - self.preview_size[0] - 10
                        preview_y = 10
                        
                        overlay = frame.copy()
                        cv2.rectangle(overlay,
                                     (preview_x - 5, preview_y - 5),
                                     (preview_x + self.preview_size[0] + 5, 
                                      preview_y + self.preview_size[1] + 5),
                                     (0, 0, 0), -1)
                        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
                        
                        frame[preview_y:preview_y+self.preview_size[1],
                              preview_x:preview_x+self.preview_size[0]] = preview
                        
                        cv2.rectangle(frame,
                                     (preview_x, preview_y),
                                     (preview_x + self.preview_size[0], 
                                      preview_y + self.preview_size[1]),
                                     (0, 255, 0), 2)
                    
                    # Інформація
                    cv2.putText(frame, f"RC: {self.rc_stick[0]}, {self.rc_stick[1]}",
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"AUX7: {self.aux7_value} Size: {w}x{h}",
                               (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    offset = tracking_result.center_offset
                    cv2.putText(frame, f"Offset: {offset[0]:.2f}, {offset[1]:.2f}",
                               (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                if not headless:
                    cv2.imshow("Scope Tracker", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self._stop_camera()
            if not headless:
                cv2.destroyAllWindows()
    
    def stop(self):
        """Зупинка контролера"""
        self.running = False


class ModeRouter:
    """
    Роутер режимів на основі CH7.
    Керує переходами між OFF/CAUGHT/KILL.
    """
    
    def __init__(self, serial_manager: SharedSerialManager,
                 scope_controller: ScopeController,
                 caught_controller: CaughtModeController):
        
        self.serial_manager = serial_manager
        self.scope = scope_controller
        self.caught = caught_controller
        
        self.current_mode = Mode.OFF
        self.running = False
        
        # Гістерезис для стабільності перемикання
        self.ch7_history = []
        self.history_size = 3
        
    def _get_mode_from_ch7(self, ch7_value: int) -> Mode:
        """Визначення режиму за значенням CH7"""
        if abs(ch7_value - 1000) < 100:
            return Mode.OFF
        elif abs(ch7_value - 1500) < 100:
            return Mode.CAUGHT
        elif abs(ch7_value - 2000) < 100:
            return Mode.KILL
        return self.current_mode
    
    def _get_stable_mode(self) -> Mode:
        """Отримання стабільного режиму з гістерезисом"""
        channels = self.serial_manager.read_rc_channels()
        
        if channels and len(channels) >= 7:
            ch7 = channels[6]  # 7-й канал (індекс 6)
            mode = self._get_mode_from_ch7(ch7)
            
            self.ch7_history.append(mode)
            if len(self.ch7_history) > self.history_size:
                self.ch7_history.pop(0)
            
            # Повертаємо режим тільки якщо всі значення в історії однакові
            if len(self.ch7_history) >= self.history_size:
                if all(m == self.ch7_history[0] for m in self.ch7_history):
                    return self.ch7_history[0]
        
        return self.current_mode
    
    def _handle_mode_change(self, new_mode: Mode):
        """Обробка зміни режиму"""
        if new_mode == self.current_mode:
            return
        
        print(f"\n{'='*60}")
        print(f"[ROUTER] Перехід: {self.current_mode.value} → {new_mode.value}")
        print(f"{'='*60}")
        
        # Зупинка поточного режиму
        if self.current_mode == Mode.CAUGHT:
            self.caught.stop()
        
        # Збереження RC при виході з OFF (або з будь-якого режиму при переході в CAUGHT)
        last_rc = self.scope.get_last_rc_values()
        
        self.current_mode = new_mode
        
        # Запуск нового режиму
        if new_mode == Mode.CAUGHT:
            print(f"\n[CAUGHT] ============ АКТИВАЦІЯ РЕЖИМУ CAUGHT ============")
            print(f"[CAUGHT] Останні RC значення з OFF режиму:")
            print(f"[CAUGHT]   Roll:     {last_rc[0]}")
            print(f"[CAUGHT]   Pitch:    {last_rc[1]}")
            print(f"[CAUGHT]   Yaw:      {last_rc[2]}")
            print(f"[CAUGHT]   Throttle: {last_rc[3]}")
            print(f"[CAUGHT] ===================================================\n")
            
            # Встановлюємо базові RC значення
            self.caught.set_base_rc(last_rc)
            
            # Встановлюємо callback для передачі результатів трекінгу
            self.scope.set_tracking_callback(self.caught.update_tracking)
            self.caught.start()
        elif new_mode == Mode.KILL:
            print("[ROUTER] *** KILL MODE - Аварійна зупинка ***")
            self.scope.stop()
        elif new_mode == Mode.OFF:
            print(f"[OFF] Повернення в OFF режим")
            print(f"[OFF] Поточні RC: R:{last_rc[0]} P:{last_rc[1]} Y:{last_rc[2]} T:{last_rc[3]}")
            self.scope.set_tracking_callback(None)
    
    def run(self):
        """Головний цикл роутера"""
        self.running = True
        print("[ROUTER] Запуск системи")
        
        try:
            while self.running:
                new_mode = self._get_stable_mode()
                self._handle_mode_change(new_mode)
                time.sleep(0.05)  # 20Hz перевірка режиму
                
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.current_mode == Mode.CAUGHT:
                self.caught.stop()
            print("[ROUTER] Зупинка системи")
    
    def stop(self):
        """Зупинка роутера"""
        self.running = False


class Application:
    """
    Головний клас застосунку.
    Об'єднує всі компоненти системи.
    """
    
    def __init__(self, serial_port: str = '/dev/serial0',
                 resolution: Tuple[int, int] = (640, 480),
                 headless: bool = False):
        
        self.headless = headless
        
        # Ініціалізація компонентів
        print("="*50)
        print("UAV Auto-Tracking System")
        print("="*50)
        
        # Serial manager
        self.serial_manager = SharedSerialManager(serial_port)
        if not self.serial_manager.connect():
            raise RuntimeError("Не вдалося підключитися до serial порту")
        
        # Scope Controller (трекінг)
        self.scope = ScopeController(
            self.serial_manager,
            resolution=resolution
        )
        
        # Caught Mode Controller
        self.caught = CaughtModeController(self.serial_manager)
        
        # Mode Router
        self.router = ModeRouter(
            self.serial_manager,
            self.scope,
            self.caught
        )
        
        # Threads
        self.scope_thread: Optional[threading.Thread] = None
        self.router_thread: Optional[threading.Thread] = None
        
    def run(self):
        """Запуск застосунку"""
        print("\n[APP] Запуск...")
        print("[APP] CH7 ~1000 = OFF | CH7 ~1500 = CAUGHT | CH7 ~2000 = KILL")
        print("[APP] Натисніть 'q' для виходу\n")
        
        try:
            # Запуск Scope Controller в окремому потоці
            self.scope_thread = threading.Thread(
                target=self.scope.run,
                kwargs={'headless': self.headless},
                daemon=True
            )
            self.scope_thread.start()
            
            # Запуск Router в головному потоці
            self.router.run()
            
        except KeyboardInterrupt:
            print("\n[APP] Отримано сигнал завершення")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Коректне завершення"""
        print("\n[APP] Завершення роботи...")
        
        self.router.stop()
        self.scope.stop()
        self.caught.stop()
        
        time.sleep(0.5)
        self.serial_manager.disconnect()
        
        print("[APP] Готово")


def main():
    """Entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='UAV Auto-Tracking System')
    parser.add_argument('--port', default='/dev/serial0', help='Serial port')
    parser.add_argument('--resolution', default='640x480', help='Camera resolution')
    parser.add_argument('--headless', action='store_true', help='Run without GUI')
    
    args = parser.parse_args()
    
    # Парсинг resolution
    res = tuple(map(int, args.resolution.split('x')))
    
    app = Application(
        serial_port=args.port,
        resolution=res,
        headless=args.headless
    )
    
    app.run()


if __name__ == "__main__":
    main()