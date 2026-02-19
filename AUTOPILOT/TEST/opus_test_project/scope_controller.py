#!/usr/bin/env python3
"""
ScopeController - Трекінг об'єкту з оптичним зумом.
Рефакторинг для підтримки:
1. SharedSerialManager (shared serial port)
2. Callback для передачі результатів трекінгу
3. Headless режим роботи

Використання:
- Standalone: python scope_controller.py
- Як модуль: from scope_controller import ScopeController
"""

import cv2
import struct
import time
import threading
import numpy as np
from typing import Tuple, Optional, Callable
from dataclasses import dataclass

# Перевірка залежностей
try:
    from tracker import MILTracker
    TRACKER_AVAILABLE = True
except ImportError:
    TRACKER_AVAILABLE = False
    print("[WARN] tracker.py не знайдено")

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False


@dataclass
class TrackingResult:
    """Результат трекінгу для передачі в CaughtMode"""
    bbox: Optional[Tuple[int, int, int, int]] = None
    center_offset: Tuple[float, float] = (0.0, 0.0)  # Нормалізований offset (-1..1)
    confidence: float = 0.0
    timestamp: float = 0.0
    
    def is_valid(self) -> bool:
        return self.bbox is not None


class DummyTracker:
    """Заглушка трекера для тестування без tracker.py"""
    def __init__(self):
        self._bbox = None
        
    def init(self, frame, bbox):
        self._bbox = bbox
        
    def update(self, frame):
        class Result:
            def __init__(self, bbox):
                self.bbox = bbox
        return Result(self._bbox)


class ScopeController:
    """
    Контролер трекінгу з оптичним зумом через AUX7.
    
    Функції:
    - Трекінг об'єкту (MIL tracker)
    - Читання RC каналів для керування offset
    - Масштабування bbox через AUX7
    - Мініатюра збільшеного об'єкта
    """
    
    def __init__(self, 
                 serial_manager=None,
                 resolution: Tuple[int, int] = (640, 480),
                 bbox_size: Tuple[int, int] = (100, 100)):
        """
        Args:
            serial_manager: SharedSerialManager або None для standalone
            resolution: Роздільна здатність камери
            bbox_size: Початковий розмір bbox
        """
        self.serial_manager = serial_manager
        self.resolution = resolution
        self.initial_bbox_size = bbox_size
        self.current_bbox_size = list(bbox_size)
        
        # Камера
        self.camera = None
        self._using_picamera = False
        
        # Трекер
        if TRACKER_AVAILABLE:
            self.tracker = MILTracker()
        else:
            self.tracker = DummyTracker()
        
        # Стан
        self.running = False
        self.rc_stick = (50, 50)
        self.aux7_value = 1500
        self.deadzone = 3
        
        # Параметри bbox
        self.bbox_min_size = (17, 17)
        self.bbox_max_size = (100, 100)
        
        # Мініатюра
        self.preview_size = (150, 150)
        
        # Callback для результатів
        self.tracking_callback: Optional[Callable[[TrackingResult], None]] = None
        
        # RC reader thread (для standalone режиму)
        self.rc_thread: Optional[threading.Thread] = None
        
        # Останні RC значення
        self.last_rc_values = [1500, 1500, 1500, 1500]
        self.rc_lock = threading.Lock()
        
        # Попередній розмір bbox
        self._prev_bbox_size = list(bbox_size)
        
        # Serial для standalone
        self._standalone_serial = None
        
    def _init_camera(self):
        """Ініціалізація камери"""
        if PICAMERA_AVAILABLE:
            try:
                self.camera = Picamera2()
                config = self.camera.create_preview_configuration(
                    raw={"size": (1640, 1232)},
                    main={"format": "RGB888", "size": self.resolution}
                )
                self.camera.configure(config)
                self._using_picamera = True
                print("[SCOPE] Picamera2 ініціалізовано")
                return True
            except Exception as e:
                print(f"[SCOPE] Помилка Picamera2: {e}")
        
        # Fallback на OpenCV
        try:
            self.camera = cv2.VideoCapture(0)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self._using_picamera = False
            print("[SCOPE] OpenCV камера ініціалізовано")
            return True
        except Exception as e:
            print(f"[SCOPE] Помилка OpenCV камери: {e}")
            return False
    
    def _capture_frame(self) -> Optional[np.ndarray]:
        """Захоплення кадру"""
        if self._using_picamera:
            return self.camera.capture_array()
        else:
            ret, frame = self.camera.read()
            if ret:
                return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return None
    
    def _start_camera(self):
        """Запуск камери"""
        if self._using_picamera:
            self.camera.start()
        print("[SCOPE] Камера запущена")
    
    def _stop_camera(self):
        """Зупинка камери"""
        try:
            if self._using_picamera:
                self.camera.stop()
                self.camera.close()
            else:
                self.camera.release()
            print("[SCOPE] Камера зупинена")
        except:
            pass
    
    def _rc_reader_thread_standalone(self):
        """Потік читання RC (standalone режим)"""
        import serial
        
        if self._standalone_serial is None:
            try:
                self._standalone_serial = serial.Serial('/dev/serial0', 115200, timeout=0.01)
            except Exception as e:
                print(f"[SCOPE] Помилка serial: {e}")
                return
        
        msp_request = b'$M<\x00\x69\x69'
        
        while self.running:
            try:
                self._standalone_serial.write(msp_request)
                data = self._standalone_serial.read(100)
                
                if len(data) >= 23 and data[:3] == b'$M>':
                    length = data[3]
                    payload = data[5:5 + length]
                    
                    if len(payload) >= 22:
                        channels = struct.unpack('<11H', payload[:22])
                        
                        roll, pitch = channels[0], channels[1]
                        
                        x = max(0, min(100, (roll - 1000) // 10))
                        y = max(0, min(100, 100 - (pitch - 1000) // 10))
                        
                        self.rc_stick = (x, y)
                        self.aux7_value = channels[10]
                        self._update_bbox_size()
                        
                        with self.rc_lock:
                            self.last_rc_values = list(channels[:4])
                        
            except Exception:
                pass
            
            time.sleep(0.02)
        
        if self._standalone_serial:
            self._standalone_serial.close()
    
    def _rc_reader_thread_integrated(self):
        """Потік читання RC (інтегрований режим з SharedSerialManager)"""
        while self.running:
            channels = self.serial_manager.read_rc_channels()
            
            if channels and len(channels) >= 11:
                roll, pitch = channels[0], channels[1]
                
                x = max(0, min(100, (roll - 1000) // 10))
                y = max(0, min(100, 100 - (pitch - 1000) // 10))
                
                self.rc_stick = (x, y)
                self.aux7_value = channels[10]
                self._update_bbox_size()
                
                with self.rc_lock:
                    self.last_rc_values = channels[:4]
            
            time.sleep(0.02)
    
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
        """Розрахунок нормалізованого offset від центру кадру"""
        x, y, w, h = bbox
        center_x = x + w / 2
        center_y = y + h / 2
        
        frame_center_x = self.resolution[0] / 2
        frame_center_y = self.resolution[1] / 2
        
        offset_x = (center_x - frame_center_x) / frame_center_x
        offset_y = (center_y - frame_center_y) / frame_center_y
        
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
    
    def set_tracking_callback(self, callback: Optional[Callable[[TrackingResult], None]]):
        """Встановлення callback для результатів трекінгу"""
        self.tracking_callback = callback
    
    def get_last_rc_values(self) -> list:
        """Отримання останніх RC значень"""
        with self.rc_lock:
            return self.last_rc_values.copy()
    
    def process_frame(self, frame: np.ndarray) -> TrackingResult:
        """
        Обробка одного кадру.
        
        Args:
            frame: Вхідний кадр
            
        Returns:
            TrackingResult з результатом трекінгу
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
        
        Args:
            headless: True для роботи без GUI
        """
        # Ініціалізація камери
        if not self._init_camera():
            print("[SCOPE] Помилка ініціалізації камери")
            return
        
        self._start_camera()
        self.running = True
        self._prev_bbox_size = self.current_bbox_size.copy()
        
        if not headless:
            cv2.namedWindow("Scope Tracker")
        
        # Запуск RC reader
        if self.serial_manager:
            self.rc_thread = threading.Thread(target=self._rc_reader_thread_integrated, daemon=True)
        else:
            self.rc_thread = threading.Thread(target=self._rc_reader_thread_standalone, daemon=True)
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
        
        print("[SCOPE] Трекінг запущено")
        print(f"[SCOPE] Роздільна здатність: {self.resolution}")
        print(f"[SCOPE] Headless: {headless}")
        
        try:
            while self.running:
                frame = self._capture_frame()
                if frame is None:
                    continue
                
                frame_clean = frame.copy()
                
                # Обробка кадру
                tracking_result = self.process_frame(frame)
                
                # Callback
                if self.tracking_callback and tracking_result:
                    self.tracking_callback(tracking_result)
                
                # Візуалізація
                if not headless and tracking_result.is_valid():
                    display_bbox = tracking_result.bbox
                    preview = self._draw_bbox_preview(frame_clean, display_bbox)
                    
                    x, y, w, h = display_bbox
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Мініатюра
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
                    # Конвертуємо RGB → BGR для cv2.imshow
                    display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Scope Tracker", display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        # Реініціалізація трекера
                        w, h = 33, 33
                        x = (self.resolution[0] - w) // 2
                        y = (self.resolution[1] - h) // 2
                        self.tracker.init(frame, (x, y, w, h))
                        print("[SCOPE] Трекер реініціалізовано")
                        
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self._stop_camera()
            if not headless:
                cv2.destroyAllWindows()
            print("[SCOPE] Завершено")
    
    def stop(self):
        """Зупинка контролера"""
        self.running = False


# ============================================================================
# Standalone режим
# ============================================================================

def main():
    """Standalone запуск"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Scope Controller - Object Tracking')
    parser.add_argument('--resolution', default='640x480', help='Camera resolution (WxH)')
    parser.add_argument('--headless', action='store_true', help='Run without GUI')
    
    args = parser.parse_args()
    
    res = tuple(map(int, args.resolution.split('x')))
    
    controller = ScopeController(
        serial_manager=None,  # Standalone
        resolution=res
    )
    
    # Приклад callback
    def on_tracking(result: TrackingResult):
        if result.is_valid():
            print(f"\r[TRACKING] Offset: {result.center_offset[0]:+.2f}, {result.center_offset[1]:+.2f}", end='')
    
    # controller.set_tracking_callback(on_tracking)
    
    controller.run(headless=args.headless)


if __name__ == "__main__":
    main()