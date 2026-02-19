#!/usr/bin/env python3
"""
RC-controlled tracker with offset correction and bbox size control via AUX7.
"""

import cv2
import threading
import numpy as np
from typing import Tuple
from tracker import MILTracker
from picamera2 import Picamera2

class RCTrackerController:
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480),
                 bbox_size: Tuple[int, int] = (100, 100)):
        self.resolution = resolution
        self.initial_bbox_size = bbox_size
        self.current_bbox_size = list(bbox_size)  # Поточний розмір bbox
        
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            raw={"size": (1640, 1232)},
            main={"format": "RGB888", "size": resolution}
        )
        self.camera.configure(config)
        
        self.tracker = MILTracker()
        self.running = False
        self.rc_stick = (50, 50)
        self.aux7_value = 1500  # Значення AUX7 (11 канал)
        self.deadzone = 3
        
        # Параметри масштабування bbox через AUX7 (зменшено у 3 рази)
        self.bbox_min_size = (17, 17)    # Мінімальний розмір (50/3 ≈ 17)
        self.bbox_max_size = (100, 100)  # Максимальний розмір (300/3 = 100)
        
        # Розмір мініатюри зверху справа
        self.preview_size = (150, 150)
        
    def _rc_reader_thread(self):
        import serial
        import struct
        
        ser = serial.Serial('/dev/serial0', 115200, timeout=0.01)
        msp_request = b'$M<\x00\x69\x69'
        
        while self.running:
            try:
                ser.write(msp_request)
                data = ser.read(100)
                
                if len(data) >= 23 and data[:3] == b'$M>':
                    length = data[3]
                    payload = data[5:5 + length]
                    
                    if len(payload) >= 22:
                        # Читаємо Roll і Pitch для керування offset
                        roll, pitch = struct.unpack('<HH', payload[0:4])
                        
                        x = max(0, min(100, (roll - 1000) // 10))
                        # ЗАДАЧА 2: Реверс команд вгору/вниз (інвертуємо pitch)
                        y = max(0, min(100, 100 - (pitch - 1000) // 10))
                        
                        self.rc_stick = (x, y)
                        
                        # Читаємо AUX7 (11 канал) для керування розміром bbox
                        aux7 = struct.unpack('<H', payload[20:22])[0]
                        self.aux7_value = aux7
                        
                        # Перераховуємо розмір bbox
                        self._update_bbox_size()
                    
            except Exception as e:
                pass
                
        ser.close()
    
    def _update_bbox_size(self):
        """Оновлює розмір bbox на основі значення AUX7 (1000-2000)"""
        # Нормалізуємо AUX7 до діапазону 0-1
        aux7_normalized = max(0, min(1, (self.aux7_value - 1000) / 1000))
        
        # Лінійна інтерполяція між мінімальним і максимальним розміром
        new_w = int(self.bbox_min_size[0] + 
                   (self.bbox_max_size[0] - self.bbox_min_size[0]) * aux7_normalized)
        new_h = int(self.bbox_min_size[1] + 
                   (self.bbox_max_size[1] - self.bbox_min_size[1]) * aux7_normalized)
        
        self.current_bbox_size = [new_w, new_h]
    
    def _get_offset(self) -> Tuple[int, int]:
        x, y = self.rc_stick
        
        if abs(x - 50) <= self.deadzone and abs(y - 50) <= self.deadzone:
            return (0, 0)
            
        # ЗАДАЧА 1: Зменшена чутливість у 3 рази (було *5, стало *5/3 ≈ 1.67)
        offset_x = int((x - 50) * 5 / 3)
        offset_y = int((y - 50) * 5 / 3)
        
        return (offset_x, offset_y)
    
    def _draw_bbox_preview(self, frame_original, bbox):
        """
        Відображає збільшену мініатюру bbox зверху справа з cv2.INTER_LINEAR
        """
        x, y, w, h = bbox
        
        # Перевірка меж
        x = max(0, min(x, frame_original.shape[1] - 1))
        y = max(0, min(y, frame_original.shape[0] - 1))
        w = max(1, min(w, frame_original.shape[1] - x))
        h = max(1, min(h, frame_original.shape[0] - y))
        
        # Витягуємо ROI з оригінального кадру (без хрестика)
        roi = frame_original[y:y+h, x:x+w].copy()
        
        if roi.size == 0:
            return None
        
        # Масштабуємо до фіксованого розміру з INTER_LINEAR
        preview = cv2.resize(roi, self.preview_size, interpolation=cv2.INTER_LINEAR)
        
        return preview
    
    def run(self):
        self.camera.start()
        self.running = True
        
        cv2.namedWindow("RC Tracker")
        
        rc_thread = threading.Thread(target=self._rc_reader_thread, daemon=True)
        rc_thread.start()
        
        # Ініціалізація (зменшено у 3 рази)
        frame = self.camera.capture_array()
        w, h = 33, 33  # 100/3 ≈ 33
        x = (self.resolution[0] - w) // 2
        y = (self.resolution[1] - h) // 2
        self.tracker.init(frame, (x, y, w, h))
        
        prev_bbox_size = self.current_bbox_size.copy()
        
        try:
            while self.running:
                frame = self.camera.capture_array()
                frame_clean = frame.copy()  # Зберігаємо чистий кадр для мініатюри
                
                # Трекінг
                result = self.tracker.update(frame)
                
                if result.bbox:
                    # Отримуємо offset
                    offset_x, offset_y = self._get_offset()
                    
                    # Перевіряємо, чи змінився розмір bbox
                    bbox_size_changed = (prev_bbox_size[0] != self.current_bbox_size[0] or 
                                        prev_bbox_size[1] != self.current_bbox_size[1])
                    
                    # Якщо є offset або змінився розмір - реініціалізуємо
                    if offset_x != 0 or offset_y != 0 or bbox_size_changed:
                        x, y, w, h = result.bbox
                        
                        # Новий розмір
                        new_w, new_h = self.current_bbox_size
                        
                        # Застосовуємо offset, зберігаючи центр bbox
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        new_x = center_x - new_w // 2 + offset_x
                        new_y = center_y - new_h // 2 + offset_y
                        
                        # Обмеження в межах кадру
                        new_x = max(0, min(new_x, self.resolution[0] - new_w))
                        new_y = max(0, min(new_y, self.resolution[1] - new_h))
                        
                        new_bbox = (new_x, new_y, new_w, new_h)
                        self.tracker.init(frame, new_bbox)
                        display_bbox = new_bbox
                        
                        prev_bbox_size = self.current_bbox_size.copy()
                    else:
                        display_bbox = result.bbox
                    
                    # Спочатку створюємо мініатюру з чистого кадру
                    preview = self._draw_bbox_preview(frame_clean, display_bbox)
                    
                    # Потім відображаємо bbox на основному кадрі
                    x, y, w, h = display_bbox
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Відображаємо мініатюру зверху справа
                    if preview is not None:
                        preview_x = frame.shape[1] - self.preview_size[0] - 10
                        preview_y = 10
                        
                        # Накладаємо напівпрозорий фон
                        overlay = frame.copy()
                        cv2.rectangle(overlay, 
                                     (preview_x - 5, preview_y - 5),
                                     (preview_x + self.preview_size[0] + 5, preview_y + self.preview_size[1] + 5),
                                     (0, 0, 0), -1)
                        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
                        
                        # Накладаємо мініатюру
                        frame[preview_y:preview_y+self.preview_size[1], 
                              preview_x:preview_x+self.preview_size[0]] = preview
                        
                        # Рамка навколо мініатюри
                        cv2.rectangle(frame, 
                                     (preview_x, preview_y),
                                     (preview_x + self.preview_size[0], preview_y + self.preview_size[1]),
                                     (0, 255, 0), 2)
                    
                    # Інформація на екрані
                    cv2.putText(frame, f"RC: {self.rc_stick[0]}, {self.rc_stick[1]}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(frame, f"AUX7: {self.aux7_value} Size: {w}x{h}", 
                               (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow("RC Tracker", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                          
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.camera.stop()
            self.camera.close()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    controller = RCTrackerController(resolution=(640, 480), bbox_size=(100, 100))
    controller.run()