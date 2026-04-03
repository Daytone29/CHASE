import cv2
import threading
import numpy as np
import libcamera
from typing import Tuple
from tracker import MILTracker
from picamera2 import Picamera2
import sys
import os
import time

# Import headless_mode config
sys.path.append('/home/obriy/CHASE/CLEAN_NORMAL_STRUCT/autopilot_bee_ept')
try:
    import definitions as vars
    HEADLESS_MODE = vars.headless_mode
except:
    HEADLESS_MODE = True  # Default to headless if import fails

class RCTrackerController:
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480),
                 bbox_size: Tuple[int, int] = (100, 100), autopilot_state=None,
                 on_tracking_stopped=None):
        self.resolution = resolution
        self.initial_bbox_size = bbox_size
        self.current_bbox_size = list(bbox_size)  # Поточний розмір bbox

        self.camera = None
        self.tracker = None
        self.running = False
        self.session_active = False
        self.rc_stick = (50, 50)
        self.aux7_value = 1500  # Значення AUX7 (11 канал)
        self.deadzone = 3
        self.autopilot_state = autopilot_state  # Посилання на стан автопілота
        self.on_tracking_stopped = on_tracking_stopped
        self.manual_speed_px_per_sec = 420.0
        self.offset_remainder = [0.0, 0.0]
        self.tracking_requested = threading.Event()
        self.shutdown_requested = threading.Event()
        self.active_attack_bbox = None
        self.attack_tracker_initialized = False
        
        # Параметри масштабування bbox через AUX7 (зменшено у 3 рази)
        self.bbox_min_size = (17, 17)    # Мінімальний розмір (50/3 ≈ 17)
        self.bbox_max_size = (100, 100)  # Максимальний розмір (300/3 = 100)
        
        # Розмір мініатюри зверху справа
        self.preview_size = (150, 150)
        self.window_name = "RC Tracker"
        self.window_mode = None

    def _is_attack_mode(self) -> bool:
        if not self.autopilot_state:
            return False

        return self.autopilot_state.get('bee_state') == 'ATACK'

    def start_tracking(self):
        self.tracking_requested.set()

    def stop_tracking(self):
        self.tracking_requested.clear()

    def shutdown(self):
        self.stop_tracking()
        self.shutdown_requested.set()

    def is_tracking_requested(self) -> bool:
        return self.tracking_requested.is_set()

    def _should_stop_tracking(self) -> bool:
        if not self.autopilot_state:
            return False

        return self.autopilot_state.get('bee_state') == 'OFF'
        
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
    
    def _get_offset(self, frame_time_delta: float) -> Tuple[int, int]:
        x, y = self.rc_stick
        centered_x = x - 50
        centered_y = y - 50
        
        if abs(centered_x) <= self.deadzone and abs(centered_y) <= self.deadzone:
            self.offset_remainder = [0.0, 0.0]
            return (0, 0)

        normalized_x = centered_x / 50.0
        normalized_y = centered_y / 50.0

        move_x = normalized_x * self.manual_speed_px_per_sec * frame_time_delta + self.offset_remainder[0]
        move_y = normalized_y * self.manual_speed_px_per_sec * frame_time_delta + self.offset_remainder[1]

        offset_x = int(move_x)
        offset_y = int(move_y)
        self.offset_remainder = [move_x - offset_x, move_y - offset_y]
        
        return (offset_x, offset_y)

    def _calculate_center_error(self, bbox: Tuple[int, int, int, int]) -> Tuple[float, float]:
        x, y, w, h = bbox
        bbox_center_x = x + w / 2
        bbox_center_y = y + h / 2

        screen_center_x = self.resolution[0] / 2
        aim_offset_y = self.resolution[1] * getattr(vars, 'attack_aim_offset_y', 0.0)
        screen_center_y = self.resolution[1] / 2 + aim_offset_y

        error_x = (bbox_center_x - screen_center_x) / screen_center_x
        error_y = (bbox_center_y - screen_center_y) / screen_center_y
        return (error_x, error_y)

    def _update_tracking_state(self, bbox, confidence: float = 0.0):
        if not self.autopilot_state:
            return

        if bbox is None:
            self.autopilot_state['target_locked'] = False
            self.autopilot_state['tracking_bbox'] = None
            self.autopilot_state['attack_error_x'] = 0.0
            self.autopilot_state['attack_error_y'] = 0.0
            self.autopilot_state['target_size'] = 0.0
            return

        error_x, error_y = self._calculate_center_error(bbox)
        x, y, w, h = bbox
        self.autopilot_state['target_locked'] = confidence > 0.0
        self.autopilot_state['tracking_bbox'] = bbox
        self.autopilot_state['attack_error_x'] = error_x
        self.autopilot_state['attack_error_y'] = error_y
        self.autopilot_state['target_size'] = float(w * h)

    def _update_capture_state(self, bbox):
        if not self.autopilot_state:
            return

        self.autopilot_state['target_locked'] = False
        self.autopilot_state['tracking_bbox'] = bbox
        self.autopilot_state['attack_error_x'] = 0.0
        self.autopilot_state['attack_error_y'] = 0.0
        self.autopilot_state['target_size'] = 0.0

    def _build_center_bbox(self, width: int, height: int) -> Tuple[int, int, int, int]:
        x = (self.resolution[0] - width) // 2
        y = (self.resolution[1] - height) // 2
        return (x, y, width, height)

    def _reset_attack_tracker(self):
        if self.tracker is not None:
            self.tracker.reset()

        self.active_attack_bbox = None
        self.attack_tracker_initialized = False
    
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

    def _initialize_window(self):
        cv2.startWindowThread()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if getattr(vars, 'tracking_fullscreen', False):
            cv2.setWindowProperty(
                self.window_name,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN,
            )

    def _get_window_mode(self) -> str:
        return 'ATACK' if self._is_attack_mode() else 'CAPTURE'

    def _get_window_name_for_mode(self, mode: str) -> str:
        if mode == 'ATACK':
            return 'RC Tracker - ATACK'

        return 'RC Tracker - CAPTURE'

    def _refresh_window_for_mode(self):
        next_mode = self._get_window_mode()
        if self.window_mode == next_mode:
            return

        if self.window_mode is not None:
            self._cleanup_window()

        self.window_mode = next_mode
        self.window_name = self._get_window_name_for_mode(next_mode)
        self._initialize_window()

    def _initialize_runtime_resources(self):
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            raw={"size": (1640, 1232)},
            main={"format": "RGB888", "size": self.resolution},
            transform=libcamera.Transform(hflip=1, vflip=1),
        )
        self.camera.configure(config)
        self.tracker = MILTracker()

    def _cleanup_runtime_resources(self):
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass

            try:
                self.camera.close()
            except Exception:
                pass

        self.camera = None
        self.tracker = None
        time.sleep(0.05)

    def _cleanup_window(self):
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass

        cv2.destroyAllWindows()
        cv2.waitKey(1)
        self.window_mode = None
        time.sleep(0.05)

    def _finalize_tracking_session(self):
        was_active = self.session_active
        self.session_active = False
        self.running = False
        self.offset_remainder = [0.0, 0.0]
        self._reset_attack_tracker()
        self._update_tracking_state(None)
        self._cleanup_runtime_resources()
        self._cleanup_window()

        if was_active and self.on_tracking_stopped:
            self.on_tracking_stopped()

    def _run_tracking_session(self):
        try:
            self._initialize_runtime_resources()
            self.camera.start()
            self.running = True
            self.session_active = True
            self._refresh_window_for_mode()

            self._reset_attack_tracker()
            last_frame_time = time.monotonic()

            while self.tracking_requested.is_set() and not self.shutdown_requested.is_set():
                attack_mode = self._is_attack_mode()
                self._refresh_window_for_mode()

                if self.autopilot_state:
                    if self._should_stop_tracking():
                        self.stop_tracking()
                        break

                    roll = self.autopilot_state.get('roll', 1500)
                    pitch = self.autopilot_state.get('pitch', 1500)
                    
                    x_rc = max(0, min(100, (roll - 1000) // 10))
                    y_rc = max(0, min(100, 100 - (pitch - 1000) // 10))

                    self.rc_stick = (x_rc, y_rc)
                    self.aux7_value = self.autopilot_state.get('aux7', 1500)
                    if not attack_mode:
                        self._update_bbox_size()
                
                frame = self.camera.capture_array()
                current_frame_time = time.monotonic()
                frame_time_delta = min(current_frame_time - last_frame_time, 0.1)
                last_frame_time = current_frame_time
                frame_clean = frame.copy()

                display_bbox = self._build_center_bbox(*self.current_bbox_size)
                confidence = 0.0

                if attack_mode:
                    if not self.attack_tracker_initialized:
                        self.tracker.init(frame, display_bbox)
                        self.active_attack_bbox = display_bbox
                        self.attack_tracker_initialized = True
                        confidence = 1.0
                    else:
                        result = self.tracker.update(frame)
                        if result.bbox:
                            display_bbox = result.bbox
                            self.active_attack_bbox = display_bbox
                            confidence = result.confidence
                        else:
                            self.active_attack_bbox = None
                            self._update_tracking_state(None)

                    if self.active_attack_bbox is not None:
                        display_bbox = self.active_attack_bbox
                        self._update_tracking_state(display_bbox, confidence)
                    else:
                        display_bbox = self._build_center_bbox(*self.current_bbox_size)
                else:
                    if self.attack_tracker_initialized:
                        self._reset_attack_tracker()

                    self._update_capture_state(display_bbox)

                preview = self._draw_bbox_preview(frame_clean, display_bbox)
                x, y, w, h = display_bbox
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                if attack_mode and self.active_attack_bbox is not None:
                    error_x, error_y = self._calculate_center_error(display_bbox)
                    frame_center_x = self.resolution[0] // 2
                    frame_center_y = self.resolution[1] // 2
                    cv2.drawMarker(frame, (frame_center_x, frame_center_y), (0, 0, 255), cv2.MARKER_CROSS, 18, 1)
                    cv2.putText(frame, f"ATACK err: {error_x:+.2f}, {error_y:+.2f}",
                               (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if preview is not None:
                    preview_x = frame.shape[1] - self.preview_size[0] - 10
                    preview_y = 10
                    overlay = frame.copy()
                    cv2.rectangle(overlay,
                                 (preview_x - 5, preview_y - 5),
                                 (preview_x + self.preview_size[0] + 5, preview_y + self.preview_size[1] + 5),
                                 (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
                    frame[preview_y:preview_y+self.preview_size[1],
                          preview_x:preview_x+self.preview_size[0]] = preview
                    cv2.rectangle(frame,
                                 (preview_x, preview_y),
                                 (preview_x + self.preview_size[0], preview_y + self.preview_size[1]),
                                 (0, 255, 0), 2)

                mode_label = "CAPTURE" if not attack_mode else "ATACK"
                cv2.putText(frame, f"MODE: {mode_label}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"AUX7: {self.aux7_value} Size: {w}x{h}",
                           (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow(self.window_name, frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop_tracking()
                    break
        finally:
            self._finalize_tracking_session()
    
    def run(self):
        try:
            while not self.shutdown_requested.is_set():
                if not self.tracking_requested.wait(timeout=0.1):
                    continue

                self._run_tracking_session()
                          
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
            self._finalize_tracking_session()


if __name__ == "__main__":
    controller = RCTrackerController(resolution=(640, 480), bbox_size=(100, 100))
    controller.run()