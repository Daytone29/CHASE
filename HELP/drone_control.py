#!/usr/bin/env python3
"""
Drone Controller - Керування дроном через MSP
PID автонаведення на ціль
"""

import time
import threading
from typing import Optional, Tuple, Dict
from dataclasses import dataclass

from msp import MSPProtocol, RCChannels


@dataclass
class PIDConfig:
    """PID параметри"""
    kp: float = 0.4
    ki: float = 0.01
    kd: float = 0.15
    max_output: float = 300  # Максимальне відхилення від 1500


class PIDController:
    """Простий PID контролер"""
    
    def __init__(self, config: PIDConfig):
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.max_output = config.max_output
        
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()
    
    def update(self, error: float) -> float:
        """
        Оновлення PID
        
        Args:
            error: Помилка (-1.0 до 1.0)
        Returns:
            Output: Корекція для RC каналу
        """
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        
        if dt <= 0 or dt > 1.0:
            return 0.0
        
        # P
        p = self.kp * error
        
        # I (з обмеженням)
        self._integral += error * dt
        self._integral = max(-1.0, min(1.0, self._integral))
        i = self.ki * self._integral
        
        # D
        derivative = (error - self._prev_error) / dt
        d = self.kd * derivative
        self._prev_error = error
        
        # Сума
        output = (p + i + d) * self.max_output
        return max(-self.max_output, min(self.max_output, output))
    
    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = time.time()


class DroneController:
    """
    Контролер дрона з автонаведенням
    
    Логіка:
    1. Читаємо поточну позицію цілі
    2. Обчислюємо помилку (відхилення від центру)
    3. PID генерує корекції для Yaw та Throttle
    4. Відправляємо через MSP Override
    
    Налаштування Betaflight:
    - set msp_override_channels_mask = 15  # Канали 1-4
    - set msp_override_failsafe = ON
    """
    
    def __init__(self, port: str = "/dev/serial0", baudrate: int = 115200):
        self.msp = MSPProtocol(port, baudrate)
        self.is_connected = False
        
        # PID контролери
        self.pid_yaw = PIDController(PIDConfig(kp=0.5, ki=0.01, kd=0.2, max_output=250))
        self.pid_throttle = PIDController(PIDConfig(kp=0.3, ki=0.005, kd=0.1, max_output=150))
        
        # Базові значення RC
        self.base_throttle = 1500  # Hovering throttle
        self.base_yaw = 1500
        self.base_pitch = 1500
        self.base_roll = 1500
        
        # Deadzone
        self.deadzone = 0.05  # 5% від центру
        
        # Стан
        self._override_active = False
        self._target_offset: Optional[Tuple[float, float]] = None
        self._target_time = 0.0
        self._target_timeout = 1.0
        
        # Control loop
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._control_rate = 50  # Hz
        
        # Останні команди
        self.last_command: Dict = {}
        self._last_rc: Optional[RCChannels] = None
    
    def connect(self) -> bool:
        """Підключення до FC"""
        if self.msp.connect():
            self.is_connected = True
            
            # Читаємо початкові RC
            rc = self.msp.get_rc_channels()
            if rc:
                self._last_rc = rc
                # Запам'ятовуємо throttle як базовий
                if 1200 < rc.throttle < 1800:
                    self.base_throttle = rc.throttle
            
            return True
        return False
    
    def disconnect(self):
        """Відключення"""
        self.stop_override()
        self.msp.disconnect()
        self.is_connected = False
    
    def start_override(self) -> bool:
        """Почати RC Override"""
        if not self.is_connected:
            return False
        
        if self._override_active:
            return True
        
        self._override_active = True
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        
        print("✅ RC Override started")
        return True
    
    def stop_override(self):
        """Зупинити Override"""
        self._running = False
        self._override_active = False
        
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        
        # Скидання PID
        self.pid_yaw.reset()
        self.pid_throttle.reset()
        
        print("✅ RC Override stopped")
    
    def track_target(self, bbox: Tuple[int, int, int, int], 
                     frame_size: Tuple[int, int]):
        """
        Оновити позицію цілі
        
        Args:
            bbox: (x, y, w, h)
            frame_size: (height, width)
        """
        if not bbox:
            self._target_offset = None
            return
        
        x, y, w, h = bbox
        fh, fw = frame_size
        
        # Центр цілі в нормалізованих координатах (-1 до 1)
        # 0 = центр кадру
        cx = (x + w / 2) / fw * 2 - 1  # -1 (ліво) до +1 (право)
        cy = (y + h / 2) / fh * 2 - 1  # -1 (верх) до +1 (низ)
        
        self._target_offset = (cx, cy)
        self._target_time = time.time()
    
    def hover(self):
        """Режим зависання"""
        self._target_offset = None
    
    def _control_loop(self):
        """Головний цикл керування"""
        interval = 1.0 / self._control_rate
        
        while self._running:
            t0 = time.time()
            
            try:
                self._control_step()
            except Exception as e:
                print(f"⚠️ Control error: {e}")
            
            # Точний timing
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)
    
    def _control_step(self):
        """Один крок керування"""
        
        # Читаємо поточні RC (для passthrough)
        current_rc = self.msp.get_rc_channels()
        if current_rc:
            self._last_rc = current_rc
        
        # Базові значення
        rc = RCChannels(
            roll=self.base_roll,
            pitch=self.base_pitch,
            throttle=self.base_throttle,
            yaw=self.base_yaw,
            aux1=self._last_rc.aux1 if self._last_rc else 1000,
            aux2=self._last_rc.aux2 if self._last_rc else 1000,
            aux3=self._last_rc.aux3 if self._last_rc else 1000,
            aux4=self._last_rc.aux4 if self._last_rc else 1000,
        )
        
        # Перевірка таймауту цілі
        target_valid = (
            self._target_offset is not None and
            (time.time() - self._target_time) < self._target_timeout
        )
        
        if target_valid:
            ox, oy = self._target_offset
            
            # Yaw - горизонтальне наведення
            # Якщо ціль справа (ox > 0), крутимо вправо (yaw > 1500)
            if abs(ox) > self.deadzone:
                yaw_correction = self.pid_yaw.update(ox)
                rc.yaw = int(self.base_yaw + yaw_correction)
            
            # Throttle - вертикальне наведення
            # Якщо ціль знизу (oy > 0), піднімаємось (throttle > base)
            if abs(oy) > self.deadzone:
                throttle_correction = self.pid_throttle.update(oy)
                rc.throttle = int(self.base_throttle + throttle_correction)
            
            self.last_command = {
                'yaw': rc.yaw,
                'throttle': rc.throttle,
                'target_x': ox,
                'target_y': oy,
                'mode': 'tracking'
            }
        else:
            # Hover mode
            self.last_command = {
                'yaw': rc.yaw,
                'throttle': rc.throttle,
                'mode': 'hover'
            }
        
        # Обмеження
        rc.roll = max(1000, min(2000, rc.roll))
        rc.pitch = max(1000, min(2000, rc.pitch))
        rc.throttle = max(1000, min(2000, rc.throttle))
        rc.yaw = max(1000, min(2000, rc.yaw))
        
        # Відправка
        if self._override_active:
            self.msp.set_rc_channels(rc)
    
    def get_status(self) -> Dict:
        """Статус контролера"""
        return {
            'connected': self.is_connected,
            'override_active': self._override_active,
            'has_target': self._target_offset is not None,
            'rc': self._last_rc.to_dict() if self._last_rc else None,
            'last_command': self.last_command
        }
    
    def set_base_throttle(self, value: int):
        """Встановити базовий throttle"""
        self.base_throttle = max(1000, min(2000, value))


# =============================================================================
# Crosshair Controller
# =============================================================================

class CrosshairController:
   
    # Параметри RC каналів
    DEADZONE = 50          # PWM deadzone (+/- від центру)
    SPEED = 12             # пікселів за оновлення
    AUX_THRESHOLD = 1700   # Порог для перемикачів (AUX каналів)
    
    def __init__(self, frame_size: Tuple[int, int] = (640, 480)):
        self.width, self.height = frame_size
        self.x = self.width // 2      # Центр X
        self.y = self.height // 2     # Центр Y
        self.locked = False
        
        # Edge detection для кнопок
        self._aux1_was_high = False
        self._aux2_was_high = False
        
        # Callbacks для подій
        self.on_lock = None   # (x, y) -> None
        self.on_reset = None  # () -> None
    
    def set_frame_size(self, width: int, height: int):
        """Оновити розмір кадру"""
        # Масштабування позиції
        if self.width > 0 and self.height > 0:
            self.x = int(self.x * width / self.width)
            self.y = int(self.y * height / self.height)
        self.width = width
        self.height = height
        
        # Обмеження
        self.x = max(20, min(self.x, width - 20))
        self.y = max(20, min(self.y, height - 20))
    
    def update(self, rc: RCChannels):
        """
        Оновити позицію прицілу на основі RC даних
        
        Отримує положення правого стіка (Roll/Pitch) та обновлює
        позицію прицілу на екрані. Roll контролює X, Pitch контролює Y.
        
        Args:
            rc: RCChannels - RC дані від FC
        """
        if self.locked:
            # Якщо приціл залучен, тільки перевіримо reset
            self._check_reset(rc)
            return
        
        # ===== ПЕРЕМІЩЕННЯ ПРИЦІЛУ =====
        # Roll (CH1) - горизонтальне переміщення
        roll_offset = rc.roll - 1500
        if abs(roll_offset) > self.DEADZONE:
            # Нормалізуємо до [-1, 1], потім множимо на SPEED
            speed = (roll_offset / 500) * self.SPEED
            self.x += int(speed)
        
        # Pitch (CH2) - вертикальне переміщення (інвертовано)
        pitch_offset = rc.pitch - 1500
        if abs(pitch_offset) > self.DEADZONE:
            # Інверсія: більший pitch = нижче на екрані
            speed = (pitch_offset / 500) * self.SPEED
            self.y -= int(speed)  # Мінус для інверсії
        
        # Обмеження в межах кадру
        self.x = max(20, min(self.x, self.width - 20))
        self.y = max(20, min(self.y, self.height - 20))
        
        # Перевірка кнопок
        self._check_lock(rc)
        self._check_reset(rc)
    
    def _check_lock(self, rc: RCChannels):
        """Перевірити натискання trigger (AUX1)"""
        aux1_high = rc.aux1 > self.AUX_THRESHOLD
        
        # Edge detection - спрацьовує тільки на перехід з LOW в HIGH
        if aux1_high and not self._aux1_was_high:
            self.locked = True
            print(f"🎯 Приціл залучен на: ({self.x}, {self.y})")
            if self.on_lock:
                self.on_lock(self.x, self.y)
        
        self._aux1_was_high = aux1_high
    
    def _check_reset(self, rc: RCChannels):
        """Перевірити натискання reset (AUX2)"""
        aux2_high = rc.aux2 > self.AUX_THRESHOLD
        
        # Edge detection
        if aux2_high and not self._aux2_was_high:
            self.reset()
            print("🔄 Приціл скинутий")
            if self.on_reset:
                self.on_reset()
        
        self._aux2_was_high = aux2_high
    
    def reset(self):
        """Скинути приціл у центр"""
        self.locked = False
        self.x = self.width // 2
        self.y = self.height // 2
        self._aux1_was_high = False
        self._aux2_was_high = False
    
    def unlock(self):
        """Розблокувати приціл"""
        self.locked = False
    
    def set_position(self, x: int, y: int):
        """Встановити позицію прицілу"""
        self.x = max(20, min(x, self.width - 20))
        self.y = max(20, min(y, self.height - 20))
    
    def get_position(self) -> Tuple[int, int]:
        """Отримати поточну позицію прицілу"""
        return (self.x, self.y)
    
    def is_locked(self) -> bool:
        """Чи приціл залучен"""
        return self.locked
        return (self.x, self.y)
    
    def draw(self, frame):
        """Малювання прицілу"""
        try:
            import cv2
        except ImportError:
            return frame
        
        color = (0, 255, 0) if self.locked else (0, 255, 255)
        size = 20
        
        # Хрестик
        cv2.line(frame, (self.x - size, self.y), (self.x + size, self.y), color, 2)
        cv2.line(frame, (self.x, self.y - size), (self.x, self.y + size), color, 2)
        
        # Коло
        cv2.circle(frame, (self.x, self.y), size, color, 2)
        cv2.circle(frame, (self.x, self.y), 3, color, -1)
        
        # Статус
        status = "LOCKED" if self.locked else "AIM"
        cv2.putText(frame, status, (self.x + size + 5, self.y + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return frame


# =============================================================================
# Тест
# =============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("  DRONE CONTROLLER TEST")
    print("=" * 50 + "\n")
    
    drone = DroneController("/dev/serial0", 115200)
    
    if not drone.connect():
        print("\n⚠️ Не вдалось підключитись")
        print("\nДля тесту без FC:")
        print("  Закоментуйте connect() та тестуйте логіку")
        exit(1)
    
    print("\n📡 Тест читання RC...")
    
    try:
        for i in range(5):
            status = drone.get_status()
            if status['rc']:
                rc = status['rc']
                print(f"  T={rc['throttle']} Y={rc['yaw']} P={rc['pitch']} R={rc['roll']}")
            time.sleep(0.2)
        
        print("\n✅ Тест Override (3 сек)...")
        drone.start_override()
        
        # Симуляція цілі справа-зверху
        drone.track_target((400, 100, 50, 50), (480, 640))  # (h, w)
        
        time.sleep(3)
        
        drone.stop_override()
        
    except KeyboardInterrupt:
        pass
    finally:
        drone.disconnect()
    
    print("\n✅ Тест завершено")
