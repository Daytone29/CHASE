import serial
import struct
import time
import threading


class CaughtMode:
    """Режим CAUGHT - передача overridden RC значень на польотний контролер"""
    
    def __init__(self, serial_port, serial_lock):
        self.ser = serial_port
        self.lock = serial_lock
        self.running = False
        self.thread = None
        self.rc_values = [1500, 1500, 1500, 1500]  # [Roll, Pitch, Yaw, Throttle]
        
    def set_rc_values(self, rc_data):
        """
        Встановлення RC значень
        rc_data може бути:
        - список/tuple: [roll, pitch, yaw, throttle]
        - словник: {'roll': val, 'pitch': val, 'yaw': val, 'throttle': val}
        """
        if not rc_data:
            return
            
        # Якщо це словник - конвертуємо в список
        if isinstance(rc_data, dict):
            rc_array = [
                rc_data.get('roll', 1500),
                rc_data.get('pitch', 1500),
                rc_data.get('yaw', 1500),
                rc_data.get('throttle', 1500)
            ]
        else:
            rc_array = rc_data
        
        # Перевірка та обмеження значень
        if len(rc_array) >= 4:
            self.rc_values = [max(1000, min(2000, int(val))) for val in rc_array[:4]]
            # ВИВІД ЗНАЧЕНЬ В ТЕРМІНАЛ
            print(f"\n[CAUGHT] Збережені значення з OFF:")
            print(f"         R:{self.rc_values[0]}  P:{self.rc_values[1]}  Y:{self.rc_values[2]}  T:{self.rc_values[3]}")
    
    def _build_msp_packet(self):
        """Формування MSP_SET_RAW_RC пакету"""
        payload = struct.pack('<4H', *self.rc_values)
        cmd = 200
        checksum = len(payload) ^ cmd
        for byte in payload:
            checksum ^= byte
        return b'$M<' + bytes([len(payload), cmd]) + payload + bytes([checksum])
    
    def _send_override(self):
        """Відправка MSP Override команди"""
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(self._build_msp_packet())
                time.sleep(0.001)
            except Exception as e:
                print(f"[CAUGHT] Помилка MSP: {e}")
    
    def _override_loop(self):
        """Циклічна відправка override команд (50Hz)"""
        print(f"[CAUGHT] Старт - R:{self.rc_values[0]} P:{self.rc_values[1]} "
              f"Y:{self.rc_values[2]} T:{self.rc_values[3]}")
        
        while self.running:
            self._send_override()
            time.sleep(0.02)
    
    def start(self):
        """Запуск режиму CAUGHT"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._override_loop, daemon=True)
            self.thread.start()
    
    def stop(self):
        """Зупинка режиму CAUGHT"""
        if self.running:
            print("[CAUGHT] Зупинка")
            self.running = False
            if self.thread:
                self.thread.join(timeout=1.0)