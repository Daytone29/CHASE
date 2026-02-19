#!/usr/bin/env python3

import struct
import time
import threading

class RCReader:
    def __init__(self, ser, serial_lock):
        self.ser = ser
        self.serial_lock = serial_lock
        self.last_values = {'roll': None, 'pitch': None, 'throttle': None, 'yaw': None}
        self.running = False
    
    def get_rc_channels(self):
        with self.serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(b'$M<\x00\x69\x69')
            data = self.ser.read(22)
        
        if len(data) >= 13 and data[:3] == b'$M>':
            payload = data[5:5 + data[3]]
            roll, pitch, throttle, yaw = struct.unpack('<HHHH', payload[0:8])
            self.last_values = {'roll': roll, 'pitch': pitch, 'throttle': throttle, 'yaw': yaw}
            return roll, pitch, throttle, yaw
        
        return None, None, None, None
    
    def _run_loop(self):
        print("\n[OFF MODE] Активовано")
        
        while self.running:
            roll, pitch, throttle, yaw = self.get_rc_channels()
            
            if roll is not None:
                print(f"\rT:{throttle:4d} Y:{yaw:4d} R:{roll:4d} P:{pitch:4d}", 
                      end='', flush=True)
            
            time.sleep(0.02)
        
        v = self.last_values
        print(f"\n[OFF MODE] Вимкнено - R:{v['roll']} P:{v['pitch']} T:{v['throttle']} Y:{v['yaw']}\n")
    
    def start(self):
        self.running = True
        threading.Thread(target=self._run_loop, daemon=True).start()
    
    def stop(self):
        self.running = False
        time.sleep(0.05)
        return self.last_values.copy()