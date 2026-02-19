import serial
import struct
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=0.5)

autopilot_state = {
    'bee_state': 'OFF'
}

def command_telemetry_mode_change():
    ser.write(b'$M<\x00\x69\x69')
    data = ser.read(100)
    
    if len(data) >= 19:
        channel_value = struct.unpack('<7H', data[5:19])[6]
        autopilot_mode = autopilot_state['bee_state']
        
        if abs(channel_value - 1000) < 100:
            autopilot_mode = 'OFF'
        elif abs(channel_value - 1500) < 100:
            autopilot_mode = 'CAUGHT'
        elif abs(channel_value - 2000) < 100:
            autopilot_mode = 'KILL'
        
        if autopilot_mode != autopilot_state['bee_state']:
            autopilot_state['bee_state'] = autopilot_mode
            print(f"Режим змінено на: {autopilot_mode}")

while True:
    command_telemetry_mode_change()
    time.sleep(0.1)