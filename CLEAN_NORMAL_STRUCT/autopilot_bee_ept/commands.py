import serial
import time
import threading
import autopilot
import definitions as vars
import msp_helper as msp

command_delays =  {
    'caught': 1
}

command_target_ids = {
    'MSP_RAW_IMU': msp.MSP_RAW_IMU,
    'MSP_ANALOG': msp.MSP_ANALOG,
    'MSP_ALTITUDE': msp.MSP_ALTITUDE,
    'MSP_RC': msp.MSP_RC
}

serial_port = {}
serial_lock = threading.RLock()

def wait_for_execution(target, delay=0):
    if delay == 0:
        delay = command_delays.get(target)
    time.sleep(delay)

def get_target_id(target):
    return int(command_target_ids.get(target))
    
def connect():
    global serial_port
    with serial_lock:
        serial_port = serial.Serial(
            vars.companion_computer, 
            vars.companion_baud_rate, 
            timeout=0.1)

def disconnect():
    with serial_lock:
        serial_port.close()

def reboot():
    with serial_lock:
        disconnect()
        time.sleep(1)
        connect()

def set_row_rc(roll, pitch, yaw, throttle, servo_aux, await_response=True):
    # ROLL/PITCH/THROTTLE/YAW/AUX1/AUX2/AUX3/AUX4
    data = [roll, 
            pitch, 
            throttle, 
            yaw, 0, 
            servo_aux, 0, 0]
    with serial_lock:
        msp.send_msp_command(serial_port, msp.MSP_SET_RAW_RC, data)

        if not await_response:
            return True

        msp_command_id, payload = msp.read_msp_response(serial_port)
        if msp_command_id != msp.MSP_SET_RAW_RC:
            return False
        return True

def send_override_rc(roll, pitch, yaw, throttle, servo_aux=None, await_response=False):
    if servo_aux is None:
        servo_aux = int(vars.default_servo_aux2)

    return set_row_rc(
        int(roll),
        int(pitch),
        int(yaw),
        int(throttle),
        int(servo_aux),
        await_response=await_response,
    )

def send_attack_rc(roll, pitch, yaw, throttle, servo_aux=None):
    return send_override_rc(roll, pitch, yaw, throttle, servo_aux, await_response=False)


def send_ready_hold_rc(roll, pitch, yaw, throttle, servo_aux=None):
    return send_override_rc(roll, pitch, yaw, throttle, servo_aux, await_response=True)

def copter_init():
    # connect()
    
    return set_row_rc(
        vars.default_roll,
        vars.default_pitch, 
        vars.default_yaw, 
        vars.default_throttle, 
        vars.default_servo_aux2)

def telemetry(target):
    msp_target_command_id = get_target_id(target)
    with serial_lock:
        msp.send_msp_request(serial_port, msp_target_command_id)
        msp_command_id, payload = msp.read_msp_response_for(
            serial_port,
            msp_target_command_id,
            ignored_command_ids={msp.MSP_SET_RAW_RC},
        )
        if msp_command_id == msp_target_command_id:
            return payload

    return {}

def caught(): 
    wait_for_execution('caught')
    return True