import time
import struct
import queue
import messages
import autopilot
import threading

import commands as mavs
import definitions as vars
import sys
sys.path.append('/home/obriy/CHASE/CV')
from scope_controller import RCTrackerController

command_queue = queue.PriorityQueue()
tracking_controller = None
tracking_thread = None
ready_hold_thread = None
ready_hold_stop_event = threading.Event()
attack_thread = None
attack_stop_event = threading.Event()


def _clear_command_queue():
    with command_queue.mutex:
        command_queue.queue.clear()


def _reset_tracking_state(clear_caught=False):
    if clear_caught:
        autopilot.state['caught'] = False

    autopilot.state['target_locked'] = False
    autopilot.state['tracking_bbox'] = None
    autopilot.state['attack_error_x'] = 0.0
    autopilot.state['attack_error_y'] = 0.0
    autopilot.state['target_size'] = 0.0
    autopilot.state['prev_attack_error_x'] = 0.0
    autopilot.state['prev_attack_time'] = None
    autopilot.state['prev_target_size'] = 0.0


def _ensure_capture_tracking():
    tracker_controller = ensure_tracking_worker()
    tracker_controller.start_tracking()
    autopilot.state['caught'] = True
    return tracker_controller


def _apply_mode_state(new_mode, previous_mode):
    if new_mode == 'OFF':
        stop_ready_hold_worker()
        stop_attack_worker()
        autopilot.state['ready_hold_pending_capture'] = False
        _reset_tracking_state(clear_caught=True)

        if tracking_controller is not None:
            tracking_controller.stop_tracking()

        _clear_command_queue()
        return

    if new_mode == 'READY':
        stop_attack_worker()
        stop_ready_hold_worker()
        autopilot.state['ready_hold_pending_capture'] = False
        _ensure_capture_tracking()
        _clear_command_queue()
        return

    if new_mode == 'ATACK':
        stop_ready_hold_worker()
        _ensure_capture_tracking()
        autopilot.state['attack_base_roll'] = _get_last_manual_rc('roll', vars.default_roll)
        autopilot.state['attack_base_pitch'] = _get_last_manual_rc('pitch', vars.default_pitch)
        autopilot.state['attack_base_yaw'] = _get_last_manual_rc('yaw', vars.default_yaw)
        autopilot.state['attack_base_throttle'] = _get_last_manual_rc('throttle', vars.default_throttle)
        autopilot.state['prev_attack_error_x'] = 0.0
        autopilot.state['prev_attack_time'] = None
        autopilot.state['prev_target_size'] = 0.0
        _clear_command_queue()
        put_command(Command(0, 'ATACK', {'previous_mode': previous_mode}))


def _handle_tracking_stopped():
    _reset_tracking_state(clear_caught=True)
    if autopilot.state.get('bee_state') == 'ATACK':
        stop_attack_worker()
        autopilot.state['bee_state'] = 'READY'
        messages.display(messages.bee_state_changed_to, ['READY'])
        _apply_mode_state('READY', 'ATACK')
    messages.display(messages.command_caught_mission_completed)


def _run_tracking_worker():
    global tracking_controller

    try:
        tracking_controller.run()
    except Exception as e:
        messages.display(messages.fatal_error, [f"Tracking error: {e}"])
        import traceback
        messages.display(messages.fatal_error, [traceback.format_exc()])
        autopilot.state['caught'] = False


def ensure_tracking_worker():
    global tracking_controller, tracking_thread

    if tracking_controller is None or tracking_thread is None or not tracking_thread.is_alive():
        tracking_controller = RCTrackerController(
            resolution=(640, 480),
            bbox_size=(100, 100),
            autopilot_state=autopilot.state,
            on_tracking_stopped=_handle_tracking_stopped,
        )
        tracking_thread = threading.Thread(target=_run_tracking_worker, daemon=False)
        tracking_thread.start()

    return tracking_controller


def shutdown_tracking_worker():
    global tracking_thread

    stop_ready_hold_worker()
    stop_attack_worker()
    _reset_tracking_state(clear_caught=True)

    if tracking_controller is not None:
        tracking_controller.shutdown()

    if tracking_thread is not None and tracking_thread.is_alive():
        tracking_thread.join(timeout=2)


def _clamp_rc(value):
    return max(1000, min(2000, int(value)))


def _slew_rc(current_value, target_value, max_step):
    current_value = _clamp_rc(current_value)
    target_value = _clamp_rc(target_value)

    if max_step <= 0:
        return target_value

    delta = target_value - current_value
    if abs(delta) <= max_step:
        return target_value

    if delta > 0:
        return current_value + max_step

    return current_value - max_step


def _get_last_manual_rc(channel_name, fallback):
    snapshot_key = f'last_manual_{channel_name}'
    if snapshot_key in autopilot.state:
        return autopilot.state.get(snapshot_key, fallback)

    return autopilot.state.get(channel_name, fallback)


def _capture_ready_hold_values():
    autopilot.state['ready_hold_roll'] = _get_last_manual_rc('roll', vars.default_roll)
    autopilot.state['ready_hold_pitch'] = _get_last_manual_rc('pitch', vars.default_pitch)
    autopilot.state['ready_hold_yaw'] = _get_last_manual_rc('yaw', vars.default_yaw)
    autopilot.state['ready_hold_throttle'] = _get_last_manual_rc('throttle', vars.default_throttle)
    autopilot.state['ready_hold_pending_capture'] = False


def _run_ready_hold_worker():
    log_counter = 0

    while not ready_hold_stop_event.is_set():
        if autopilot.state.get('bee_state') != 'READY':
            break

        roll = _clamp_rc(autopilot.state.get('ready_hold_roll', vars.default_roll))
        pitch = _clamp_rc(autopilot.state.get('ready_hold_pitch', vars.default_pitch))
        yaw = _clamp_rc(autopilot.state.get('ready_hold_yaw', vars.default_yaw))
        throttle = _clamp_rc(autopilot.state.get('ready_hold_throttle', vars.default_throttle))

        mavs.send_ready_hold_rc(roll, pitch, yaw, throttle)

        log_counter += 1
        if log_counter % 50 == 0:
            messages.display(messages.command_ready_hold_tracking, [roll, pitch, yaw, throttle])

        time.sleep(vars.ready_hold_loop_interval)

    autopilot.state['ready_hold_active'] = False
    messages.display(messages.command_ready_hold_stopped)


def start_ready_hold_worker():
    global ready_hold_thread

    if ready_hold_thread is not None and ready_hold_thread.is_alive():
        return

    ready_hold_stop_event.clear()
    autopilot.state['ready_hold_active'] = True
    ready_hold_thread = threading.Thread(target=_run_ready_hold_worker, daemon=True)
    ready_hold_thread.start()
    messages.display(messages.command_ready_hold_started)


def stop_ready_hold_worker():
    global ready_hold_thread

    ready_hold_stop_event.set()
    if ready_hold_thread is not None and ready_hold_thread.is_alive():
        ready_hold_thread.join(timeout=1)
    ready_hold_thread = None
    autopilot.state['ready_hold_active'] = False


def _run_attack_worker():
    log_counter = 0

    while not attack_stop_event.is_set():
        if autopilot.state.get('bee_state') != 'ATACK':
            break

        base_roll = autopilot.state.get('attack_base_roll', autopilot.state.get('roll', vars.default_roll))
        base_pitch = autopilot.state.get('attack_base_pitch', autopilot.state.get('pitch', vars.default_pitch))
        base_yaw = autopilot.state.get('attack_base_yaw', autopilot.state.get('yaw', vars.default_yaw))
        base_throttle = autopilot.state.get('attack_base_throttle', autopilot.state.get('throttle', vars.default_throttle))

        error_x = autopilot.state.get('attack_error_x', 0.0)
        error_y = autopilot.state.get('attack_error_y', 0.0)
        target_locked = autopilot.state.get('target_locked', False)
        target_size = float(autopilot.state.get('target_size', 0.0) or 0.0)

        now = time.monotonic()
        prev_time = autopilot.state.get('prev_attack_time')
        dt = vars.attack_loop_interval
        if prev_time is not None:
            dt = max(now - prev_time, 1e-6)

        if abs(error_x) < vars.attack_deadzone:
            error_x = 0.0
        if abs(error_y) < vars.attack_deadzone:
            error_y = 0.0

        if not target_locked:
            error_x = 0.0
            error_y = 0.0
            target_size = 0.0

        prev_error_x = autopilot.state.get('prev_attack_error_x', error_x)
        prev_target_size = float(autopilot.state.get('prev_target_size', target_size) or target_size)
        omega = (error_x - prev_error_x) / dt if target_locked else 0.0
        size_rate = (target_size - prev_target_size) / dt if target_locked and target_size > 0.0 else 0.0

        target_roll = _clamp_rc(base_roll + int(error_x * vars.attack_roll_gain))
        target_pitch = _clamp_rc(base_pitch - int(error_y * vars.attack_pitch_gain))
        yaw_pn = int(error_x * vars.attack_yaw_gain + vars.attack_yaw_N * omega * 0.0005 * 500)
        target_yaw = _clamp_rc(base_yaw + yaw_pn)
        throttle_correction = -int(error_y * vars.attack_throttle_gain) - int(size_rate * vars.attack_throttle_brake)
        target_throttle = _clamp_rc(base_throttle + throttle_correction)

        if target_size > vars.attack_terminal_size:
            target_pitch = _clamp_rc(target_pitch - vars.attack_terminal_pitch_boost)
            target_throttle = _clamp_rc(target_throttle - vars.attack_terminal_throttle_drop)
            target_yaw = _clamp_rc(base_yaw + int(yaw_pn * vars.attack_terminal_yaw_damping))

        current_roll = autopilot.state.get('attack_output_roll', autopilot.state.get('roll', base_roll))
        current_pitch = autopilot.state.get('attack_output_pitch', autopilot.state.get('pitch', base_pitch))
        current_yaw = autopilot.state.get('attack_output_yaw', autopilot.state.get('yaw', base_yaw))
        current_throttle = autopilot.state.get('attack_output_throttle', autopilot.state.get('throttle', base_throttle))

        roll = _slew_rc(current_roll, target_roll, vars.attack_roll_max_step)
        pitch = _slew_rc(current_pitch, target_pitch, vars.attack_pitch_max_step)
        yaw = _slew_rc(current_yaw, target_yaw, vars.attack_yaw_max_step)
        throttle = _slew_rc(current_throttle, target_throttle, vars.attack_throttle_max_step)

        autopilot.state['prev_attack_error_x'] = error_x
        autopilot.state['prev_attack_time'] = now
        autopilot.state['prev_target_size'] = target_size
        autopilot.state['attack_output_roll'] = roll
        autopilot.state['attack_output_pitch'] = pitch
        autopilot.state['attack_output_yaw'] = yaw
        autopilot.state['attack_output_throttle'] = throttle

        mavs.send_attack_rc(roll, pitch, yaw, throttle)

        log_counter += 1
        if log_counter % 50 == 0:
            messages.display(messages.command_attack_tracking, [roll, pitch, yaw, throttle, error_x, error_y])

        time.sleep(vars.attack_loop_interval)

    autopilot.state['attack_active'] = False
    messages.display(messages.command_attack_stopped)


def start_attack_worker():
    global attack_thread

    if attack_thread is not None and attack_thread.is_alive():
        return

    attack_stop_event.clear()
    autopilot.state['attack_active'] = True
    autopilot.state['attack_output_roll'] = _clamp_rc(autopilot.state.get('roll', autopilot.state.get('attack_base_roll', vars.default_roll)))
    autopilot.state['attack_output_pitch'] = _clamp_rc(autopilot.state.get('pitch', autopilot.state.get('attack_base_pitch', vars.default_pitch)))
    autopilot.state['attack_output_yaw'] = _clamp_rc(autopilot.state.get('yaw', autopilot.state.get('attack_base_yaw', vars.default_yaw)))
    autopilot.state['attack_output_throttle'] = _clamp_rc(autopilot.state.get('throttle', autopilot.state.get('attack_base_throttle', vars.default_throttle)))
    attack_thread = threading.Thread(target=_run_attack_worker, daemon=True)
    attack_thread.start()
    messages.display(messages.command_attack_started)


def stop_attack_worker():
    global attack_thread

    attack_stop_event.set()
    if attack_thread is not None and attack_thread.is_alive():
        attack_thread.join(timeout=1)
    attack_thread = None
    autopilot.state['attack_active'] = False


def _is_duplicate_command(queued_command, new_command):
    if queued_command.name != new_command.name:
        return False

    queued_body = queued_command.body if isinstance(queued_command.body, dict) else {}
    new_body = new_command.body if isinstance(new_command.body, dict) else {}
    return queued_body.get('target') == new_body.get('target')

class Command:
    def __init__(self, priority, name, body):
        self.priority = priority
        self.name = name
        self.body = body

    def __lt__(self, other):
        return self.priority < other.priority

def put_command(command):
    with command_queue.mutex:
        for queued_command in command_queue.queue:
            if _is_duplicate_command(queued_command, command):
                return
    command_queue.put(command)

def command_executor(stop_command):
    connection = False
    while not connection and not stop_command.is_set():
        try:
            time.sleep(2)
            mavs.connect()
            messages.display(messages.command_executor_connected, [vars.companion_computer])
            connection = True
            autopilot.state['connection'] = True
        except Exception as e:
            messages.display(messages.fatal_error, [e])
            pass

    while not stop_command.is_set():
        try:
            command = command_queue.get(timeout=1)
            execute_command(command)
            command_queue.task_done()
            time.sleep(0.05)
        except:
            pass

    stopped_time = time.strftime("%H:%M:%S, %Y, %d %B", time.localtime())  
    messages.display(messages.command_executor_done, [stopped_time])

def execute_command(command):
    messages.display(messages.command_executor_executing_command, 
                     [command.name, command.priority, command.body])

    if command.name in commands:
        commands[command.name](command.body)

def command_monitor(params):
    monitor = mavs.telemetry(params['target'])
    messages.display(messages.command_monitor_log, [monitor])

    rssi_bytes = monitor[3:5]
    rssi = struct.unpack('<H', rssi_bytes)[0]

    battery_voltage = float(monitor[0]) / 10
    autopilot.state['battery'] = battery_voltage
    autopilot.state['rssi'] = rssi

    if rssi > 100:
        autopilot.state['rssi_msg'] = 'Strong signal'
    else:
        autopilot.state['rssi_msg'] = 'No signal'
    
    messages.display(
            messages.command_monitor_current_rssi_and_battery, 
            [rssi, autopilot.state['rssi_msg'], battery_voltage])

def command_telemetry_viable_status(telemetry):
    altitude = struct.unpack('<i', telemetry[0:4])[0] / 100
    speed = struct.unpack('<H', telemetry[4:6])[0] / 100
    autopilot.state['speed'] = speed
    autopilot.state['altitude'] = altitude
    if speed > 1:
        messages.display(
            messages.command_telemetry_current_speed, 
            [speed])
    if altitude > 1:
        messages.display(
            messages.command_telemetry_current_altitude, 
            [altitude])

def command_telemetry_mode_change(telemetry):
    rc_chs = struct.unpack('<' + 'H' * (len(telemetry) // 2), telemetry)

    autopilot.state['roll'] = rc_chs[0]
    autopilot.state['pitch'] = rc_chs[1]
    autopilot.state['yaw'] = rc_chs[2]
    autopilot.state['throttle'] = rc_chs[3]
    autopilot.state['last_manual_roll'] = rc_chs[0]
    autopilot.state['last_manual_pitch'] = rc_chs[1]
    autopilot.state['last_manual_yaw'] = rc_chs[2]
    autopilot.state['last_manual_throttle'] = rc_chs[3]
    autopilot.state['aux3'] = rc_chs[6]
    autopilot.state['aux4'] = rc_chs[7]
    autopilot.state['aux5'] = rc_chs[13] 
    autopilot.state['aux6'] = rc_chs[15]
    autopilot.state['aux7'] = rc_chs[10] 
    
    mode_aux_raw = int(autopilot.state['aux4'])
    autopilot_mode = autopilot.state['bee_state']
    if 950 <= mode_aux_raw <= 1050:
        autopilot_mode = 'OFF'
    elif 1450 <= mode_aux_raw <= 1550:
        autopilot_mode = 'READY'
    elif 1950 <= mode_aux_raw <= 2050:
        autopilot_mode = 'ATACK'
    
    if autopilot_mode != autopilot.state['bee_state']:
        previous_mode = autopilot.state['bee_state']
        autopilot.state['bee_state'] = autopilot_mode
        messages.display(
                    messages.bee_state_changed_to, [autopilot_mode])

        _apply_mode_state(autopilot_mode, previous_mode)

    elif autopilot_mode == 'READY':
        if not autopilot.state.get('caught'):
            _ensure_capture_tracking()

    elif autopilot_mode == 'ATACK':
        if not autopilot.state.get('caught'):
            _ensure_capture_tracking()
        if not autopilot.state.get('attack_active'):
            put_command(Command(0, 'ATACK', {'previous_mode': 'ATACK'}))

    elif autopilot_mode == 'OFF':
        if autopilot.state.get('caught'):
            _reset_tracking_state(clear_caught=True)

def command_telemetry(params):
    try:
        telemetry = mavs.telemetry(params['target'])
        messages.display(messages.command_telemetry_log, [telemetry])
 
        if telemetry != {}:
            if params['target'] == 'MSP_ALTITUDE':
                command_telemetry_viable_status(telemetry)
            if params['target'] == 'MSP_RC':
                command_telemetry_mode_change(telemetry)
                        
        messages.display(
            messages.command_telemetry_autopilot_state, 
            [autopilot.state])
    except Exception as ex:
        messages.display(
            messages.telemetry_reconnection, [ex])
        # In case of any error we will reboot connection
        mavs.reboot()


def command_init(params):
    messages.display(messages.initializing_autopilot)
    mavs.copter_init()

def command_caught(params):
    if autopilot.state.get('caught'):
        return

    try:
        _ensure_capture_tracking()
        messages.display(messages.command_caught_we_are_going_forward, [])
    except Exception as e:
        messages.display(messages.fatal_error, [f"Failed to init tracker: {e}"])
        import traceback
        messages.display(messages.fatal_error, [traceback.format_exc()])


def command_atack(params):
    if not autopilot.state.get('caught') or tracking_controller is None:
        messages.display(messages.command_attack_no_target)
        return

    start_attack_worker()

commands = {
    'INIT': command_init,
    'MONITOR': command_monitor,
    'TELEMETRY': command_telemetry,
    'CAUGHT': command_caught,
    'ATACK': command_atack,
}