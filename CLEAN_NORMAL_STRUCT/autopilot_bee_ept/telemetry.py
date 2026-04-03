import time
import autopilot
import messages
import router
import definitions as vars


def _queue_telemetry_command(command_name, priority, target, now, last_requested_at, interval):
    last_request_time = last_requested_at.get(target, 0.0)
    if now - last_request_time < interval:
        return

    router.put_command(router.Command(priority, command_name, {'target': target}))
    last_requested_at[target] = now


def _route_telemetry_by_mode(now, last_requested_at):
    bee_state = autopilot.state.get('bee_state', 'OFF')

    if bee_state == 'OFF':
        off_interval = vars.telemetry_off_interval
        _queue_telemetry_command('MONITOR', 2, 'MSP_ANALOG', now, last_requested_at, off_interval)
        _queue_telemetry_command('TELEMETRY', 2, 'MSP_ALTITUDE', now, last_requested_at, off_interval)
        _queue_telemetry_command('TELEMETRY', 1, 'MSP_RC', now, last_requested_at, off_interval)
        return bee_state

    if bee_state == 'ATACK':
        _queue_telemetry_command(
            'MONITOR',
            2,
            'MSP_ANALOG',
            now,
            last_requested_at,
            vars.telemetry_attack_status_interval,
        )
        _queue_telemetry_command(
            'TELEMETRY',
            2,
            'MSP_ALTITUDE',
            now,
            last_requested_at,
            vars.telemetry_attack_status_interval,
        )
        _queue_telemetry_command(
            'TELEMETRY',
            1,
            'MSP_RC',
            now,
            last_requested_at,
            vars.telemetry_attack_rc_interval,
        )
        return bee_state

    _queue_telemetry_command(
        'MONITOR',
        2,
        'MSP_ANALOG',
        now,
        last_requested_at,
        vars.telemetry_ready_status_interval,
    )
    _queue_telemetry_command(
        'TELEMETRY',
        2,
        'MSP_ALTITUDE',
        now,
        last_requested_at,
        vars.telemetry_ready_status_interval,
    )
    _queue_telemetry_command(
        'TELEMETRY',
        1,
        'MSP_RC',
        now,
        last_requested_at,
        vars.telemetry_ready_rc_interval,
    )
    return bee_state

def telemetry_requestor(stop_command):
    while autopilot.state['connection'] == False and not stop_command.is_set():
        try:
            time.sleep(5)
            messages.display(messages.telemetry_process_connecting, [vars.companion_computer])
        except Exception as e:
            messages.display(messages.fatal_error, [e])
            pass
    
    if not stop_command.is_set():
        messages.display(messages.telemetry_process_connected, [vars.companion_computer])

    last_requested_at = {}
    last_mode = autopilot.state.get('bee_state', 'OFF')

    while not stop_command.is_set():
        try:            
            now = time.monotonic()
            current_mode = _route_telemetry_by_mode(now, last_requested_at)
            if current_mode != last_mode:
                last_requested_at.clear()
                last_mode = current_mode

            time.sleep(vars.telemetry_scheduler_interval)
        except:
            pass

    stopped_time = time.strftime("%H:%M:%S, %Y, %d %B", time.localtime())
    messages.display(messages.telemetry_requestor_done, [stopped_time])