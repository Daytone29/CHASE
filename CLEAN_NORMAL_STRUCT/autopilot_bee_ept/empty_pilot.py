import time
import autopilot
import messages
import router
import definitions as vars

def empty_pilot_process(stop_command):
    while autopilot.state['connection'] == False and not stop_command.is_set():
        try:
            time.sleep(7)
            messages.display(messages.empty_pilot_process_connecting, [vars.companion_computer])
        except Exception as e:
            messages.display(messages.fatal_error, [e])
            pass
    
    if not stop_command.is_set():
        messages.display(messages.empty_pilot_process_connected, [vars.companion_computer])

    last_state = None
    last_caught = autopilot.state.get('caught', False)

    while not stop_command.is_set():
        try:
            state = autopilot.state.get('bee_state')
            caught = autopilot.state.get('caught', False)

            if state == 'READY':
                should_enqueue = not caught and (last_state != 'READY' or last_caught)
                if should_enqueue:
                    messages.display(messages.empty_pilot_state_ready)
                    # Use higher priority so CAUGHT runs before the telemetry backlog
                    router.put_command(router.Command(0, 'CAUGHT', {}))
                elif caught:
                    messages.display(messages.empty_pilot_already_caught)

            last_state = state
            last_caught = caught
            time.sleep(2)
        except:
            pass

    stopped_time = time.strftime("%H:%M:%S, %Y, %d %B", time.localtime())
    messages.display(messages.empty_pilot_process_done, [stopped_time])