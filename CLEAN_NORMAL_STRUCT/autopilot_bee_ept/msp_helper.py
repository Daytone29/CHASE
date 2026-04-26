import struct
import time

# MSP command IDs
MSP_RAW_IMU = 102
MSP_ANALOG = 110
MSP_ALTITUDE = 109

MSP_RC = 105
MSP_SET_RAW_RC = 200

def get_checksum(msp_command_id, payload):
    checksum = 0
    length = len(payload)

    for byte in bytes([length, msp_command_id]) + payload:
        checksum ^= byte
    
    checksum &= 0xFF
    return checksum

def send_msp_command(serial_port, msp_command_id, data):
    payload = bytearray()
    for value in data:
        payload += struct.pack('<1H', value)

    header = b'$M<'
    length = len(payload)
    checksum = get_checksum(msp_command_id, payload)

    msp_package = header + bytes([length, msp_command_id]) + payload + bytes([checksum])
    serial_port.write(msp_package)

def send_msp_request(serial_port, msp_command_id):
    header = b'$M<'
    length = 0
    checksum = get_checksum(msp_command_id, bytes([]))

    msp_package = header + struct.pack('<BB', length, msp_command_id) + bytes([checksum])
    serial_port.write(msp_package)


def read_msp_response(serial_port):
    deadline = time.monotonic() + max(float(getattr(serial_port, 'timeout', 0.1) or 0.1), 0.01)
    response = bytearray()

    while time.monotonic() < deadline:
        chunk_size = getattr(serial_port, 'in_waiting', 0) or 1
        chunk = serial_port.read(chunk_size)
        if not chunk:
            continue

        response.extend(chunk)
        header_start = response.find(b'$M>')
        if header_start == -1:
            if len(response) > 3:
                del response[:-2]
            continue

        if header_start > 0:
            del response[:header_start]

        if len(response) < 6:
            continue

        payload_length = response[3]
        packet_length = 6 + payload_length
        if len(response) < packet_length:
            continue

        msp_command_id = response[4]
        payload = bytes(response[5:5 + payload_length])
        checksum = response[5 + payload_length]
        expected_checksum = get_checksum(msp_command_id, payload)
        if checksum != expected_checksum:
            raise ValueError("Invalid MSP checksum")

        return msp_command_id, payload

    raise TimeoutError("MSP response timeout")


def read_msp_response_for(serial_port, expected_command_id, ignored_command_ids=None, timeout=None):
    ignored_command_ids = set(ignored_command_ids or [])
    if timeout is None:
        timeout = max(float(getattr(serial_port, 'timeout', 0.1) or 0.1), 0.01)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msp_command_id, payload = read_msp_response(serial_port)
        if msp_command_id == expected_command_id:
            return msp_command_id, payload
        if msp_command_id in ignored_command_ids:
            continue

    raise TimeoutError(f"MSP response timeout for command {expected_command_id}")