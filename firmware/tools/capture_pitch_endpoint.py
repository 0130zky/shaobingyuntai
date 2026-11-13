"""Capture a disabled Pitch DM4310 endpoint through the existing RAM SWD mailbox.
Only mailbox command 4 (disable ID2) is sent. No position, zero or parameter write.
"""
import argparse
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
import socket
import struct
import time

repo = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('endpoint', choices=['lower', 'upper'])
parser.add_argument('--samples', type=int, default=24)
parser.add_argument('--output', type=Path, default=repo / 'outputs/calibration/endpoints.json')
args = parser.parse_args()
if not 8 <= args.samples <= 64:
    parser.error('--samples must be between 8 and 64')
symbols = json.loads((repo / 'outputs/can_discovery/pitch_id_editor_symbols.json').read_text())
address = symbols['id_editor']
connection = socket.create_connection(('127.0.0.1', 6666), timeout=3)
connection.settimeout(4)

def rpc(command):
    connection.sendall(command.encode('ascii') + b'\x1a')
    data = b''
    while b'\x1a' not in data:
        chunk = connection.recv(16384)
        if not chunk:
            raise RuntimeError('Debugger disconnected')
        data += chunk
    result = data.rstrip(b'\x1a').decode('ascii')
    if 'failed' in result.lower() or 'error' in result.lower():
        raise RuntimeError(result)
    return result

def words(address, count):
    return [int(word, 0) for word in rpc(f'read_memory {address:#x} 32 {count}').split()]

samples = []
try:
    if words(0xe000ed08, 1) != [0x24000000]:
        raise RuntimeError('RAM diagnostic is not active; no command sent')
    if words(0x24000000, 2)[1] != symbols['Reset_Handler'] | 1:
        raise RuntimeError('Diagnostic image mismatch; no command sent')
    state = words(address, 15)
    if state[0] != 0x49444544 or state[1]:
        raise RuntimeError('Mailbox not ready; no command sent')
    for _ in range(args.samples):
        before = words(address, 15)
        rpc(f'mww {address + 4:#x} 4')
        time.sleep(.07)
        current = words(address, 15)
        if current[2] != before[2] + 1 or current[3] != 4 or current[4]:
            raise RuntimeError('Disable request not processed successfully')
        if current[5] != before[5] + 1:
            raise RuntimeError('Expected exactly one fresh motor reply; check isolation')
        frame = struct.pack('<II', current[13], current[14])
        if current[12] != 0 or frame[0] != 2:
            raise RuntimeError(f'Unexpected reply: CAN {current[12]:#x}, {frame.hex()}')
        velocity = (frame[3] << 4) | (frame[4] >> 4)
        if abs(velocity - 2047.5) > 4.5:
            raise RuntimeError('Motor is moving; no endpoint saved')
        samples.append({'raw_encoder': (frame[1] << 8) | frame[2],
                        'velocity_raw': velocity, 'feedback_hex': frame.hex(),
                        'mos_c': frame[6], 'rotor_c': frame[7],
                        'can_ecr': current[11]})
finally:
    connection.close()
raws = [sample['raw_encoder'] for sample in samples]
if max(raws) - min(raws) > 8:
    raise RuntimeError(f'Endpoint not stationary: range {min(raws)}..{max(raws)}; no endpoint saved')
raw = Counter(raws).most_common(1)[0][0]
record = {'captured_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
          'raw_encoder': raw, 'raw_min': min(raws), 'raw_max': max(raws),
          'sample_count': len(samples), 'samples': samples,
          'motor_disabled': True, 'position_range_verified': True,
          'mit_position_range_rad': [-12.5, 12.5],
          'motor_position_rad': raw * 25.0 / 65535.0 - 12.5,
          'controller_flash_written': False}
if args.output.exists():
    document = json.loads(args.output.read_text())
else:
    document = {'version': 1, 'status': 'draft_not_in_controller_flash',
                'fold': {'esc_id': 1, 'feedback_can_id': 17,
                         'endpoints': {'folded': None, 'unfolded': None}},
                'pitch': {'esc_id': 2, 'feedback_can_id': 0,
                          'endpoints': {'lower': None, 'upper': None}}}
if document['pitch']['esc_id'] != 2 or document['pitch']['feedback_can_id'] != 0:
    raise RuntimeError('Existing calibration belongs to another motor; no overwrite')
document['pitch']['endpoints'][args.endpoint] = record
document['status'] = 'pitch_endpoint_capture_in_progress'
args.output.parent.mkdir(parents=True, exist_ok=True)
temporary = args.output.with_suffix('.tmp')
temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2) + '\n')
temporary.replace(args.output)
print(json.dumps({'endpoint': args.endpoint, 'raw_encoder': raw,
                  'raw_min': min(raws), 'raw_max': max(raws),
                  'samples': len(samples), 'saved_to': str(args.output)}, ensure_ascii=False))
