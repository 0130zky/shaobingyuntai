"""Interactive ASCII console for Sentry USB CDC. Does not enable motors on connect."""
import argparse
import select
import sys
import time
import serial

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', required=True)
args = parser.parse_args()
commands = {'STATUS', 'CAL BEGIN', 'CAL FOLD A', 'CAL FOLD B', 'CAL PITCH A',
            'CAL PITCH B', 'CAL SAVE', 'CAL CANCEL', 'RUN'}
with serial.Serial(args.port, 115200, timeout=0) as port:
    print('Commands: ' + ', '.join(sorted(commands)), flush=True)
    pending = b''
    last_seq = None
    last_print = 0
    port.write(b'STATUS\n')
    try:
        while True:
            readable, _, _ = select.select([sys.stdin, port], [], [], 0.1)
            if port in readable:
                pending += port.read(4096)
                while b'\n' in pending:
                    row, pending = pending.split(b'\n', 1)
                    text = row.decode('ascii', errors='replace').strip()
                    seq = next((x for x in text.split() if x.startswith('seq=')), None)
                    if seq != last_seq or time.monotonic() - last_print >= 1:
                        print(text, flush=True)
                        last_seq, last_print = seq, time.monotonic()
                if len(pending) > 8192:
                    pending = b''
            if sys.stdin in readable:
                entry = sys.stdin.readline()
                if not entry:
                    break
                command = entry.strip()
                if command in commands:
                    port.write((command + '\n').encode('ascii'))
                else:
                    print('Unknown command; use one of: ' + ', '.join(sorted(commands)), flush=True)
    except KeyboardInterrupt:
        pass
