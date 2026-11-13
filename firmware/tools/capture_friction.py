#!/usr/bin/env python3
"""Finite capture of the current 25-channel stream, using the recorder's decoder."""
import argparse
import csv
import math
import time
from pathlib import Path
from telemetry_protocol import Decoder, FIELDS
from serial_transport import SerialPort


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--channels', type=int, choices=[25], default=25)
    parser.add_argument('--baudrate', type=int, choices=[115200,460800], default=460800)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    port = SerialPort(args.port,args.baudrate)
    frames = 0
    decoder = Decoder()
    start = time.monotonic()
    try:
        with args.output.open('w',newline='',encoding='utf-8') as stream:
            writer = csv.writer(stream);writer.writerow(['host_elapsed_s',*FIELDS])
            last_flush = start
            while time.monotonic()-start < args.seconds:
                chunk = port.read(timeout=min(.1,max(0,args.seconds-(time.monotonic()-start))))
                for values in decoder.feed(chunk):
                    writer.writerow([time.monotonic()-start,*values]);frames += 1
                if time.monotonic()-last_flush >= 1:
                    stream.flush();last_flush = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
    print('FRAMES',frames,'CSV',args.output)
    if not frames:raise SystemExit('No valid telemetry received')


if __name__ == '__main__':main()
