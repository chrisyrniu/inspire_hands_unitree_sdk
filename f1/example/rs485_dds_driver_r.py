#!/usr/bin/env python3
"""
RS485 + DDS driver for the RIGHT Inspire RH56F1 hand (single hand).

Reads hand state via Modbus RTU and publishes on DDS topics:
  - rt/inspire_hand_f1/state/r
  - rt/inspire_hand_f1/touch/r  (if --touch enabled)
Subscribes to control commands on:
  - rt/inspire_hand_f1/ctrl/r

Usage:
    python rs485_dds_driver_r.py
    python rs485_dds_driver_r.py --port /dev/ttyUSB1 --id 1
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk.dds_handler import F1HandDDSHandler


def main():
    parser = argparse.ArgumentParser(description='RH56F1 RS485+DDS Driver - RIGHT')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB1')
    parser.add_argument('--id', type=int, default=1, help='Right hand ID (default: 1)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--touch', action='store_true', help='Also read/publish touch data')
    parser.add_argument('--touch-skip', type=int, default=4,
                        help='Read touch every N cycles (default: 4)')
    args = parser.parse_args()

    print(f"[RIGHT] Starting RS485+DDS driver: port={args.port}, id={args.id}")

    states_structure = [
        ('angle_act', 1064, 6, 'short'),
        ('force_act', 1070, 6, 'short'),
        ('status',    1088, 6, 'short'),
    ]

    handler = F1HandDDSHandler(
        LR='r',
        device_id=args.id,
        serial_port=args.port,
        baudrate=args.baudrate,
        states_structure=states_structure,
        read_touch=args.touch,
    )
    time.sleep(0.5)

    call_count = 0
    start_time = time.perf_counter()

    try:
        while True:
            if args.touch and call_count % args.touch_skip == 0:
                handler.read_touch = True
            else:
                handler.read_touch = False

            data_dict = handler.read()
            call_count += 1

            if call_count % 20 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                print(f"[RIGHT] freq: {freq:.2f} Hz, calls: {call_count}, "
                      f"elapsed: {elapsed:.6f}s")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"[RIGHT] Done. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        handler.close()


if __name__ == '__main__':
    main()
