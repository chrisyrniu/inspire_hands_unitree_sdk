#!/usr/bin/env python3
"""
RS485 + DDS driver for the LEFT Inspire RH56F1 hand (single hand).

Reads hand state via Modbus RTU and publishes on DDS topics:
  - rt/inspire_hand_f1/state/l
  - rt/inspire_hand_f1/touch/l  (if --touch enabled)
Subscribes to control commands on:
  - rt/inspire_hand_f1/ctrl/l

Usage:
    python rs485_dds_driver_l.py
    python rs485_dds_driver_l.py --port /dev/ttyUSB0 --id 2
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk.dds_handler import F1HandDDSHandler


def main():
    parser = argparse.ArgumentParser(description='RH56F1 RS485+DDS Driver - LEFT')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0')
    parser.add_argument('--id', type=int, default=2, help='Left hand ID (default: 2)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--touch', action='store_true', help='Also read/publish touch data')
    args = parser.parse_args()

    print(f"[LEFT] Starting RS485+DDS driver: port={args.port}, id={args.id}")

    ## Publish all data
    # states_structure = [
    #     ('pos_act',     1058, 6, 'short'),
    #     ('angle_act',   1064, 6, 'short'),
    #     ('force_act',   1070, 6, 'short'),
    #     ('current',     1076, 6, 'short'),
    #     ('err',         1082, 6, 'short'),
    #     ('status',      1088, 6, 'short'),
    #     ('temperature', 1094, 6, 'short'),
    # ]

    ## Only publish this subset to increase frequency
    states_structure = [
        ('angle_act', 1064, 6, 'short'),
        ('force_act', 1070, 6, 'short'),
        ('status',    1088, 6, 'short'),
    ]

    handler = F1HandDDSHandler(
        LR='l',
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
            data_dict = handler.read()
            call_count += 1
            time.sleep(0.001)

            if call_count % 20 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                print(f"[LEFT]  freq: {freq:.2f} Hz, calls: {call_count}, "
                      f"elapsed: {elapsed:.6f}s")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"[LEFT] Done. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        handler.close()


if __name__ == '__main__':
    main()
