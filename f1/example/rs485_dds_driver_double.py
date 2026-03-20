#!/usr/bin/env python3
"""
RS485 + DDS driver for BOTH left and right Inspire RH56F1 hands
on separate serial ports, with concurrent reads.

Reads both hands via Modbus RTU and publishes on DDS topics:
  - rt/inspire_hand_f1/state/l  and  rt/inspire_hand_f1/state/r
  - rt/inspire_hand_f1/touch/l  and  rt/inspire_hand_f1/touch/r  (if --touch)
Subscribes to control commands on:
  - rt/inspire_hand_f1/ctrl/l  and  rt/inspire_hand_f1/ctrl/r

Usage:
    python rs485_dds_driver_double.py --port-left /dev/ttyUSB0 --port-right /dev/ttyUSB1
"""

import sys
import os
import time
import argparse
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk.dds_handler import F1HandDDSHandler


def main():
    parser = argparse.ArgumentParser(
        description='RH56F1 RS485+DDS Driver - LEFT + RIGHT (separate ports)')
    parser.add_argument('--port-left', type=str, default='/dev/ttyUSB0')
    parser.add_argument('--port-right', type=str, default='/dev/ttyUSB1')
    parser.add_argument('--id-left', type=int, default=1, help='Left hand ID (default: 1)')
    parser.add_argument('--id-right', type=int, default=1, help='Right hand ID (default: 1)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--touch', action='store_true', help='Also read/publish touch data')
    parser.add_argument('--touch-skip', type=int, default=4,
                        help='Read touch every N cycles (default: 4)')
    args = parser.parse_args()

    print(f"[DOUBLE] left: {args.port_left} (id={args.id_left}), "
          f"right: {args.port_right} (id={args.id_right})")

    states_structure = [
        ('angle_act', 1064, 6, 'short'),
        ('force_act', 1070, 6, 'short'),
        ('status',    1088, 6, 'short'),
    ]

    handler_l = F1HandDDSHandler(
        LR='l',
        device_id=args.id_left,
        serial_port=args.port_left,
        baudrate=args.baudrate,
        states_structure=states_structure,
        read_touch=args.touch,
        init_dds=True,
    )
    handler_r = F1HandDDSHandler(
        LR='r',
        device_id=args.id_right,
        serial_port=args.port_right,
        baudrate=args.baudrate,
        states_structure=states_structure,
        read_touch=args.touch,
        init_dds=False,
    )
    time.sleep(0.5)

    call_count = 0
    start_time = time.perf_counter()
    window_start = start_time

    try:
        while True:
            do_touch = args.touch and call_count % args.touch_skip == 0
            handler_l.read_touch = do_touch
            handler_r.read_touch = do_touch

            t = threading.Thread(target=handler_r.read)
            t.start()
            handler_l.read()
            t.join()

            call_count += 1

            if call_count % 10 == 0:
                now = time.perf_counter()
                inst_freq = 10 / (now - window_start)
                avg_freq = call_count / (now - start_time)
                print(f"  inst: {inst_freq:.1f} Hz, avg: {avg_freq:.1f} Hz, "
                      f"calls: {call_count}")
                window_start = now

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"Done. calls={call_count}, elapsed={elapsed:.6f}s, avg={freq:.2f} Hz")
    finally:
        handler_l.close()
        handler_r.close()


if __name__ == '__main__':
    main()
