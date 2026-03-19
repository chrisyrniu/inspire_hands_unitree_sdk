#!/usr/bin/env python3
"""
RS485 driver for BOTH left and right Inspire RH56F1 dexterous hands
on the same serial bus.

Two hands share the same serial port, distinguished by their IDs.
Default: left=ID 2, right=ID 1 (matching DFTP convention).

Usage:
    python rs485_driver_double.py
    python rs485_driver_double.py --port /dev/ttyUSB0 --id-left 2 --id-right 1
    python rs485_driver_double.py --protocol serial
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_RS485


def main():
    parser = argparse.ArgumentParser(
        description='Inspire RH56F1 RS485 Driver - LEFT + RIGHT hands')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--id-left', type=int, default=2,
                        help='Left hand ID (default: 2)')
    parser.add_argument('--id-right', type=int, default=1,
                        help='Right hand ID (default: 1)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--protocol', type=str, default='modbus_rtu',
                        choices=['modbus_rtu', 'serial'])
    args = parser.parse_args()

    print(f"[DOUBLE] port={args.port}, left_id={args.id_left}, right_id={args.id_right}, "
          f"protocol={args.protocol}")

    hand_l = InspireHandF1_RS485(
        port=args.port, hand_id=args.id_left,
        baudrate=args.baudrate, protocol=args.protocol,
    )
    hand_r = hand_l.create_shared(args.id_right)

    try:
        # Initialize both hands
        for label, hand in [('LEFT', hand_l), ('RIGHT', hand_r)]:
            hand.clear_errors()
            time.sleep(0.05)
            hand.set_modes([0] * 6)
            time.sleep(0.05)
            hand.set_speeds([2000] * 6)
            time.sleep(0.05)
            hand.set_forces([6000] * 6)
            time.sleep(0.05)
            print(f"  [{label}] initialized")

        time.sleep(0.5)

        call_count = 0
        start_time = time.perf_counter()

        while True:
            states_l = hand_l.get_all_states()
            states_r = hand_r.get_all_states()
            call_count += 1
            time.sleep(0.001)

            if call_count % 10 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                angles_l = states_l['angle_act'] or [0]*6
                angles_r = states_r['angle_act'] or [0]*6
                print(f"  {freq:.2f} Hz | L={angles_l} | R={angles_r}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"Stopped. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        hand_l.close()
        hand_r.close()


if __name__ == '__main__':
    main()
