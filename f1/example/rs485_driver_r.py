#!/usr/bin/env python3
"""
RS485 driver for the RIGHT Inspire RH56F1 dexterous hand (single hand).

Reads hand state in a loop. Adjust port, ID, and protocol as needed.

Usage:
    python rs485_driver_r.py
    python rs485_driver_r.py --port /dev/ttyUSB1 --id 1 --protocol serial
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_RS485


def main():
    parser = argparse.ArgumentParser(description='Inspire RH56F1 RS485 Driver - RIGHT hand')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB1',
                        help='Serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--id', type=int, default=1,
                        help='Hand ID (default: 1 for right)')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--protocol', type=str, default='modbus_rtu',
                        choices=['modbus_rtu', 'serial'])
    args = parser.parse_args()

    print(f"[RIGHT] Connecting hand ID={args.id} on {args.port} @ {args.baudrate}, "
          f"protocol={args.protocol}")

    hand = InspireHandF1_RS485(
        port=args.port, hand_id=args.id,
        baudrate=args.baudrate, protocol=args.protocol,
    )

    try:
        hand.clear_errors()
        time.sleep(0.1)

        hand.set_modes([0] * 6)
        time.sleep(0.1)
        hand.set_speeds([2000] * 6)
        time.sleep(0.1)
        hand.set_forces([6000] * 6)
        time.sleep(0.1)

        call_count = 0
        start_time = time.perf_counter()

        while True:
            states = hand.get_all_states()
            call_count += 1
            time.sleep(0.001)

            if call_count % 20 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                angles = states['angle_act'] or [0]*6
                print(f"[RIGHT] {freq:.2f} Hz | angles={angles}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"[RIGHT] Stopped. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        hand.close()


if __name__ == '__main__':
    main()
