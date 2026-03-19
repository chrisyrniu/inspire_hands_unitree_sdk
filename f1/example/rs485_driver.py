#!/usr/bin/env python3
"""
RS485 driver example for the Inspire RH56F1 dexterous hand.

Demonstrates both Modbus RTU and raw serial protocol modes.
Reads hand state in a loop and performs basic motions.

Usage:
    python rs485_driver.py                        # default: Modbus RTU
    python rs485_driver.py --protocol serial       # raw serial protocol
    python rs485_driver.py --port /dev/ttyUSB1     # specify serial port
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_RS485


def main():
    parser = argparse.ArgumentParser(description='Inspire RH56F1 RS485 Driver')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--id', type=int, default=1,
                        help='Hand ID on the bus (default: 1)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--protocol', type=str, default='modbus_rtu',
                        choices=['modbus_rtu', 'serial'],
                        help='Protocol: modbus_rtu or serial (default: modbus_rtu)')
    args = parser.parse_args()

    print(f"Connecting to hand ID={args.id} on {args.port} "
          f"@ {args.baudrate} baud, protocol={args.protocol}")

    hand = InspireHandF1_RS485(
        port=args.port,
        hand_id=args.id,
        baudrate=args.baudrate,
        protocol=args.protocol,
    )

    try:
        hand.clear_errors()
        time.sleep(0.1)

        # Set operating modes: 0 = speed+force protection
        print("Setting modes to speed+force protection...")
        hand.set_modes([0, 0, 0, 0, 0, 0])
        time.sleep(0.1)

        # Set speeds
        print("Setting speeds to 2000...")
        hand.set_speeds([2000, 2000, 2000, 2000, 2000, 2000])
        time.sleep(0.1)

        # Set force thresholds
        print("Setting force thresholds to 6000...")
        hand.set_forces([6000, 6000, 6000, 6000, 6000, 6000])
        time.sleep(0.1)

        # Open hand
        print("Opening hand...")
        hand.set_angles([1740, 1740, 1740, 1740, 1350, 1800])
        time.sleep(1.5)
        hand.print_status()

        # Close hand
        print("Closing hand...")
        hand.set_angles([900, 900, 900, 900, 1100, 600])
        time.sleep(1.5)
        hand.print_status()

        # Open hand again
        print("Opening hand again...")
        hand.open_hand()
        time.sleep(1.5)

        # Run built-in action sequence #1
        print("Running action sequence #1...")
        hand.set_action_sequence(1)
        time.sleep(0.1)
        hand.run_action_sequence()
        time.sleep(2.0)

        # Continuous state reading loop
        print("\nEntering state reading loop (Ctrl+C to exit)...")
        call_count = 0
        start_time = time.perf_counter()

        while True:
            states = hand.get_all_states()
            call_count += 1
            time.sleep(0.005)

            if call_count % 50 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                angles = states['angle_act'] if states['angle_act'] else [0]*6
                forces = states['force_act'] if states['force_act'] else [0]*6
                print(f"  {freq:.1f} Hz | angles={angles} | forces={forces}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time if 'start_time' in dir() else 0
        print(f"\nStopped. Total calls: {call_count if 'call_count' in dir() else 0}")

    finally:
        hand.open_hand(speed=1000)
        time.sleep(1.0)
        hand.close()
        print("Connection closed.")


if __name__ == '__main__':
    main()
