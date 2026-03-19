#!/usr/bin/env python3
"""
EtherCAT driver example for the Inspire RH56F1 dexterous hand.

Demonstrates cyclic PDO control and SDO configuration.
Requires root/sudo for raw socket access.

Usage:
    sudo python ethercat_driver.py
    sudo python ethercat_driver.py --ifname enp3s0
    sudo python ethercat_driver.py --ifname eth0 --cycle 1.0

Prerequisites:
    pip install pysoem
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_EtherCAT


def main():
    parser = argparse.ArgumentParser(description='Inspire RH56F1 EtherCAT Driver')
    parser.add_argument('--ifname', type=str, default='eth0',
                        help='Network interface for EtherCAT (default: eth0)')
    parser.add_argument('--slave', type=int, default=1,
                        help='EtherCAT slave index, 1-based (default: 1)')
    parser.add_argument('--cycle', type=float, default=2.0,
                        help='PDO cycle time in ms (default: 2.0)')
    args = parser.parse_args()

    print(f"Starting EtherCAT on {args.ifname}, slave={args.slave}, "
          f"cycle={args.cycle}ms")

    hand = InspireHandF1_EtherCAT(
        ifname=args.ifname,
        slave_index=args.slave,
        cycle_time_ms=args.cycle,
    )

    try:
        hand.start()
        time.sleep(0.5)

        # Configure via SDO
        print("Clearing errors via SDO...")
        hand.clear_errors()

        # Set modes via SDO (0 = speed+force protection for all fingers)
        print("Setting finger modes via SDO...")
        hand.set_modes_sdo([0, 0, 0, 0, 0, 0])

        # Set speeds and forces via PDO
        print("Setting speeds and forces via PDO...")
        hand.set_speeds([2000, 2000, 2000, 2000, 2000, 2000])
        hand.set_forces([6000, 6000, 6000, 6000, 6000, 6000])
        hand.enable()
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

        # Partial motion: only index finger
        print("Moving index finger only (others hold via -1)...")
        hand.set_angles([-1, -1, -1, 1200, -1, -1])
        time.sleep(1.0)

        # Open hand
        print("Opening hand...")
        hand.open_hand()
        time.sleep(1.5)

        # Read touch data
        print("Reading touch data...")
        touch = hand.get_touch_data()
        for name, data in touch.items():
            print(f"  {name}: {data}")

        # Continuous state reading loop
        print("\nEntering state reading loop (Ctrl+C to exit)...")
        call_count = 0
        start_time = time.perf_counter()

        while True:
            states = hand.get_all_states()
            call_count += 1
            time.sleep(0.002)

            if call_count % 100 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                angles = states['angle_act']
                forces = states['force_act']
                print(f"  {freq:.1f} Hz | angles={angles} | forces={forces}")

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        hand.open_hand(speed=1000)
        time.sleep(1.0)
        hand.stop()
        print("Done.")


if __name__ == '__main__':
    main()
