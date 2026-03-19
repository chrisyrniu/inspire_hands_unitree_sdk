#!/usr/bin/env python3
"""
EtherCAT driver for the RIGHT Inspire RH56F1 dexterous hand (single hand).

Reads hand state in a loop via cyclic PDO exchange.
Requires root/sudo for raw socket access.

Usage:
    sudo python ethercat_driver_r.py
    sudo python ethercat_driver_r.py --ifname enp3s0 --slave 2
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_EtherCAT


def main():
    parser = argparse.ArgumentParser(
        description='Inspire RH56F1 EtherCAT Driver - RIGHT hand')
    parser.add_argument('--ifname', type=str, default='eth0',
                        help='Network interface (default: eth0)')
    parser.add_argument('--slave', type=int, default=2,
                        help='EtherCAT slave index for right hand (default: 2)')
    parser.add_argument('--cycle', type=float, default=2.0,
                        help='PDO cycle time in ms (default: 2.0)')
    args = parser.parse_args()

    print(f"[RIGHT] EtherCAT on {args.ifname}, slave={args.slave}, cycle={args.cycle}ms")

    hand = InspireHandF1_EtherCAT(
        ifname=args.ifname,
        slave_index=args.slave,
        cycle_time_ms=args.cycle,
    )

    try:
        hand.start()
        time.sleep(0.5)

        hand.clear_errors()
        hand.set_modes_sdo([0] * 6)
        hand.set_speeds([2000] * 6)
        hand.set_forces([6000] * 6)
        hand.enable()

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
                print(f"[RIGHT] {freq:.2f} Hz | angles={angles}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"[RIGHT] Stopped. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        hand.stop()


if __name__ == '__main__':
    main()
