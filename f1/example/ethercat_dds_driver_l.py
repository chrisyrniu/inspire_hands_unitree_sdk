#!/usr/bin/env python3
"""
EtherCAT + DDS driver for the LEFT Inspire RH56F1 hand.

Reads hand state via EtherCAT PDO and publishes on DDS topics:
  - rt/inspire_hand_f1/state/l
  - rt/inspire_hand_f1/touch/l
Subscribes to control commands on:
  - rt/inspire_hand_f1/ctrl/l

Requires root/sudo for raw socket access.

Usage:
    sudo python ethercat_dds_driver_l.py
    sudo python ethercat_dds_driver_l.py --ifname enp3s0
"""

import sys
import os
import time
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk import InspireHandF1_EtherCAT
from inspire_f1_sdk.inspire_dds import (
    inspire_hand_f1_state, inspire_hand_f1_touch, inspire_hand_f1_ctrl,
)
from inspire_f1_sdk.dds_handler import _new_state_msg, _new_touch_msg

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber


def main():
    parser = argparse.ArgumentParser(description='RH56F1 EtherCAT+DDS Driver - LEFT')
    parser.add_argument('--ifname', type=str, default='eth0')
    parser.add_argument('--slave', type=int, default=1)
    parser.add_argument('--cycle', type=float, default=2.0)
    args = parser.parse_args()

    print(f"[LEFT] EtherCAT+DDS on {args.ifname}, slave={args.slave}")

    ChannelFactoryInitialize(0)

    hand = InspireHandF1_EtherCAT(
        ifname=args.ifname, slave_index=args.slave, cycle_time_ms=args.cycle)
    hand.start()
    time.sleep(0.5)

    hand.clear_errors()
    hand.set_modes_sdo([0] * 6)
    hand.set_speeds([2000] * 6)
    hand.set_forces([6000] * 6)
    hand.enable()

    state_pub = ChannelPublisher("rt/inspire_hand_f1/state/l", inspire_hand_f1_state)
    state_pub.Init()
    touch_pub = ChannelPublisher("rt/inspire_hand_f1/touch/l", inspire_hand_f1_touch)
    touch_pub.Init()

    def ctrl_callback(msg: inspire_hand_f1_ctrl):
        if msg.mode & 0b0001:
            hand.set_angles(list(msg.angle_set))
        if msg.mode & 0b0100:
            hand.set_forces(list(msg.force_set))
        if msg.mode & 0b1000:
            hand.set_speeds(list(msg.speed_set))

    ctrl_sub = ChannelSubscriber("rt/inspire_hand_f1/ctrl/l", inspire_hand_f1_ctrl)
    ctrl_sub.Init(ctrl_callback, 10)

    call_count = 0
    start_time = time.perf_counter()

    try:
        while True:
            states = hand.get_all_states()
            msg = _new_state_msg()
            msg.pos_act = states['pos_act']
            msg.angle_act = states['angle_act']
            msg.force_act = states['force_act']
            msg.current = states['current']
            msg.err = states['error']
            msg.status = states['status']
            msg.temperature = states['temp']
            state_pub.Write(msg)

            touch_data = hand.get_touch_data()
            touch_msg = _new_touch_msg()
            finger_names = ['little', 'ring', 'middle', 'index', 'thumb']
            for fn in finger_names:
                if fn in touch_data:
                    setattr(touch_msg, f'{fn}_normal_force', touch_data[fn]['normal_force'])
                    setattr(touch_msg, f'{fn}_tangential_force', touch_data[fn]['tangential_force'])
                    setattr(touch_msg, f'{fn}_direction', touch_data[fn]['direction'])
                    prox = touch_data[fn]['proximity']
                    setattr(touch_msg, f'{fn}_proximity_lo', prox & 0xFFFF)
                    setattr(touch_msg, f'{fn}_proximity_hi', (prox >> 16) & 0xFFFF)
            for pn in ['palm_left', 'palm_center', 'palm_right']:
                if pn in touch_data:
                    setattr(touch_msg, f'{pn}_normal_force', touch_data[pn]['normal_force'])
                    setattr(touch_msg, f'{pn}_tangential_force', touch_data[pn]['tangential_force'])
                    setattr(touch_msg, f'{pn}_direction', touch_data[pn]['direction'])
            touch_pub.Write(touch_msg)

            call_count += 1
            time.sleep(0.002)

            if call_count % 100 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                print(f"[LEFT]  {freq:.2f} Hz | angles={states['angle_act']}")

    except KeyboardInterrupt:
        print("[LEFT] Stopping...")
    finally:
        hand.stop()


if __name__ == '__main__':
    main()
