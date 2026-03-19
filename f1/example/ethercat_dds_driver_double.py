#!/usr/bin/env python3
"""
EtherCAT + DDS driver for BOTH left and right Inspire RH56F1 hands
on the same EtherCAT bus.

Uses a single pysoem master managing both slaves. DDS topics are published
and subscribed for external integration.

Usage:
    sudo python ethercat_dds_driver_double.py
    sudo python ethercat_dds_driver_double.py --ifname enp3s0
"""

import sys
import os
import struct
import time
import threading
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pysoem
from inspire_f1_sdk.registers import NUM_DOFS, ECAT_SDO
from inspire_f1_sdk.inspire_dds import (
    inspire_hand_f1_state, inspire_hand_f1_touch, inspire_hand_f1_ctrl,
)
from inspire_f1_sdk.dds_handler import _new_state_msg, _new_touch_msg

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber


_INPUT_PDO_SIZE = (6 + 6 + 6 + 6 + 6 + 6 + 6 + 34) * 2   # 152 bytes
_OUTPUT_PDO_SIZE = (1 + 6 + 6 + 6) * 2                      # 38 bytes


class DualHandEtherCATDDS:
    """Two RH56F1 hands on one EtherCAT bus with DDS pub/sub."""

    def __init__(self, ifname: str, slave_left: int = 1, slave_right: int = 2,
                 cycle_time_ms: float = 2.0, network=None):
        self.ifname = ifname
        self.slave_left = slave_left
        self.slave_right = slave_right
        self.cycle_time_ms = cycle_time_ms
        self._network = network

        self._master = None
        self._slaves = {}
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        self._output = {
            'l': bytearray(_OUTPUT_PDO_SIZE),
            'r': bytearray(_OUTPUT_PDO_SIZE),
        }
        self._input = {
            'l': bytes(_INPUT_PDO_SIZE),
            'r': bytes(_INPUT_PDO_SIZE),
        }
        self._setpoints = {
            'l': {'enable': 0, 'angle': [0]*6, 'force': [0]*6, 'speed': [0]*6},
            'r': {'enable': 0, 'angle': [0]*6, 'force': [0]*6, 'speed': [0]*6},
        }

    def start(self):
        self._master = pysoem.Master()
        self._master.open(self.ifname)

        num_slaves = self._master.config_init()
        if num_slaves < 2:
            self._master.close()
            raise RuntimeError(
                f"Expected at least 2 EtherCAT slaves, found {num_slaves}")

        self._slaves['l'] = self._master.slaves[self.slave_left - 1]
        self._slaves['r'] = self._master.slaves[self.slave_right - 1]

        for side, slave in self._slaves.items():
            print(f"  [{side.upper()}] slave: {slave.name}, "
                  f"vendor=0x{slave.man:08X}, product=0x{slave.id:08X}")

        self._master.config_map()
        self._master.state_check(pysoem.SAFEOP_STATE, 500_000)

        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        for _ in range(40):
            self._master.state_check(pysoem.OP_STATE, 50_000)
            if self._master.state == pysoem.OP_STATE:
                break
        else:
            self._master.close()
            raise RuntimeError("Failed to reach OP state")

        print("Both slaves in OP state")

        if self._network is not None:
            ChannelFactoryInitialize(0, self._network)
        else:
            ChannelFactoryInitialize(0)

        self._state_pubs = {}
        self._touch_pubs = {}
        self._ctrl_subs = {}
        for lr in ('l', 'r'):
            pub = ChannelPublisher(f"rt/inspire_hand_f1/state/{lr}", inspire_hand_f1_state)
            pub.Init()
            self._state_pubs[lr] = pub

            tpub = ChannelPublisher(f"rt/inspire_hand_f1/touch/{lr}", inspire_hand_f1_touch)
            tpub.Init()
            self._touch_pubs[lr] = tpub

            sub = ChannelSubscriber(f"rt/inspire_hand_f1/ctrl/{lr}", inspire_hand_f1_ctrl)
            cb = self._make_ctrl_callback(lr)
            sub.Init(cb, 10)
            self._ctrl_subs[lr] = sub

        self._running = True
        self._thread = threading.Thread(target=self._cyclic_task, daemon=True)
        self._thread.start()

    def _make_ctrl_callback(self, side: str):
        def callback(msg: inspire_hand_f1_ctrl):
            with self._lock:
                if msg.mode & 0b0001:
                    self._setpoints[side]['angle'] = list(msg.angle_set)
                    self._setpoints[side]['enable'] = 1
                if msg.mode & 0b0100:
                    self._setpoints[side]['force'] = list(msg.force_set)
                if msg.mode & 0b1000:
                    self._setpoints[side]['speed'] = list(msg.speed_set)
        return callback

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._master:
            self._master.state = pysoem.INIT_STATE
            self._master.write_state()
            self._master.close()
            self._master = None
        print("EtherCAT stopped")

    def _cyclic_task(self):
        while self._running:
            with self._lock:
                for side in ('l', 'r'):
                    sp = self._setpoints[side]
                    vals = ([sp['enable']] + sp['angle'] + sp['force'] + sp['speed'])
                    struct.pack_into('<' + 'h' * len(vals),
                                    self._output[side], 0, *vals)
                    self._slaves[side].output = bytes(self._output[side])

            self._master.send_processdata()
            self._master.receive_processdata(timeout=2000)

            with self._lock:
                for side in ('l', 'r'):
                    raw = self._slaves[side].input
                    if raw and len(raw) >= _INPUT_PDO_SIZE:
                        self._input[side] = bytes(raw[:_INPUT_PDO_SIZE])

            time.sleep(self.cycle_time_ms / 1000.0)

    def enable(self, side: str):
        with self._lock:
            self._setpoints[side]['enable'] = 1

    def set_angles(self, side: str, angles: list[int]):
        with self._lock:
            self._setpoints[side]['angle'] = list(angles)
            self._setpoints[side]['enable'] = 1

    def set_forces(self, side: str, forces: list[int]):
        with self._lock:
            self._setpoints[side]['force'] = list(forces)

    def set_speeds(self, side: str, speeds: list[int]):
        with self._lock:
            self._setpoints[side]['speed'] = list(speeds)

    def clear_errors(self, side: str):
        idx, sub = ECAT_SDO['clear_error']
        self._slaves[side].sdo_write(idx, sub, struct.pack('<h', 1))

    def set_modes_sdo(self, side: str, modes: list[int]):
        idx, base_sub = ECAT_SDO['finger_mode']
        for i, mode in enumerate(modes):
            self._slaves[side].sdo_write(idx, base_sub + i, struct.pack('<h', mode))

    def _get_state_for_side(self, side: str) -> dict:
        with self._lock:
            data = self._input[side]
        return {
            'pos_act':   list(struct.unpack_from('<6h', data, 0)),
            'angle_act': list(struct.unpack_from('<6h', data, 12)),
            'force_act': list(struct.unpack_from('<6h', data, 24)),
            'current':   list(struct.unpack_from('<6h', data, 36)),
            'error':     list(struct.unpack_from('<6h', data, 48)),
            'status':    list(struct.unpack_from('<6h', data, 60)),
            'temp':      list(struct.unpack_from('<6h', data, 72)),
        }

    def _get_touch_for_side(self, side: str) -> list[int]:
        with self._lock:
            data = self._input[side]
        return list(struct.unpack_from('<34H', data, 84))

    def publish_states(self):
        """Read PDO data for both hands and publish via DDS. Returns state dicts."""
        results = {}
        for lr in ('l', 'r'):
            states = self._get_state_for_side(lr)
            msg = _new_state_msg()
            msg.pos_act = states['pos_act']
            msg.angle_act = states['angle_act']
            msg.force_act = states['force_act']
            msg.current = states['current']
            msg.err = states['error']
            msg.status = states['status']
            msg.temperature = states['temp']
            self._state_pubs[lr].Write(msg)

            touch_vals = self._get_touch_for_side(lr)
            touch_msg = _new_touch_msg()
            finger_names = ['little', 'ring', 'middle', 'index', 'thumb']
            for i, fn in enumerate(finger_names):
                base = i * 5
                setattr(touch_msg, f'{fn}_normal_force', touch_vals[base])
                setattr(touch_msg, f'{fn}_tangential_force', touch_vals[base + 1])
                setattr(touch_msg, f'{fn}_direction', touch_vals[base + 2])
                setattr(touch_msg, f'{fn}_proximity_lo', touch_vals[base + 3])
                setattr(touch_msg, f'{fn}_proximity_hi', touch_vals[base + 4])
            palm_base = 25
            for i, pn in enumerate(['palm_left', 'palm_center', 'palm_right']):
                base = palm_base + i * 3
                setattr(touch_msg, f'{pn}_normal_force', touch_vals[base])
                setattr(touch_msg, f'{pn}_tangential_force', touch_vals[base + 1])
                setattr(touch_msg, f'{pn}_direction', touch_vals[base + 2])
            self._touch_pubs[lr].Write(touch_msg)

            results[lr] = states
        return results


def main():
    parser = argparse.ArgumentParser(
        description='RH56F1 EtherCAT+DDS Driver - LEFT + RIGHT (single master)')
    parser.add_argument('--ifname', type=str, default='eth0')
    parser.add_argument('--slave-left', type=int, default=1)
    parser.add_argument('--slave-right', type=int, default=2)
    parser.add_argument('--cycle', type=float, default=2.0)
    args = parser.parse_args()

    print(f"[DOUBLE] EtherCAT+DDS on {args.ifname}, "
          f"left=slave {args.slave_left}, right=slave {args.slave_right}")

    hands = DualHandEtherCATDDS(
        ifname=args.ifname,
        slave_left=args.slave_left,
        slave_right=args.slave_right,
        cycle_time_ms=args.cycle,
    )

    try:
        hands.start()
        time.sleep(0.5)

        for side in ('l', 'r'):
            hands.clear_errors(side)
            hands.set_modes_sdo(side, [0] * 6)
            hands.set_speeds(side, [2000] * 6)
            hands.set_forces(side, [6000] * 6)
            hands.enable(side)

        call_count = 0
        start_time = time.perf_counter()

        while True:
            results = hands.publish_states()
            call_count += 1
            time.sleep(0.002)

            if call_count % 100 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                print(f"  {freq:.2f} Hz | L={results['l']['angle_act']} "
                      f"| R={results['r']['angle_act']}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"Stopped. calls={call_count}, freq={freq:.2f} Hz")
    finally:
        hands.stop()


if __name__ == '__main__':
    main()
