#!/usr/bin/env python3
"""
EtherCAT driver for BOTH left and right Inspire RH56F1 dexterous hands
on the same EtherCAT bus.

Both hands are EtherCAT slaves on the same network interface, distinguished
by their slave index (position on the bus). The single pysoem master manages
both slaves with independent PDO exchange.

Usage:
    sudo python ethercat_driver_double.py
    sudo python ethercat_driver_double.py --ifname enp3s0
    sudo python ethercat_driver_double.py --slave-left 1 --slave-right 2
"""

import sys
import os
import time
import struct
import threading
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pysoem
from inspire_f1_sdk.registers import (
    NUM_DOFS, DOF_NAMES, ECAT_SDO,
    decode_error, decode_status,
)


_INPUT_PDO_SIZE = (6 + 6 + 6 + 6 + 6 + 6 + 6 + 34) * 2   # 152 bytes
_OUTPUT_PDO_SIZE = (1 + 6 + 6 + 6) * 2                      # 38 bytes


class DualHandEtherCAT:
    """Manages two RH56F1 hands on the same EtherCAT bus.

    Parameters
    ----------
    ifname : str
        Network interface name.
    slave_left : int
        Slave index (1-based) for the left hand.
    slave_right : int
        Slave index (1-based) for the right hand.
    cycle_time_ms : float
        PDO exchange cycle time in milliseconds.
    """

    def __init__(self, ifname: str, slave_left: int = 1, slave_right: int = 2,
                 cycle_time_ms: float = 2.0):
        self.ifname = ifname
        self.slave_left = slave_left
        self.slave_right = slave_right
        self.cycle_time_ms = cycle_time_ms

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
        self._running = True
        self._thread = threading.Thread(target=self._cyclic_task, daemon=True)
        self._thread.start()

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

    # ──────────────── Per-hand Control ───────────────────────────────────────

    def enable(self, side: str):
        with self._lock:
            self._setpoints[side]['enable'] = 1

    def disable(self, side: str):
        with self._lock:
            self._setpoints[side]['enable'] = 0

    def set_angles(self, side: str, angles: list[int]):
        assert len(angles) == NUM_DOFS
        with self._lock:
            self._setpoints[side]['angle'] = list(angles)
            self._setpoints[side]['enable'] = 1

    def set_forces(self, side: str, forces: list[int]):
        assert len(forces) == NUM_DOFS
        with self._lock:
            self._setpoints[side]['force'] = list(forces)

    def set_speeds(self, side: str, speeds: list[int]):
        assert len(speeds) == NUM_DOFS
        with self._lock:
            self._setpoints[side]['speed'] = list(speeds)

    def get_angles(self, side: str) -> list[int]:
        with self._lock:
            return list(struct.unpack_from('<6h', self._input[side], 6 * 2))

    def get_forces(self, side: str) -> list[int]:
        with self._lock:
            return list(struct.unpack_from('<6h', self._input[side], 12 * 2))

    def get_status(self, side: str) -> list[int]:
        with self._lock:
            return list(struct.unpack_from('<6h', self._input[side], 30 * 2))

    def get_all_states(self, side: str) -> dict:
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

    # ──────────────── SDO ────────────────────────────────────────────────────

    def clear_errors(self, side: str):
        idx, sub = ECAT_SDO['clear_error']
        self._slaves[side].sdo_write(idx, sub, struct.pack('<h', 1))

    def set_modes_sdo(self, side: str, modes: list[int]):
        idx, base_sub = ECAT_SDO['finger_mode']
        for i, mode in enumerate(modes):
            self._slaves[side].sdo_write(idx, base_sub + i, struct.pack('<h', mode))

    # ──────────────── Convenience ────────────────────────────────────────────

    def open_hand(self, side: str, speed: int = 2000, force: int = 6000):
        self.set_speeds(side, [speed] * NUM_DOFS)
        self.set_forces(side, [force] * NUM_DOFS)
        self.set_angles(side, [1740, 1740, 1740, 1740, 1350, 1800])

    def close_hand(self, side: str, speed: int = 2000, force: int = 6000):
        self.set_speeds(side, [speed] * NUM_DOFS)
        self.set_forces(side, [force] * NUM_DOFS)
        self.set_angles(side, [900, 900, 900, 900, 1100, 600])


def main():
    parser = argparse.ArgumentParser(
        description='Inspire RH56F1 EtherCAT Driver - LEFT + RIGHT hands')
    parser.add_argument('--ifname', type=str, default='eth0',
                        help='Network interface (default: eth0)')
    parser.add_argument('--slave-left', type=int, default=1,
                        help='Left hand slave index (default: 1)')
    parser.add_argument('--slave-right', type=int, default=2,
                        help='Right hand slave index (default: 2)')
    parser.add_argument('--cycle', type=float, default=2.0,
                        help='PDO cycle time in ms (default: 2.0)')
    args = parser.parse_args()

    print(f"[DOUBLE] EtherCAT on {args.ifname}, "
          f"left=slave {args.slave_left}, right=slave {args.slave_right}")

    hands = DualHandEtherCAT(
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

        # Open both hands
        print("Opening both hands...")
        hands.open_hand('l')
        hands.open_hand('r')
        time.sleep(1.5)

        # Close both hands
        print("Closing both hands...")
        hands.close_hand('l')
        hands.close_hand('r')
        time.sleep(1.5)

        # Open again
        hands.open_hand('l')
        hands.open_hand('r')
        time.sleep(1.5)

        # Continuous state reading
        print("\nReading states (Ctrl+C to exit)...")
        call_count = 0
        start_time = time.perf_counter()

        while True:
            states_l = hands.get_all_states('l')
            states_r = hands.get_all_states('r')
            call_count += 1
            time.sleep(0.002)

            if call_count % 100 == 0:
                elapsed = time.perf_counter() - start_time
                freq = call_count / elapsed
                print(f"  {freq:.2f} Hz | L={states_l['angle_act']} "
                      f"| R={states_r['angle_act']}")

    except KeyboardInterrupt:
        elapsed = time.perf_counter() - start_time
        freq = call_count / elapsed if elapsed > 0 else 0
        print(f"Stopped. calls={call_count}, elapsed={elapsed:.6f}s, freq={freq:.2f} Hz")
    finally:
        hands.open_hand('l', speed=1000)
        hands.open_hand('r', speed=1000)
        time.sleep(1.0)
        hands.stop()


if __name__ == '__main__':
    main()
