"""
EtherCAT communication SDK for the Inspire RH56F1 dexterous hand.

Uses pysoem (Python bindings for SOEM - Simple Open EtherCAT Master).
The F1 hand supports EtherCAT I/O mode with PDO and SDO objects.

Usage:
    hand = InspireHandF1_EtherCAT('eth0')
    hand.start()
    hand.set_angles([1000]*6)
    print(hand.get_angles())
    hand.stop()

Install:  pip install pysoem
"""

import struct
import time
import threading
from typing import Optional

import pysoem

from .registers import (
    NUM_DOFS, DOF_NAMES,
    ECAT_IN, ECAT_OUT, ECAT_SDO,
    decode_error, decode_status,
)

# PDO struct sizes (all INT16) per manual V1.0.0
# NOTE: The EEPROM on some units reports smaller sizes (SM2=18B, SM3=76B)
# instead of these values. If the slave fails to reach SAFEOP, the EEPROM
# ESI needs to be updated by Inspire (因时机器人) to match these sizes.
_INPUT_PDO_SIZE = (6 + 6 + 6 + 6 + 6 + 6 + 6 + 34) * 2   # 76 INT16 = 152 bytes
_OUTPUT_PDO_SIZE = (1 + 6 + 6 + 6) * 2                      # 19 INT16 = 38 bytes


class InspireHandF1_EtherCAT:
    """Driver for the Inspire RH56F1 hand over EtherCAT.

    Parameters
    ----------
    ifname : str
        Network interface name, e.g. 'eth0', 'enp3s0'.
    slave_index : int
        EtherCAT slave index (1-based). Default 1 for single-hand setups.
    cycle_time_ms : float
        Process data exchange cycle time in milliseconds.
    """

    def __init__(
        self,
        ifname: str,
        slave_index: int = 1,
        cycle_time_ms: float = 2.0,
    ):
        self.ifname = ifname
        self.slave_index = slave_index
        self.cycle_time_ms = cycle_time_ms

        self._master: Optional[pysoem.Master] = None
        self._slave = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._output_data = bytearray(_OUTPUT_PDO_SIZE)
        self._input_data = bytes(_INPUT_PDO_SIZE)

        self._enable = 0
        self._angle_set = [0] * NUM_DOFS
        self._force_set = [0] * NUM_DOFS
        self._speed_set = [0] * NUM_DOFS

    # ──────────────────── Lifecycle ──────────────────────────────────────────

    def start(self):
        """Initialize the EtherCAT master, configure the slave, and start cyclic PDO exchange."""
        self._master = pysoem.Master()
        self._master.open(self.ifname)

        num_slaves = self._master.config_init()
        if num_slaves == 0:
            self._master.close()
            raise RuntimeError("No EtherCAT slaves found on " + self.ifname)

        if self.slave_index > num_slaves:
            self._master.close()
            raise RuntimeError(
                f"Slave index {self.slave_index} out of range (found {num_slaves} slaves)")

        self._slave = self._master.slaves[self.slave_index - 1]
        print(f"Found slave: {self._slave.name}, vendor=0x{self._slave.man:08X}, "
              f"product=0x{self._slave.id:08X}")

        self._master.config_map()
        self._master.state_check(pysoem.SAFEOP_STATE, 500_000)

        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        for _ in range(200):
            self._master.send_processdata()
            self._master.receive_processdata(timeout=2000)
            self._master.state_check(pysoem.OP_STATE, 50_000)
            if self._master.state == pysoem.OP_STATE:
                break
        else:
            self._master.close()
            raise RuntimeError("Failed to reach OP state")

        print("EtherCAT slave in OP state")
        self._running = True
        self._thread = threading.Thread(target=self._cyclic_task, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop the cyclic task and close the EtherCAT master."""
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
        """Background thread performing cyclic PDO exchange."""
        while self._running:
            with self._lock:
                self._pack_output()
                self._slave.output = bytes(self._output_data)

            self._master.send_processdata()
            self._master.receive_processdata(timeout=2000)

            with self._lock:
                raw = self._slave.input
                if raw and len(raw) >= _INPUT_PDO_SIZE:
                    self._input_data = bytes(raw[:_INPUT_PDO_SIZE])

            time.sleep(self.cycle_time_ms / 1000.0)

    # ──────────────── PDO Pack/Unpack ────────────────────────────────────────

    def _pack_output(self):
        """Pack the output PDO buffer from current setpoints."""
        vals = [self._enable] + self._angle_set + self._force_set + self._speed_set
        struct.pack_into('<' + 'h' * len(vals), self._output_data, 0, *vals)

    def _unpack_input_block(self, start_index: int, count: int) -> list[int]:
        """Unpack a block of INT16 values from the input PDO buffer."""
        offset = start_index * 2
        return list(struct.unpack_from('<' + 'h' * count, self._input_data, offset))

    def _unpack_input_block_unsigned(self, start_index: int, count: int) -> list[int]:
        """Unpack a block of UINT16 values from the input PDO buffer."""
        offset = start_index * 2
        return list(struct.unpack_from('<' + 'H' * count, self._input_data, offset))

    # ──────────────── High-level Control API ─────────────────────────────────

    def enable(self):
        """Enable motion control via EtherCAT."""
        with self._lock:
            self._enable = 1

    def disable(self):
        """Disable motion control."""
        with self._lock:
            self._enable = 0

    def set_angles(self, angles: list[int]):
        """Set target angles for 6 DOFs.
        Four fingers: 900-1740 (unit 0.1°), thumb bend: 1100-1350,
        thumb rotate: 600-1800. Use -1 to skip.
        """
        assert len(angles) == NUM_DOFS
        with self._lock:
            self._angle_set = list(angles)
            self._enable = 1

    def set_forces(self, forces: list[int]):
        """Set force control thresholds for 6 DOFs, range 0-12000 (unit: g)."""
        assert len(forces) == NUM_DOFS
        with self._lock:
            self._force_set = list(forces)

    def set_speeds(self, speeds: list[int]):
        """Set speeds for 6 DOFs, range 0-4000."""
        assert len(speeds) == NUM_DOFS
        with self._lock:
            self._speed_set = list(speeds)

    # ──────────────── High-level Read API ────────────────────────────────────

    def get_positions(self) -> list[int]:
        """Read actual actuator positions for 6 DOFs."""
        with self._lock:
            return self._unpack_input_block(0, 6)

    def get_angles(self) -> list[int]:
        """Read actual angles for 6 DOFs (unit: 0.1°)."""
        with self._lock:
            return self._unpack_input_block(6, 6)

    def get_forces(self) -> list[int]:
        """Read actual forces for 6 fingers (unit: g)."""
        with self._lock:
            return self._unpack_input_block(12, 6)

    def get_currents(self) -> list[int]:
        """Read actual currents for 6 DOFs (unit: mA)."""
        with self._lock:
            return self._unpack_input_block(18, 6)

    def get_errors(self) -> list[int]:
        """Read error codes for 6 DOFs."""
        with self._lock:
            return self._unpack_input_block(24, 6)

    def get_status(self) -> list[int]:
        """Read status codes for 6 DOFs."""
        with self._lock:
            return self._unpack_input_block(30, 6)

    def get_temperatures(self) -> list[int]:
        """Read temperatures for 6 DOFs (unit: °C)."""
        with self._lock:
            return self._unpack_input_block(36, 6)

    def get_touch_data(self) -> dict:
        """Read capacitive tactile sensor data from EtherCAT PDO.

        Returns a dict with finger/palm touch data.
        """
        with self._lock:
            touch_vals = self._unpack_input_block_unsigned(42, 34)

        finger_names = ['little', 'ring', 'middle', 'index', 'thumb']
        result = {}
        for i, name in enumerate(finger_names):
            base = i * 5
            result[name] = {
                'normal_force': touch_vals[base],
                'tangential_force': touch_vals[base + 1],
                'direction': touch_vals[base + 2],
                'proximity': touch_vals[base + 3] | (touch_vals[base + 4] << 16),
            }

        palm_base = 25
        for i, palm_name in enumerate(['palm_left', 'palm_center', 'palm_right']):
            base = palm_base + i * 3
            result[palm_name] = {
                'normal_force': touch_vals[base],
                'tangential_force': touch_vals[base + 1],
                'direction': touch_vals[base + 2],
            }

        return result

    def get_all_states(self) -> dict:
        """Read all state data from the input PDO."""
        return {
            'pos_act':   self.get_positions(),
            'angle_act': self.get_angles(),
            'force_act': self.get_forces(),
            'current':   self.get_currents(),
            'error':     self.get_errors(),
            'status':    self.get_status(),
            'temp':      self.get_temperatures(),
        }

    # ──────────────── SDO Access (Configuration) ─────────────────────────────

    def sdo_read(self, index: int, subindex: int) -> int:
        """Read a 16-bit value from an SDO object."""
        data = self._slave.sdo_read(index, subindex, 2)
        return struct.unpack('<h', data)[0]

    def sdo_write(self, index: int, subindex: int, value: int):
        """Write a 16-bit value to an SDO object."""
        data = struct.pack('<h', value)
        self._slave.sdo_write(index, subindex, data)

    def clear_errors(self):
        """Clear all clearable faults via SDO."""
        idx, sub = ECAT_SDO['clear_error']
        self.sdo_write(idx, sub, 1)

    def save_params(self):
        """Save parameters to flash via SDO."""
        idx, sub = ECAT_SDO['save']
        self.sdo_write(idx, sub, 1)

    def set_modes_sdo(self, modes: list[int]):
        """Set finger operating modes via SDO.
        0: speed+force protection, 1: force closed-loop, 2: impedance.
        """
        assert len(modes) == NUM_DOFS
        idx, base_sub = ECAT_SDO['finger_mode']
        for i, mode in enumerate(modes):
            self.sdo_write(idx, base_sub + i, mode)

    def set_current_limits_sdo(self, limits: list[int]):
        """Set current protection values via SDO, range 0-1500 mA."""
        assert len(limits) == NUM_DOFS
        idx, base_sub = ECAT_SDO['current_limit']
        for i, lim in enumerate(limits):
            self.sdo_write(idx, base_sub + i, lim)

    def pause_sdo(self):
        """Send pause command via SDO."""
        idx, sub = ECAT_SDO['pause']
        self.sdo_write(idx, sub, 1)

    def emergency_stop_sdo(self):
        """Send emergency stop via SDO."""
        idx, sub = ECAT_SDO['estop']
        self.sdo_write(idx, sub, 1)

    def release_estop_sdo(self):
        """Release emergency stop via SDO."""
        idx, sub = ECAT_SDO['estop']
        self.sdo_write(idx, sub, 0)

    # ──────────────── Convenience ────────────────────────────────────────────

    def open_hand(self, speed: int = 2000, force: int = 6000):
        """Fully open the hand."""
        self.set_speeds([speed] * NUM_DOFS)
        self.set_forces([force] * NUM_DOFS)
        self.set_angles([1740, 1740, 1740, 1740, 1350, 1800])

    def close_hand(self, speed: int = 2000, force: int = 6000):
        """Fully close the hand."""
        self.set_speeds([speed] * NUM_DOFS)
        self.set_forces([force] * NUM_DOFS)
        self.set_angles([900, 900, 900, 900, 1100, 600])

    def print_status(self):
        """Print a human-readable summary of the hand state."""
        states = self.get_all_states()
        print("─" * 60)
        for i, name in enumerate(DOF_NAMES):
            angle = states['angle_act'][i]
            force = states['force_act'][i]
            status = decode_status(states['status'][i])
            err = states['error'][i]
            err_str = ', '.join(decode_error(err)) if err else 'none'
            temp = states['temp'][i]
            print(f"  {name:14s}  angle={angle:>5}  force={force:>5}  "
                  f"status={status:>22s}  err={err_str:>20s}  temp={temp}°C")
        print("─" * 60)
