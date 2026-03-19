"""
RS485 communication SDK for the Inspire RH56F1 dexterous hand.

Supports two protocol modes on the RS485 physical layer:
  - 'serial':     Raw custom protocol (0xEB 0x90 frames) via pyserial
  - 'modbus_rtu': Standard Modbus RTU via pymodbus

Usage:
    hand = InspireHandF1_RS485('/dev/ttyUSB0', hand_id=1, protocol='modbus_rtu')
    hand.set_speeds([2000]*6)
    hand.set_forces([6000]*6)
    hand.set_angles([1000, 1000, 1000, 1000, 1200, 1000])
    print(hand.get_angles())
    hand.close()
"""

import struct
import time
import threading
from typing import Optional

from .registers import (
    REG_ID, REG_BAUDRATE, REG_CLEAR_ERROR, REG_SAVE, REG_RESET_FACTORY,
    REG_FORCE_CALIBRATE, REG_CURRENT_LIMIT, REG_DEFAULT_SPEED, REG_DEFAULT_FORCE,
    REG_POS_SET, REG_ANGLE_SET, REG_FORCE_SET, REG_SPEED_SET,
    REG_POS_ACT, REG_ANGLE_ACT, REG_FORCE_ACT, REG_CURRENT_ACT,
    REG_ERROR, REG_STATUS, REG_TEMP, REG_MODE,
    REG_PAUSE, REG_ESTOP, REG_ACTION_SEQ, REG_ACTION_RUN,
    REG_TOUCH_BASE, TOUCH_REGISTERS_PER_FINGER, TOUCH_FINGER_OFFSETS,
    TOUCH_PALM_OFFSETS, NUM_DOFS,
    decode_error, decode_status,
)


class InspireHandF1_RS485:
    """Driver for the Inspire RH56F1 hand over RS485.

    Parameters
    ----------
    port : str
        Serial port path, e.g. '/dev/ttyUSB0'.
    hand_id : int
        Hand ID on the bus (1-254).
    baudrate : int
        Serial baudrate. Default 115200.
    protocol : str
        'serial' for raw 0xEB/0x90 frame protocol,
        'modbus_rtu' for standard Modbus RTU.
    """

    def __init__(
        self,
        port: str,
        hand_id: int = 1,
        baudrate: int = 115200,
        protocol: str = 'modbus_rtu',
        _shared_client=None,
        _shared_lock=None,
    ):
        self.port = port
        self.hand_id = hand_id
        self.baudrate = baudrate
        self.protocol = protocol
        self._owns_connection = _shared_client is None
        self._lock = _shared_lock if _shared_lock is not None else threading.Lock()

        if _shared_client is not None:
            if protocol == 'modbus_rtu':
                self._client = _shared_client
            else:
                self._ser = _shared_client
        else:
            if protocol == 'modbus_rtu':
                self._init_modbus(port, baudrate)
            elif protocol == 'serial':
                self._init_serial(port, baudrate)
            else:
                raise ValueError(f"Unknown protocol '{protocol}'. Use 'serial' or 'modbus_rtu'.")

    # ──────────────────── Initialization ─────────────────────────────────────

    def _init_modbus(self, port: str, baudrate: int):
        from pymodbus.client import ModbusSerialClient
        self._client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            stopbits=1,
            parity='N',
            timeout=1,
        )
        if not self._client.connect():
            raise ConnectionError(f"Failed to connect Modbus RTU on {port}")

    def _init_serial(self, port: str, baudrate: int):
        import serial
        self._ser = serial.Serial(port, baudrate, timeout=0.1)

    def create_shared(self, hand_id: int) -> 'InspireHandF1_RS485':
        """Create a second instance sharing the same serial/Modbus connection.

        Use this when two hands are on the same RS485 bus. The returned
        instance uses the same underlying connection and lock but a different
        hand ID. Only close the *original* instance (not the shared one).
        """
        client = self._client if self.protocol == 'modbus_rtu' else self._ser
        return InspireHandF1_RS485(
            self.port, hand_id, self.baudrate, self.protocol,
            _shared_client=client, _shared_lock=self._lock,
        )

    def close(self):
        """Close the underlying connection (only if this instance owns it)."""
        if not self._owns_connection:
            return
        if self.protocol == 'modbus_rtu':
            self._client.close()
        else:
            self._ser.close()

    # ──────────────── Low-level: Modbus RTU ──────────────────────────────────

    def _modbus_read_registers(self, address: int, count: int) -> Optional[list[int]]:
        with self._lock:
            resp = self._client.read_holding_registers(address, count, device_id=self.hand_id)
        if resp.isError():
            return None
        packed = struct.pack('>' + 'H' * count, *resp.registers)
        return list(struct.unpack('>' + 'h' * count, packed))

    def _modbus_read_registers_unsigned(self, address: int, count: int) -> Optional[list[int]]:
        with self._lock:
            resp = self._client.read_holding_registers(address, count, device_id=self.hand_id)
        if resp.isError():
            return None
        return list(resp.registers)

    def _modbus_write_registers(self, address: int, values: list[int]):
        unsigned = [v & 0xFFFF for v in values]
        with self._lock:
            self._client.write_registers(address, unsigned, device_id=self.hand_id)

    def _modbus_write_register(self, address: int, value: int):
        with self._lock:
            self._client.write_register(address, value & 0xFFFF, device_id=self.hand_id)

    # ──────────────── Low-level: Raw Serial Protocol ─────────────────────────

    def _serial_write_register(self, address: int, num_bytes: int, data: list[int]):
        """Send a write-register frame using the custom 0xEB 0x90 protocol."""
        frame = [0xEB, 0x90, self.hand_id, num_bytes + 3, 0x12,
                 address & 0xFF, (address >> 8) & 0xFF]
        frame.extend(data)
        checksum = sum(frame[2:]) & 0xFF
        frame.append(checksum)
        with self._lock:
            self._ser.write(bytes(frame))
            time.sleep(0.01)
            self._ser.read_all()

    def _serial_read_register(self, address: int, num_bytes: int) -> Optional[list[int]]:
        """Send a read-register frame and parse the response."""
        frame = [0xEB, 0x90, self.hand_id, 0x04, 0x11,
                 address & 0xFF, (address >> 8) & 0xFF, num_bytes]
        checksum = sum(frame[2:]) & 0xFF
        frame.append(checksum)
        with self._lock:
            self._ser.write(bytes(frame))
            time.sleep(0.01)
            recv = self._ser.read_all()
        if len(recv) < 8:
            return None
        data_len = (recv[3] & 0xFF) - 3
        if len(recv) < 7 + data_len:
            return None
        return list(recv[7:7 + data_len])

    # ──────────────── Protocol-agnostic Read/Write ───────────────────────────

    def _write_6dof(self, address: int, values: list[int]):
        """Write 6 INT16 values to consecutive registers."""
        assert len(values) == NUM_DOFS
        if self.protocol == 'modbus_rtu':
            self._modbus_write_registers(address, values)
        else:
            data = []
            for v in values:
                data.append(v & 0xFF)
                data.append((v >> 8) & 0xFF)
            self._serial_write_register(address, 12, data)

    def _read_6dof(self, address: int) -> Optional[list[int]]:
        """Read 6 INT16 values from consecutive registers."""
        if self.protocol == 'modbus_rtu':
            return self._modbus_read_registers(address, NUM_DOFS)
        else:
            raw = self._serial_read_register(address, 12)
            if raw is None or len(raw) < 12:
                return None
            vals = []
            for i in range(NUM_DOFS):
                v = (raw[2 * i] & 0xFF) | (raw[2 * i + 1] << 8)
                if v > 32767:
                    v -= 65536
                vals.append(v)
            return vals

    def _write_single(self, address: int, value: int):
        """Write a single INT16 register."""
        if self.protocol == 'modbus_rtu':
            self._modbus_write_register(address, value)
        else:
            data = [value & 0xFF, (value >> 8) & 0xFF]
            self._serial_write_register(address, 2, data)

    def _read_single(self, address: int) -> Optional[int]:
        """Read a single INT16 register."""
        if self.protocol == 'modbus_rtu':
            vals = self._modbus_read_registers(address, 1)
            return vals[0] if vals else None
        else:
            raw = self._serial_read_register(address, 2)
            if raw is None or len(raw) < 2:
                return None
            v = (raw[0] & 0xFF) | (raw[1] << 8)
            if v > 32767:
                v -= 65536
            return v

    # ──────────────── High-level Control API ─────────────────────────────────

    def set_angles(self, angles: list[int]):
        """Set target angles for 6 DOFs.
        Four fingers: 900-1740 (unit 0.1°), thumb bend: 1100-1350,
        thumb rotate: 600-1800. Use -1 to skip a DOF.
        """
        self._write_6dof(REG_ANGLE_SET, angles)

    def set_forces(self, forces: list[int]):
        """Set force control thresholds for 6 DOFs, range 0-12000 (unit: g)."""
        self._write_6dof(REG_FORCE_SET, forces)

    def set_speeds(self, speeds: list[int]):
        """Set speeds for 6 DOFs, range 0-4000."""
        self._write_6dof(REG_SPEED_SET, speeds)

    def set_positions(self, positions: list[int]):
        """Set actuator positions for 6 DOFs, range 0-2000, -1 = no action."""
        self._write_6dof(REG_POS_SET, positions)

    def set_modes(self, modes: list[int]):
        """Set operating modes for 6 DOFs.
        0: speed+force protection, 1: force closed-loop, 2: impedance.
        """
        self._write_6dof(REG_MODE, modes)

    def set_default_speeds(self, speeds: list[int]):
        """Set power-on default speeds for 6 DOFs (saveable)."""
        self._write_6dof(REG_DEFAULT_SPEED, speeds)

    def set_default_forces(self, forces: list[int]):
        """Set power-on default force thresholds for 6 DOFs (saveable)."""
        self._write_6dof(REG_DEFAULT_FORCE, forces)

    def set_current_limits(self, limits: list[int]):
        """Set current protection values for 6 DOFs, range 0-1500 mA (saveable)."""
        self._write_6dof(REG_CURRENT_LIMIT, limits)

    # ──────────────── High-level Read API ────────────────────────────────────

    def get_angles(self) -> Optional[list[int]]:
        """Read actual angles for 6 DOFs (unit: 0.1°)."""
        return self._read_6dof(REG_ANGLE_ACT)

    def get_forces(self) -> Optional[list[int]]:
        """Read actual forces for 6 fingers (unit: g)."""
        return self._read_6dof(REG_FORCE_ACT)

    def get_positions(self) -> Optional[list[int]]:
        """Read actual actuator positions for 6 DOFs."""
        return self._read_6dof(REG_POS_ACT)

    def get_currents(self) -> Optional[list[int]]:
        """Read actual currents for 6 DOFs (unit: mA)."""
        return self._read_6dof(REG_CURRENT_ACT)

    def get_errors(self) -> Optional[list[int]]:
        """Read error codes for 6 DOFs."""
        return self._read_6dof(REG_ERROR)

    def get_status(self) -> Optional[list[int]]:
        """Read status codes for 6 DOFs."""
        return self._read_6dof(REG_STATUS)

    def get_temperatures(self) -> Optional[list[int]]:
        """Read temperatures for 6 DOFs (unit: °C)."""
        return self._read_6dof(REG_TEMP)

    def get_angle_setpoints(self) -> Optional[list[int]]:
        """Read the currently set angle targets."""
        return self._read_6dof(REG_ANGLE_SET)

    def get_force_setpoints(self) -> Optional[list[int]]:
        """Read the currently set force thresholds."""
        return self._read_6dof(REG_FORCE_SET)

    def get_speed_setpoints(self) -> Optional[list[int]]:
        """Read the currently set speed values."""
        return self._read_6dof(REG_SPEED_SET)

    def get_touch_data(self) -> Optional[dict]:
        """Read capacitive tactile sensor data for all fingers and palm.

        Returns a dict with keys for each finger ('little', 'ring', 'middle',
        'index', 'thumb') containing:
            normal_force, tangential_force, direction, proximity
        And palm regions ('palm_right', 'palm_center', 'palm_left') containing:
            normal_force, tangential_force, direction
        """
        total_regs = 34  # 5 fingers * 5 + 3 palms * 3
        if self.protocol == 'modbus_rtu':
            vals = self._modbus_read_registers_unsigned(REG_TOUCH_BASE, total_regs)
            if vals is None:
                return None
        else:
            raw = self._serial_read_register(REG_TOUCH_BASE, total_regs * 2)
            if raw is None or len(raw) < total_regs * 2:
                return None
            vals = []
            for i in range(total_regs):
                v = (raw[2 * i] & 0xFF) | (raw[2 * i + 1] << 8)
                vals.append(v)

        result = {}
        for finger, offset in TOUCH_FINGER_OFFSETS.items():
            normal = vals[offset]
            tangential = vals[offset + 1]
            direction = vals[offset + 2]
            proximity = vals[offset + 3] | (vals[offset + 4] << 16)
            result[finger] = {
                'normal_force': normal,
                'tangential_force': tangential,
                'direction': direction,
                'proximity': proximity,
            }

        for palm, offset in TOUCH_PALM_OFFSETS.items():
            result[palm] = {
                'normal_force': vals[offset],
                'tangential_force': vals[offset + 1],
                'direction': vals[offset + 2],
            }

        return result

    def get_all_states(self) -> Optional[dict]:
        """Read all state data in one call and return as a dict."""
        return {
            'pos_act':   self.get_positions(),
            'angle_act': self.get_angles(),
            'force_act': self.get_forces(),
            'current':   self.get_currents(),
            'error':     self.get_errors(),
            'status':    self.get_status(),
            'temp':      self.get_temperatures(),
        }

    # ──────────────── System Commands ────────────────────────────────────────

    def clear_errors(self):
        """Clear all clearable faults (locked-rotor, over-current, etc.)."""
        self._write_single(REG_CLEAR_ERROR, 1)

    def save_params(self):
        """Save current parameters to flash (persist across power cycles)."""
        self._write_single(REG_SAVE, 1)

    def reset_factory(self):
        """Restore all parameters to factory defaults."""
        self._write_single(REG_RESET_FACTORY, 1)

    def calibrate_force_sensors(self):
        """Start force sensor calibration. Hand must be unloaded (fingers open)."""
        self._write_single(REG_FORCE_CALIBRATE, 1)

    def set_id(self, new_id: int):
        """Change the hand ID (takes effect immediately, save to persist)."""
        self._write_single(REG_ID, new_id)
        self.hand_id = new_id

    def set_baudrate(self, baud_code: int):
        """Change RS485 baudrate. 0=115200, 1=57600, 2=19200. Requires save+reboot."""
        self._write_single(REG_BAUDRATE, baud_code)

    def set_action_sequence(self, seq_no: int):
        """Set the action sequence index (1-40). 1-13 are factory presets."""
        self._write_single(REG_ACTION_SEQ, seq_no)

    def run_action_sequence(self):
        """Execute the currently selected action sequence."""
        self._write_single(REG_ACTION_RUN, 1)

    def pause(self):
        """Pause all finger motion."""
        self._write_single(REG_PAUSE, 1)

    def emergency_stop(self):
        """Emergency stop all fingers."""
        self._write_single(REG_ESTOP, 1)

    def release_estop(self):
        """Release emergency stop."""
        self._write_single(REG_ESTOP, 0)

    # ──────────────── Convenience ────────────────────────────────────────────

    def open_hand(self, speed: int = 2000, force: int = 6000):
        """Fully open the hand (all fingers to max angle)."""
        self.set_speeds([speed] * NUM_DOFS)
        self.set_forces([force] * NUM_DOFS)
        self.set_angles([1740, 1740, 1740, 1740, 1350, 1800])

    def close_hand(self, speed: int = 2000, force: int = 6000):
        """Fully close the hand (all fingers to min angle)."""
        self.set_speeds([speed] * NUM_DOFS)
        self.set_forces([force] * NUM_DOFS)
        self.set_angles([900, 900, 900, 900, 1100, 600])

    def print_status(self):
        """Print a human-readable summary of the hand state."""
        states = self.get_all_states()
        if states is None:
            print("Failed to read hand state")
            return
        from .registers import DOF_NAMES
        print("─" * 60)
        for i, name in enumerate(DOF_NAMES):
            angle = states['angle_act'][i] if states['angle_act'] else '?'
            force = states['force_act'][i] if states['force_act'] else '?'
            status = decode_status(states['status'][i]) if states['status'] else '?'
            err = states['error'][i] if states['error'] else 0
            err_str = ', '.join(decode_error(err)) if err else 'none'
            temp = states['temp'][i] if states['temp'] else '?'
            print(f"  {name:14s}  angle={angle:>5}  force={force:>5}  "
                  f"status={status:>22s}  err={err_str:>20s}  temp={temp}°C")
        print("─" * 60)
