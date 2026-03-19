"""
DDS-integrated handler for the Inspire RH56F1 dexterous hand.

Wraps the RS485 or EtherCAT SDK with unitree_sdk2py DDS publishing
and subscribing, matching the DFTP SDK pattern:
  - Publishes hand state on   rt/inspire_hand_f1/state/{l|r}
  - Publishes touch data on   rt/inspire_hand_f1/touch/{l|r}
  - Subscribes to ctrl on     rt/inspire_hand_f1/ctrl/{l|r}

Usage (single hand):
    handler = F1HandDDSHandler(
        LR='r', device_id=1, use_serial=True,
        serial_port='/dev/ttyUSB0',
    )
    while True:
        data = handler.read()

Usage (double hand):
    handler = F1HandDDSHandlerDouble(
        device_id=[2, 1], use_serial=True,
        serial_port='/dev/ttyUSB0',
    )
    while True:
        data = handler.read()
"""

import struct
import threading
import time
from typing import Optional

from .inspire_dds import inspire_hand_f1_ctrl, inspire_hand_f1_state, inspire_hand_f1_touch
from .registers import (
    REG_ANGLE_SET, REG_FORCE_SET, REG_SPEED_SET, REG_POS_SET,
    REG_POS_ACT, REG_ANGLE_ACT, REG_FORCE_ACT, REG_CURRENT_ACT,
    REG_ERROR, REG_STATUS, REG_TEMP, REG_TOUCH_BASE,
    REG_CLEAR_ERROR, REG_DEFAULT_SPEED, REG_DEFAULT_FORCE, NUM_DOFS,
)

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber

from pymodbus.client import ModbusSerialClient

_modbus_lock = threading.Lock()

# F1 register-based states structure (all INT16, 6 registers each)
_DEFAULT_STATES_STRUCTURE = [
    ('pos_act',     REG_POS_ACT,     6, 'short'),
    ('angle_act',   REG_ANGLE_ACT,   6, 'short'),
    ('force_act',   REG_FORCE_ACT,   6, 'short'),
    ('current',     REG_CURRENT_ACT, 6, 'short'),
    ('err',         REG_ERROR,       6, 'short'),
    ('status',      REG_STATUS,      6, 'short'),
    ('temperature', REG_TEMP,        6, 'short'),
]


def _new_state_msg():
    return inspire_hand_f1_state(
        pos_act=[0] * 6,
        angle_act=[0] * 6,
        force_act=[0] * 6,
        current=[0] * 6,
        err=[0] * 6,
        status=[0] * 6,
        temperature=[0] * 6,
    )


def _new_ctrl_msg():
    return inspire_hand_f1_ctrl(
        pos_set=[0] * 6,
        angle_set=[0] * 6,
        force_set=[0] * 6,
        speed_set=[0] * 6,
        mode=0b0000,
    )


def _new_touch_msg():
    return inspire_hand_f1_touch(
        little_normal_force=0, little_tangential_force=0, little_direction=0,
        little_proximity_lo=0, little_proximity_hi=0,
        ring_normal_force=0, ring_tangential_force=0, ring_direction=0,
        ring_proximity_lo=0, ring_proximity_hi=0,
        middle_normal_force=0, middle_tangential_force=0, middle_direction=0,
        middle_proximity_lo=0, middle_proximity_hi=0,
        index_normal_force=0, index_tangential_force=0, index_direction=0,
        index_proximity_lo=0, index_proximity_hi=0,
        thumb_normal_force=0, thumb_tangential_force=0, thumb_direction=0,
        thumb_proximity_lo=0, thumb_proximity_hi=0,
        palm_left_normal_force=0, palm_left_tangential_force=0,
        palm_left_direction=0,
        palm_center_normal_force=0, palm_center_tangential_force=0,
        palm_center_direction=0,
        palm_right_normal_force=0, palm_right_tangential_force=0,
        palm_right_direction=0,
    )


_TOUCH_FIELD_ORDER = [
    'little_normal_force', 'little_tangential_force', 'little_direction',
    'little_proximity_lo', 'little_proximity_hi',
    'ring_normal_force', 'ring_tangential_force', 'ring_direction',
    'ring_proximity_lo', 'ring_proximity_hi',
    'middle_normal_force', 'middle_tangential_force', 'middle_direction',
    'middle_proximity_lo', 'middle_proximity_hi',
    'index_normal_force', 'index_tangential_force', 'index_direction',
    'index_proximity_lo', 'index_proximity_hi',
    'thumb_normal_force', 'thumb_tangential_force', 'thumb_direction',
    'thumb_proximity_lo', 'thumb_proximity_hi',
    'palm_left_normal_force', 'palm_left_tangential_force',
    'palm_left_direction',
    'palm_center_normal_force', 'palm_center_tangential_force',
    'palm_center_direction',
    'palm_right_normal_force', 'palm_right_tangential_force',
    'palm_right_direction',
]


def _read_modbus_registers(client, start_address, num_registers, device_id, lock=None):
    """Read and parse Modbus holding registers (signed INT16)."""
    _lock = lock or _modbus_lock
    with _lock:
        response = client.read_holding_registers(start_address, count=num_registers, device_id=device_id)
    if response.isError():
        return None
    packed = struct.pack('>' + 'H' * num_registers, *response.registers)
    return list(struct.unpack('>' + 'h' * num_registers, packed))


def _merge_contiguous_reads(states_structure):
    """Merge contiguous register groups into bulk reads.

    Returns a list of (start_address, total_count, [(attr, offset, count)]).
    """
    if not states_structure:
        return []

    sorted_s = sorted(states_structure, key=lambda x: x[1])
    groups = []
    cur_start = sorted_s[0][1]
    cur_end = cur_start + sorted_s[0][2]
    cur_fields = [(sorted_s[0][0], 0, sorted_s[0][2])]

    for attr, addr, count, dtype in sorted_s[1:]:
        if addr <= cur_end + 20:
            cur_fields.append((attr, addr - cur_start, count))
            cur_end = max(cur_end, addr + count)
        else:
            groups.append((cur_start, cur_end - cur_start, cur_fields))
            cur_start = addr
            cur_end = addr + count
            cur_fields = [(attr, 0, count)]
    groups.append((cur_start, cur_end - cur_start, cur_fields))
    return groups


def _read_modbus_registers_unsigned(client, start_address, num_registers, device_id, lock=None):
    """Read Modbus holding registers as unsigned INT16."""
    _lock = lock or _modbus_lock
    with _lock:
        response = client.read_holding_registers(start_address, count=num_registers, device_id=device_id)
    if response.isError():
        return None
    return list(response.registers)


# ═══════════════════════════════════════════════════════════════════════════════
#  Single Hand DDS Handler
# ═══════════════════════════════════════════════════════════════════════════════

class F1HandDDSHandler:
    """DDS-integrated handler for a single RH56F1 hand.

    Calling ``read()`` in a loop reads registers from the hand and publishes
    state/touch via DDS. Subscribing to the ctrl topic allows external
    processes to command the hand.

    Parameters
    ----------
    LR : str
        'l' or 'r' — appended to the DDS topic name.
    device_id : int
        Hand ID on the RS485 bus.
    use_serial : bool
        True for RS485 Modbus RTU serial.
    serial_port : str
        Serial port path.
    baudrate : int
        RS485 baud rate.
    network : str or None
        NIC name for DDS. None = default.
    states_structure : list or None
        Custom list of (attr, addr, count, dtype) tuples.
        Use a smaller subset to increase poll frequency.
    read_touch : bool
        Whether to read and publish touch data each cycle.
    init_dds : bool
        Call ChannelFactoryInitialize (only needed once per process).
    max_retries : int
        Connection retry count.
    retry_delay : float
        Seconds between retries.
    """

    def __init__(
        self,
        LR: str = 'r',
        device_id: int = 1,
        use_serial: bool = True,
        serial_port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        network: Optional[str] = None,
        states_structure: Optional[list] = None,
        read_touch: bool = False,
        init_dds: bool = True,
        max_retries: int = 5,
        retry_delay: float = 2.0,
    ):
        self.device_id = device_id
        self.read_touch = read_touch
        self.states_structure = states_structure or _DEFAULT_STATES_STRUCTURE
        self._bulk_groups = _merge_contiguous_reads(self.states_structure)
        self._lock = threading.Lock()

        self.client = ModbusSerialClient(
            port=serial_port, baudrate=baudrate, bytesize=8,
            stopbits=1, parity='N', timeout=1,
        )
        self._connect(max_retries, retry_delay)

        if init_dds:
            try:
                if network is not None:
                    ChannelFactoryInitialize(0, network)
                else:
                    ChannelFactoryInitialize(0)
            except Exception as e:
                print(f"DDS init error: {e}")
                return

        with self._lock:
            self.client.write_register(REG_CLEAR_ERROR, 1, device_id=self.device_id)
            self.client.write_registers(REG_SPEED_SET, [2000] * NUM_DOFS, device_id=self.device_id)
            self.client.write_registers(REG_FORCE_SET, [6000] * NUM_DOFS, device_id=self.device_id)

        self.state_pub = ChannelPublisher(
            "rt/inspire_hand_f1/state/" + LR, inspire_hand_f1_state)
        self.state_pub.Init()

        if self.read_touch:
            self.touch_pub = ChannelPublisher(
                "rt/inspire_hand_f1/touch/" + LR, inspire_hand_f1_touch)
            self.touch_pub.Init()

        self.ctrl_sub = ChannelSubscriber(
            "rt/inspire_hand_f1/ctrl/" + LR, inspire_hand_f1_ctrl)
        self.ctrl_sub.Init(self._ctrl_callback, 10)

    def _connect(self, max_retries, retry_delay):
        for attempt in range(1, max_retries + 1):
            if self.client.connect():
                print(f"Modbus connected on attempt {attempt}")
                return
            print(f"Connection attempt {attempt} failed, retrying in {retry_delay}s...")
            time.sleep(retry_delay)
        raise ConnectionError("Failed to connect after max retries")

    def _ctrl_callback(self, msg: inspire_hand_f1_ctrl):
        with self._lock:
            if msg.mode & 0b0001:
                vals = [v & 0xFFFF for v in msg.angle_set]
                self.client.write_registers(REG_ANGLE_SET, vals, device_id=self.device_id)
            if msg.mode & 0b0010:
                vals = [v & 0xFFFF for v in msg.pos_set]
                self.client.write_registers(REG_POS_SET, vals, device_id=self.device_id)
            if msg.mode & 0b0100:
                vals = [v & 0xFFFF for v in msg.force_set]
                self.client.write_registers(REG_FORCE_SET, vals, device_id=self.device_id)
            if msg.mode & 0b1000:
                vals = [v & 0xFFFF for v in msg.speed_set]
                self.client.write_registers(REG_SPEED_SET, vals, device_id=self.device_id)

    def read(self) -> dict:
        """Read hand state (and optionally touch), publish via DDS, return data."""
        states_msg = _new_state_msg()
        for start_addr, total_count, fields in self._bulk_groups:
            bulk = _read_modbus_registers(self.client, start_addr, total_count, self.device_id, lock=self._lock)
            if bulk is not None:
                for attr_name, offset, count in fields:
                    setattr(states_msg, attr_name, bulk[offset:offset + count])
        self.state_pub.Write(states_msg)

        touch_dict = {}
        if self.read_touch:
            touch_msg = _new_touch_msg()
            touch_vals = _read_modbus_registers_unsigned(
                self.client, REG_TOUCH_BASE, 34, self.device_id, lock=self._lock)
            if touch_vals is not None:
                for i, field in enumerate(_TOUCH_FIELD_ORDER):
                    if i < len(touch_vals):
                        setattr(touch_msg, field, touch_vals[i])
                self.touch_pub.Write(touch_msg)
                touch_dict = {field: getattr(touch_msg, field) for field in _TOUCH_FIELD_ORDER}

        return {
            'states': {
                'POS_ACT':   list(states_msg.pos_act),
                'ANGLE_ACT': list(states_msg.angle_act),
                'FORCE_ACT': list(states_msg.force_act),
                'CURRENT':   list(states_msg.current),
                'ERROR':     list(states_msg.err),
                'STATUS':    list(states_msg.status),
                'TEMP':      list(states_msg.temperature),
            },
            'touch': touch_dict,
        }

    def close(self):
        self.client.close()


# ═══════════════════════════════════════════════════════════════════════════════
#  Double Hand DDS Handler
# ═══════════════════════════════════════════════════════════════════════════════

class F1HandDDSHandlerDouble:
    """DDS-integrated handler for TWO RH56F1 hands on the same RS485 bus.

    Both hands share one serial port, distinguished by device IDs.
    Publishes on ``rt/inspire_hand_f1/state/l`` and ``rt/inspire_hand_f1/state/r``.
    Subscribes on ``rt/inspire_hand_f1/ctrl/l`` and ``rt/inspire_hand_f1/ctrl/r``.

    Parameters
    ----------
    device_id : list[int]
        [left_id, right_id], e.g. [2, 1].
    use_serial : bool
        True for RS485 Modbus RTU serial.
    serial_port : str
        Serial port path.
    baudrate : int
        RS485 baud rate.
    network : str or None
        NIC name for DDS.
    states_structure : list or None
        Custom state register list.
    read_touch : bool
        Whether to read touch data each cycle.
    init_dds : bool
        Call ChannelFactoryInitialize (only needed once per process).
    max_retries : int
        Connection retry count.
    retry_delay : float
        Seconds between retries.
    """

    def __init__(
        self,
        device_id: list = None,
        use_serial: bool = True,
        serial_port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        network: Optional[str] = None,
        states_structure: Optional[list] = None,
        read_touch: bool = False,
        init_dds: bool = True,
        max_retries: int = 5,
        retry_delay: float = 2.0,
    ):
        if device_id is None:
            device_id = [2, 1]
        self.device_id = device_id  # [left, right]
        self.read_touch = read_touch
        self.states_structure = states_structure or _DEFAULT_STATES_STRUCTURE
        self._bulk_groups = _merge_contiguous_reads(self.states_structure)

        self.client = ModbusSerialClient(
            port=serial_port, baudrate=baudrate, bytesize=8,
            stopbits=1, parity='N', timeout=1,
        )
        self._connect(max_retries, retry_delay)

        if init_dds:
            try:
                if network is not None:
                    ChannelFactoryInitialize(0, network)
                else:
                    ChannelFactoryInitialize(0)
            except Exception as e:
                print(f"DDS init error: {e}")
                return

        for did in self.device_id:
            with _modbus_lock:
                self.client.write_register(REG_CLEAR_ERROR, 1, device_id=did)
                self.client.write_registers(REG_SPEED_SET, [2000] * NUM_DOFS, device_id=did)
                self.client.write_registers(REG_FORCE_SET, [6000] * NUM_DOFS, device_id=did)

        # State publishers
        self.state_pub_l = ChannelPublisher(
            "rt/inspire_hand_f1/state/l", inspire_hand_f1_state)
        self.state_pub_l.Init()
        self.state_pub_r = ChannelPublisher(
            "rt/inspire_hand_f1/state/r", inspire_hand_f1_state)
        self.state_pub_r.Init()

        # Touch publishers
        if self.read_touch:
            self.touch_pub_l = ChannelPublisher(
                "rt/inspire_hand_f1/touch/l", inspire_hand_f1_touch)
            self.touch_pub_l.Init()
            self.touch_pub_r = ChannelPublisher(
                "rt/inspire_hand_f1/touch/r", inspire_hand_f1_touch)
            self.touch_pub_r.Init()

        # Control subscribers
        self.ctrl_sub_l = ChannelSubscriber(
            "rt/inspire_hand_f1/ctrl/l", inspire_hand_f1_ctrl)
        self.ctrl_sub_l.Init(self._ctrl_callback_l, 10)
        self.ctrl_sub_r = ChannelSubscriber(
            "rt/inspire_hand_f1/ctrl/r", inspire_hand_f1_ctrl)
        self.ctrl_sub_r.Init(self._ctrl_callback_r, 10)

    def _connect(self, max_retries, retry_delay):
        for attempt in range(1, max_retries + 1):
            if self.client.connect():
                print(f"Modbus connected on attempt {attempt}")
                return
            print(f"Connection attempt {attempt} failed, retrying in {retry_delay}s...")
            time.sleep(retry_delay)
        raise ConnectionError("Failed to connect after max retries")

    def _write_ctrl(self, msg: inspire_hand_f1_ctrl, device_id: int):
        with _modbus_lock:
            if msg.mode & 0b0001:
                vals = [v & 0xFFFF for v in msg.angle_set]
                self.client.write_registers(REG_ANGLE_SET, vals, device_id=device_id)
            if msg.mode & 0b0010:
                vals = [v & 0xFFFF for v in msg.pos_set]
                self.client.write_registers(REG_POS_SET, vals, device_id=device_id)
            if msg.mode & 0b0100:
                vals = [v & 0xFFFF for v in msg.force_set]
                self.client.write_registers(REG_FORCE_SET, vals, device_id=device_id)
            if msg.mode & 0b1000:
                vals = [v & 0xFFFF for v in msg.speed_set]
                self.client.write_registers(REG_SPEED_SET, vals, device_id=device_id)

    def _ctrl_callback_l(self, msg: inspire_hand_f1_ctrl):
        self._write_ctrl(msg, self.device_id[0])

    def _ctrl_callback_r(self, msg: inspire_hand_f1_ctrl):
        self._write_ctrl(msg, self.device_id[1])

    def _read_one(self, device_id):
        states_msg = _new_state_msg()
        for start_addr, total_count, fields in self._bulk_groups:
            bulk = _read_modbus_registers(self.client, start_addr, total_count, device_id)
            if bulk is not None:
                for attr_name, offset, count in fields:
                    setattr(states_msg, attr_name, bulk[offset:offset + count])

        touch_dict = {}
        touch_msg = None
        if self.read_touch:
            touch_msg = _new_touch_msg()
            touch_vals = _read_modbus_registers_unsigned(
                self.client, REG_TOUCH_BASE, 34, device_id)
            if touch_vals is not None:
                for i, field in enumerate(_TOUCH_FIELD_ORDER):
                    if i < len(touch_vals):
                        setattr(touch_msg, field, touch_vals[i])
                touch_dict = {field: getattr(touch_msg, field) for field in _TOUCH_FIELD_ORDER}

        return states_msg, touch_msg, touch_dict

    def read(self) -> list[dict]:
        """Read both hands, publish via DDS, return [left_data, right_data]."""
        states_l, touch_l, touch_dict_l = self._read_one(self.device_id[0])
        states_r, touch_r, touch_dict_r = self._read_one(self.device_id[1])

        self.state_pub_l.Write(states_l)
        self.state_pub_r.Write(states_r)

        if self.read_touch and touch_l is not None:
            self.touch_pub_l.Write(touch_l)
        if self.read_touch and touch_r is not None:
            self.touch_pub_r.Write(touch_r)

        def _to_dict(msg, td):
            return {
                'states': {
                    'POS_ACT':   list(msg.pos_act),
                    'ANGLE_ACT': list(msg.angle_act),
                    'FORCE_ACT': list(msg.force_act),
                    'CURRENT':   list(msg.current),
                    'ERROR':     list(msg.err),
                    'STATUS':    list(msg.status),
                    'TEMP':      list(msg.temperature),
                },
                'touch': td,
            }

        return [_to_dict(states_l, touch_dict_l), _to_dict(states_r, touch_dict_r)]

    def close(self):
        self.client.close()
