"""
Register map for the Inspire RH56F1 dexterous hand.
Addresses and data types derived from the official user manual V1.0.0.

All register addresses are 16-bit Modbus-style addresses.
Each address holds one 16-bit (INT16) register unless noted otherwise.
"""


# ──────────────────────── Configuration Registers (W/R) ────────────────────────

REG_ID              = 1000   # Hand ID, range 1-254, saveable
REG_BAUDRATE        = 1001   # 0=115200, 1=57600, 2=19200, saveable
REG_CLEAR_ERROR     = 1003   # Write 1 to clear clearable faults
REG_SAVE            = 1004   # Write 1 to save current params to flash
REG_RESET_FACTORY   = 1005   # Write 1 to restore factory defaults
REG_FORCE_CALIBRATE = 1007   # Write 1 to start force sensor calibration

# ──────────────────────── Current Limit Registers (W/R) ────────────────────────
# 6 x INT16, addresses 1016-1021, range 0-1500 mA, saveable
REG_CURRENT_LIMIT   = 1016

# ──────────────────── Default Power-on Speed Registers (W/R) ──────────────────
# 6 x INT16, addresses 1022-1027, range 0-4000, saveable
REG_DEFAULT_SPEED   = 1022

# ──────────────── Default Power-on Force Threshold Registers (W/R) ────────────
# 6 x INT16, addresses 1028-1033, range 0-12000 g, saveable
REG_DEFAULT_FORCE   = 1028

# ──────────────────── Actuator Position Set Registers (W/R) ───────────────────
# 6 x INT16, addresses 1034-1039, range 0-2000, -1 = no action
REG_POS_SET         = 1034

# ────────────────────────── Angle Set Registers (W/R) ─────────────────────────
# 6 x INT16, addresses 1040-1045
# Fingers 0-3: 900-1740 (unit: 0.1 deg), Thumb bend: 1100-1350, Thumb rotate: 600-1800
# -1 = no action for that DOF
REG_ANGLE_SET       = 1040

# ────────────────── Force Control Threshold Registers (W/R) ───────────────────
# 6 x INT16, addresses 1046-1051, range 0-12000 g
REG_FORCE_SET       = 1046

# ──────────────────────── Speed Set Registers (W/R) ───────────────────────────
# 6 x INT16, addresses 1052-1057, range 0-4000
REG_SPEED_SET       = 1052

# ────────────────── Actuator Position Actual Registers (R) ────────────────────
# 6 x INT16, addresses 1058-1063, range 0-2000
REG_POS_ACT         = 1058

# ────────────────────── Angle Actual Registers (R) ────────────────────────────
# 6 x INT16, addresses 1064-1069
REG_ANGLE_ACT       = 1064

# ──────────────────── Force Actual Registers (R) ──────────────────────────────
# 6 x INT16, addresses 1070-1075, range 0-12000 g
REG_FORCE_ACT       = 1070

# ────────────────────── Current Actual Registers (R) ──────────────────────────
# 6 x INT16, addresses 1076-1081, range 0-1500 mA
REG_CURRENT_ACT     = 1076

# ────────────────────── Error Code Registers (R) ──────────────────────────────
# 6 x INT16, addresses 1082-1087
REG_ERROR           = 1082

# ──────────────────── Status Info Registers (R) ───────────────────────────────
# 6 x INT16, addresses 1088-1093
REG_STATUS          = 1088

# ──────────────────── Temperature Registers (R) ───────────────────────────────
# 6 x INT16, addresses 1094-1099, range 0-100 °C
REG_TEMP            = 1094

# ──────────────────── Finger Mode Registers (W/R) ────────────────────────────
# 6 x INT16, addresses 1100-1105, range 0-2
# 0: speed+force protection, 1: force closed-loop, 2: impedance control
REG_MODE            = 1100

# ──────────────────── Pause / E-Stop Registers (W/R) ─────────────────────────
REG_PAUSE           = 1130   # Write 1 to pause
REG_ESTOP           = 1131   # Write 1 to emergency stop, write other to release

# ──────────────── Action Sequence Registers (W/R) ────────────────────────────
REG_ACTION_SEQ      = 2160   # Action sequence index, range 1-40
REG_ACTION_RUN      = 2162   # Write 1 to run the current action sequence

# ──────────────────── Touch Sensor Registers (R) ─────────────────────────────
# Capacitive tactile sensors: normal force, tangential force, direction, proximity
REG_TOUCH_BASE      = 3000
TOUCH_REGISTERS_PER_FINGER = 5   # normal(1) + tangential(1) + direction(1) + proximity(2)

TOUCH_FINGER_OFFSETS = {
    'little': 0,    # 3000-3004
    'ring':   5,    # 3005-3009
    'middle': 10,   # 3010-3014
    'index':  15,   # 3015-3019
    'thumb':  20,   # 3020-3024
}
TOUCH_PALM_OFFSETS = {
    'palm_left':   25,   # 3025-3027  (normal, tangential, direction)
    'palm_center': 28,   # 3028-3030
    'palm_right':  31,   # 3031-3033
}

NUM_DOFS = 6
DOF_NAMES = ['little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotate']

# ──────────────── EtherCAT PDO Object Dictionary ─────────────────────────────

ETHERCAT_INPUT_PDO_INDEX  = 0x6000
ETHERCAT_OUTPUT_PDO_INDEX = 0x7000
ETHERCAT_SDO_INDEX        = 0x2000

# Input PDO (TxPDO, slave -> master) subindex mapping
ECAT_IN = {
    'pos_act':   (0x6000, 0x01, 6),   # subindex 01-06
    'angle_act': (0x6000, 0x07, 6),   # subindex 07-0C
    'force_act': (0x6000, 0x0D, 6),   # subindex 0D-12
    'current':   (0x6000, 0x13, 6),   # subindex 13-18
    'error':     (0x6000, 0x19, 6),   # subindex 19-1E
    'status':    (0x6000, 0x1F, 6),   # subindex 1F-24
    'temp':      (0x6000, 0x25, 6),   # subindex 25-2A
    'touch':     (0x6000, 0x2B, 34),  # subindex 2B-4C (finger+palm touch)
}

# Output PDO (RxPDO, master -> slave) subindex mapping
ECAT_OUT = {
    'enable':    (0x7000, 0x01, 1),   # subindex 01
    'angle_set': (0x7000, 0x02, 6),   # subindex 02-07
    'force_set': (0x7000, 0x08, 6),   # subindex 08-0D
    'speed_set': (0x7000, 0x0E, 6),   # subindex 0E-13
}

# SDO configuration objects (0x2000)
ECAT_SDO = {
    'hand_id':           (0x2000, 0x01),
    'baudrate':          (0x2000, 0x02),
    'clear_error':       (0x2000, 0x03),
    'save':              (0x2000, 0x04),
    'reset_para':        (0x2000, 0x05),
    'force_calibrate':   (0x2000, 0x06),
    'current_limit':     (0x2000, 0x07),  # subindex 07-0C for 6 fingers
    'default_speed':     (0x2000, 0x0D),  # subindex 0D-12
    'default_force':     (0x2000, 0x13),  # subindex 13-18
    'action_seq_index':  (0x2000, 0x19),
    'action_seq_run':    (0x2000, 0x1A),
    'finger_mode':       (0x2000, 0x1B),  # subindex 1B-20
    'pause':             (0x2000, 0x21),
    'estop':             (0x2000, 0x22),
}

# ──────────────────── Status / Error Descriptions ────────────────────────────

STATUS_CODES = {
    0: 'releasing',
    1: 'gripping',
    2: 'position_reached',
    3: 'force_reached',
    5: 'current_protection_stop',
    6: 'actuator_locked_stop',
    7: 'actuator_fault_stop',
}

ERROR_BITS = {
    0: 'locked_rotor',
    1: 'over_temperature',
    2: 'over_current',
    3: 'motor_abnormal',
    4: 'communication_error',
}

BAUDRATE_MAP = {
    0: 115200,
    1: 57600,
    2: 19200,
}


def decode_error(error_code: int) -> list[str]:
    """Decode an error code bitmask into a list of error description strings."""
    return [desc for bit, desc in ERROR_BITS.items() if error_code & (1 << bit)]


def decode_status(status_code: int) -> str:
    """Decode a status code into a human-readable string."""
    return STATUS_CODES.get(status_code, f'unknown({status_code})')
