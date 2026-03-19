"""
Inspire RH56F1 Dexterous Hand Python SDK

Supports RS485 (raw serial / Modbus RTU), EtherCAT communication,
and DDS broadcasting via unitree_sdk2py.
"""

from .rs485_sdk import InspireHandF1_RS485
from .ethercat_sdk import InspireHandF1_EtherCAT
from .registers import (
    NUM_DOFS, DOF_NAMES,
    decode_error, decode_status,
    STATUS_CODES, ERROR_BITS, BAUDRATE_MAP,
)

__all__ = [
    'InspireHandF1_RS485',
    'InspireHandF1_EtherCAT',
    'NUM_DOFS',
    'DOF_NAMES',
    'decode_error',
    'decode_status',
    'STATUS_CODES',
    'ERROR_BITS',
    'BAUDRATE_MAP',
]

# DDS handler imports — guarded so the SDK works without unitree_sdk2py
try:
    from .dds_handler import F1HandDDSHandler, F1HandDDSHandlerDouble
    __all__ += ['F1HandDDSHandler', 'F1HandDDSHandlerDouble']
except ImportError:
    pass
