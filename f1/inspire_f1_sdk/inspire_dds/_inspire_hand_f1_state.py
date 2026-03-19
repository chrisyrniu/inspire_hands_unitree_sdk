"""
  CycloneDDS IDL definition for Inspire RH56F1 hand state.
  Adapted from DFTP inspire_hand_state but with INT16 for all fields
  (F1 uses 16-bit registers for error, status, and temperature).
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class inspire_hand_f1_state(idl.IdlStruct, typename="inspire.inspire_hand_f1_state"):
    pos_act: types.sequence[types.int16, 6]
    angle_act: types.sequence[types.int16, 6]
    force_act: types.sequence[types.int16, 6]
    current: types.sequence[types.int16, 6]
    err: types.sequence[types.int16, 6]
    status: types.sequence[types.int16, 6]
    temperature: types.sequence[types.int16, 6]
