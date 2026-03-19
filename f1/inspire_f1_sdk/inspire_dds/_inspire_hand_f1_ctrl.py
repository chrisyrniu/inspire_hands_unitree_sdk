"""
  CycloneDDS IDL definition for Inspire RH56F1 hand control commands.
  Compatible with the DFTP ctrl message layout.
  mode bitmask: 0b0001=angle, 0b0010=position, 0b0100=force, 0b1000=speed.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class inspire_hand_f1_ctrl(idl.IdlStruct, typename="inspire.inspire_hand_f1_ctrl"):
    pos_set: types.sequence[types.int16, 6]
    angle_set: types.sequence[types.int16, 6]
    force_set: types.sequence[types.int16, 6]
    speed_set: types.sequence[types.int16, 6]
    mode: types.int8
