"""
  CycloneDDS IDL definition for Inspire RH56F1 hand capacitive touch data.
  F1 uses capacitive tactile sensors: per-finger normal/tangential force,
  direction, and proximity; plus 3 palm regions.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class inspire_hand_f1_touch(idl.IdlStruct, typename="inspire.inspire_hand_f1_touch"):
    little_normal_force: types.uint16
    little_tangential_force: types.uint16
    little_direction: types.uint16
    little_proximity_lo: types.uint16
    little_proximity_hi: types.uint16

    ring_normal_force: types.uint16
    ring_tangential_force: types.uint16
    ring_direction: types.uint16
    ring_proximity_lo: types.uint16
    ring_proximity_hi: types.uint16

    middle_normal_force: types.uint16
    middle_tangential_force: types.uint16
    middle_direction: types.uint16
    middle_proximity_lo: types.uint16
    middle_proximity_hi: types.uint16

    index_normal_force: types.uint16
    index_tangential_force: types.uint16
    index_direction: types.uint16
    index_proximity_lo: types.uint16
    index_proximity_hi: types.uint16

    thumb_normal_force: types.uint16
    thumb_tangential_force: types.uint16
    thumb_direction: types.uint16
    thumb_proximity_lo: types.uint16
    thumb_proximity_hi: types.uint16

    palm_left_normal_force: types.uint16
    palm_left_tangential_force: types.uint16
    palm_left_direction: types.uint16

    palm_center_normal_force: types.uint16
    palm_center_tangential_force: types.uint16
    palm_center_direction: types.uint16

    palm_right_normal_force: types.uint16
    palm_right_tangential_force: types.uint16
    palm_right_direction: types.uint16
