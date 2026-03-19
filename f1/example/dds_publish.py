"""
DDS control publisher for Inspire RH56F1 hands.

Publishes angle commands to both left and right F1 hands via DDS,
toggling between open and closed positions at 10 Hz.

Usage:
    python dds_publish.py [network_interface]

The F1 DDS driver must be running to relay these commands to hardware.
"""

import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from inspire_f1_sdk.inspire_dds import inspire_hand_f1_ctrl

# F1 angle register order: [little, ring, middle, index, thumb_bend, thumb_rotate]
# Ranges (0.1 deg): fingers 900-1740, thumb_bend 1100-1350, thumb_rotate 600-1800
OPEN  = [1740, 1740, 1740, 1740, 1100, 600]
CLOSE = [900,  900,  900,  900,  1350, 1800]


def make_cmd(angles):
    return inspire_hand_f1_ctrl(
        pos_set=[0] * 6,
        angle_set=angles,
        force_set=[0] * 6,
        speed_set=[0] * 6,
        mode=0b0001,
    )


if __name__ == "__main__":
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    pub_l = ChannelPublisher("rt/inspire_hand_f1/ctrl/l", inspire_hand_f1_ctrl)
    pub_l.Init()
    pub_r = ChannelPublisher("rt/inspire_hand_f1/ctrl/r", inspire_hand_f1_ctrl)
    pub_r.Init()

    print("Opening hands...")
    cmd = make_cmd(OPEN)
    pub_l.Write(cmd)
    pub_r.Write(cmd)
    time.sleep(2.0)

    print("Closing hands...")
    cmd = make_cmd(CLOSE)
    pub_l.Write(cmd)
    pub_r.Write(cmd)
    time.sleep(2.0)

    toggle = 0
    for step in range(100000):
        if (step + 1) % 10 == 0:
            toggle = 1 - toggle

        target = CLOSE if toggle else OPEN
        angles = np.array(target, dtype=np.int16)
        angles = np.clip(angles, [900, 900, 900, 900, 1100, 600],
                                 [1740, 1740, 1740, 1740, 1350, 1800])

        cmd = make_cmd(angles.tolist())

        if pub_l.Write(cmd) and pub_r.Write(cmd):
            print(f"[{step:6d}] angles: {angles.tolist()}")
        else:
            print("Waiting for subscriber...")

        time.sleep(0.1)
