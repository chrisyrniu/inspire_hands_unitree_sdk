#!/usr/bin/env python3
"""
Simple DDS test script to verify communication
"""
import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

def test_dds_communication():
    print("Testing DDS communication...")
    
    # Initialize DDS
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
        print(f"Initialized DDS with network: {sys.argv[1]}")
    else:
        ChannelFactoryInitialize(0)
        print("Initialized DDS with default settings")
    
    # Create publisher
    print("Creating publisher...")
    pub = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    pub.Init()
    
    # Create subscriber
    print("Creating subscriber...")
    sub = ChannelSubscriber("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
    
    received_messages = []
    
    def callback(msg):
        received_messages.append(msg)
        print(f"Received message: angle_set={msg.angle_set}, mode={msg.mode}")
    
    sub.Init(callback, 10)
    
    # Test message
    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    cmd.angle_set = [500, 500, 500, 500, 500, 500]
    cmd.mode = 0b0001
    
    print("Sending test message...")
    success = pub.Write(cmd)
    print(f"Publish success: {success}")
    
    # Wait for message to be received
    print("Waiting for message reception...")
    time.sleep(2.0)
    
    if received_messages:
        print(f"✅ DDS communication working! Received {len(received_messages)} messages")
        return True
    else:
        print("❌ DDS communication failed! No messages received")
        return False

if __name__ == "__main__":
    test_dds_communication()
