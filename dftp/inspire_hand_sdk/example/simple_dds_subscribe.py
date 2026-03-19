#!/usr/bin/env python3
"""
Simple DDS subscriber to get hand status without GUI visualization
Only prints the received values to console
"""
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from inspire_sdkpy import inspire_hand_defaut, inspire_dds

class SimpleDDSHandler:
    def __init__(self, network=None, LR='r'):
        # Initialize DDS
        if network is None:
            ChannelFactoryInitialize(0)
            print("Initialized DDS with default settings")
        else:
            ChannelFactoryInitialize(0, network)
            print(f"Initialized DDS with network: {network}")
        
        # Create subscribers
        self.sub_states = ChannelSubscriber(f"rt/inspire_hand/state/{LR}", inspire_dds.inspire_hand_state)
        self.sub_states.Init(self.update_data_state, 10)
        
        self.sub_touch = ChannelSubscriber(f"rt/inspire_hand/touch/{LR}", inspire_dds.inspire_hand_touch)
        self.sub_touch.Init(self.update_data_touch, 10)
        
        self.states = {}
        self.touch = {}
        self.message_count = 0
        
        print(f"Subscribed to channels:")
        print(f"  - rt/inspire_hand/state/{LR}")
        print(f"  - rt/inspire_hand/touch/{LR}")
        print("Waiting for data...")

    def update_data_state(self, states_msg):
        """Update hand state data"""
        self.states = {
            'POS_ACT': states_msg.pos_act,
            'ANGLE_ACT': states_msg.angle_act,
            'FORCE_ACT': states_msg.force_act,
            'CURRENT': states_msg.current,
            'ERROR': states_msg.err,
            'STATUS': states_msg.status,
            'TEMP': states_msg.temperature
        }
        self.message_count += 1
        self.print_states()

    def update_data_touch(self, touch_msg):
        """Update touch sensor data"""
        # Get touch data from the message
        self.touch = {
            'touch_data': touch_msg.touch_data if hasattr(touch_msg, 'touch_data') else None,
            'timestamp': time.time()
        }

    def print_states(self):
        """Print current hand states"""
        print(f"\n--- Message #{self.message_count} ---")
        print(f"Timestamp: {time.strftime('%H:%M:%S')}")
        
        # Print position data
        if self.states.get('POS_ACT'):
            print(f"Position (POS_ACT): {self.states['POS_ACT']}")
        
        # Print angle data
        if self.states.get('ANGLE_ACT'):
            print(f"Angles (ANGLE_ACT): {self.states['ANGLE_ACT']}")
        
        # Print force data
        if self.states.get('FORCE_ACT'):
            print(f"Forces (FORCE_ACT): {self.states['FORCE_ACT']}")
        
        # Print current data
        if self.states.get('CURRENT'):
            print(f"Current: {self.states['CURRENT']}")
        
        # Print error status
        if self.states.get('ERROR'):
            print(f"Errors: {self.states['ERROR']}")
        
        # Print status
        if self.states.get('STATUS'):
            print(f"Status: {self.states['STATUS']}")
        
        # Print temperature
        if self.states.get('TEMP'):
            print(f"Temperature: {self.states['TEMP']}")
        
        print("-" * 40)

    def print_summary(self):
        """Print summary of received data"""
        print(f"\n=== SUMMARY ===")
        print(f"Total messages received: {self.message_count}")
        print(f"Available data keys: {list(self.states.keys())}")
        if self.touch:
            print(f"Touch data available: {list(self.touch.keys())}")

def main():
    print("Simple DDS Hand Status Subscriber")
    print("=================================")
    
    # Parse command line arguments
    LR = 'r'  # Default to right hand
    network = None
    
    if len(sys.argv) > 1:
        if sys.argv[1] in ['l', 'L', 'left']:
            LR = 'l'
        elif sys.argv[1] in ['r', 'R', 'right']:
            LR = 'r'
        else:
            network = sys.argv[1]
    
    if len(sys.argv) > 2:
        if sys.argv[2] in ['l', 'L', 'left']:
            LR = 'l'
        elif sys.argv[2] in ['r', 'R', 'right']:
            LR = 'r'
    
    print(f"Monitoring {LR.upper()} hand")
    
    try:
        # Create DDS handler
        handler = SimpleDDSHandler(network=network, LR=LR)
        
        # Keep running and print data
        print("Press Ctrl+C to stop...")
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        handler.print_summary()
        print("Goodbye!")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
