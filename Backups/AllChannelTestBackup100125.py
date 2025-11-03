#!/usr/bin/env python3

import time
from SeniorDesignProject.modules.DACControl import update_dac  # Uses SPI1, as in your DACControl.py

# Map channel letters A–H to channels 1–8 as per your LTC1665 logic.
channel_map = {
    'a': 1,
    'b': 2,
    'c': 3,
    'd': 4,
    'e': 5,
    'f': 6,
    'g': 7,
    'h': 8,
}

def main():
    while True:
        print("\nAvailable channels: A, B, C, D, E, F, G, H")
        choice = input("Enter Y to send all channels high: ").strip().lower()

        # Drive all channels to 255 (full scale) for 5 seconds
        print(f"\nSetting channels to 255.")
        for channel in channel_map.values():
            update_dac(255, channel)
        time.sleep(5)

        # Turn the channel off (0)
        print(f"Turning off channels.")
        for channel in channel_map.values():
            update_dac(0, channel)

        # Prompt user to continue or exit
        again = input("Test another channel? (y/n): ").strip().lower()
        if again != 'y':
            print("Exiting DAC test.")
            break

if __name__ == "__main__":
    main()
