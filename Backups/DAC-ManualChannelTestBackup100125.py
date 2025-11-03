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
    'h': 8
}

def main():
    while True:
        print("\nAvailable channels: A, B, C, D, E, F, G, H")
        choice = input("Enter the DAC channel to test (A–H): ").strip().lower()

        if choice not in channel_map:
            print("Invalid input. Please enter a letter A through H.")
            continue

        # Drive that channel to 255 (full scale) for 5 seconds
        ch_num = channel_map[choice]
        print(f"\nSetting channel {choice.upper()} (#{ch_num}) to 255.")
        #update_dac(75, ch_num)
        #time.sleep(.5)
        update_dac(124, ch_num)
        time.sleep(2)
        #update_dac(255, ch_num)
        #time.sleep(3)

        # Turn the channel off (0)
        print(f"Turning off channel {choice.upper()} (#{ch_num}).")
        update_dac(0, ch_num)

        # Prompt user to continue or exit
        again = input("Test another channel? (y/n): ").strip().lower()
        if again != 'y':
            print("Exiting DAC test.")
            break

if __name__ == "__main__":
    main()
