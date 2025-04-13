#!/usr/bin/env python3
import spidev
import time

def update_dac(value, channel):
    """
    Update a single DAC channel on the LTC1665CN.

    The 16-bit command word is structured as follows:
      - Bits 15-12: Channel address (A3-A0). For DAC A (pin 2), use 0001; for DAC H, use 1000.
      - Bits 11-4:  8-bit DAC value (D7-D0).
      - Bits 3-0:   Don't care bits (set to 0).

    The command word is constructed as:
      command_word = (channel << 12) | (value << 4)
    
    Valid channel addresses range from 0001 (channel 1, DAC A) to 1000 (channel 8, DAC H).
    """
    if channel not in range(1, 9):   # Valid channels: 1 through 8
        raise ValueError("Channel must be between 1 and 8")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")
    
    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF
    
    spi = spidev.SpiDev()
    spi.open(0, 0)           # Bus 0, Device 0; adjust if needed.
    spi.max_speed_hz = 500 # 5 MHz; adjust as necessary.
    spi.writebytes([high_byte, low_byte])
    spi.close()
    
    print(f"Channel {channel} set to value {value}")

def initialize_unused_channels(active_channel):
    """
    Set all DAC channels except the active one to 0V.
    Iterates over channels 1 to 8, skipping the active channel.
    """
    for ch in range(1, 9):
        if ch != active_channel:
            update_dac(0, ch)
            time.sleep(0.05)  # Small delay between channel updates

if __name__ == "__main__":
    try:
        # Set the active channel: DAC A (pin 2) is channel 1.
        active_channel = 1
        print("Initializing unused channels to 0V...")
        initialize_unused_channels(active_channel)
        
        print("Starting single-channel DAC test on channel 1 (DAC A)...")
        
        # Fixed mid-scale test.
        print("Setting channel 1 to mid-scale value (128)...")
        update_dac(128, active_channel)
        time.sleep(2)  # Pause to observe the output.
        
        # Ramp test: Increase from 0 to 255 and then decrease back to 0.
        print("Performing ramp test on channel 1...")
        for val in range(0, 256, 5):
            update_dac(val, active_channel)
            time.sleep(0.1)
        for val in range(255, -1, -5):
            update_dac(val, active_channel)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
