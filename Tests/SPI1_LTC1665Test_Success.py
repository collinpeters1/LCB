import spidev
import time

def update_dac(value, channel):
    """
    Update a single DAC channel on the LTC1665.
    
    The 16-bit command word is structured as follows:
      - Bits 15-12: Channel address (A3-A0).
          For DAC A (channel 1), use 0001; for DAC H (channel 8), use 1000.
      - Bits 11-4:  8-bit DAC value (D7-D0).
      - Bits 3-0:   Don't care bits (set to 0).
    
    Command word = (channel << 12) | (value << 4)
    This command word is split into two bytes (big-endian) and sent over SPI.
    """
    # Allow channels 1-8 only.
    if channel not in range(1, 9):
        raise ValueError("Channel must be between 1 and 8")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")
    
    # Build the command word.
    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF
    
    print(f"Updating DAC: Channel {channel}, value {value}")
    print(f"Command word: 0x{command_word:04X} -> Bytes: 0x{high_byte:02X}, 0x{low_byte:02X}")
    
    spi.writebytes([high_byte, low_byte])
    time.sleep(0.001)  # Optional delay for settling

def initialize_unused_channels(active_channel):
    """
    Set all DAC channels (channels 1 through 8) except the active one to 0.
    """
    for ch in range(1, 9):
        if ch != active_channel:
            update_dac(0, ch)
            time.sleep(0.05)  # Small delay between updates

def main():
    global spi
    # Open SPI bus 10, device 0, which corresponds to SPI1 (/dev/spidev10.0)
    spi = spidev.SpiDev()
    spi.open(1, 0)
    spi.max_speed_hz = 5000000  # 5 MHz; adjust as needed
    spi.mode = 0  # SPI mode 0 (CPOL=0, CPHA=0)
    
    try:
        active_channel = 1  # Use channel 1 (DAC A)
        print("Initializing unused DAC channels to 0V...")
        initialize_unused_channels(active_channel)
        
        print("Starting single-channel DAC test on channel 1 (DAC A)...")
        # Fixed mid-scale test
        print("Setting channel 1 to mid-scale value (128)...")
        update_dac(128, active_channel)
        time.sleep(2)  # Wait to observe the output
        
        # Ramp test: Increase from 0 to 255 and then decrease back to 0.
        print("Performing ramp test on channel 1 (increasing)...")
        for val in range(0, 256, 5):
            update_dac(val, active_channel)
            time.sleep(0.1)
        print("Performing ramp test on channel 1 (decreasing)...")
        for val in range(255, -1, -5):
            update_dac(val, active_channel)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        spi.close()
        print("SPI closed.")

if __name__ == "__main__":
    main()
