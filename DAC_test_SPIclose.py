import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 5000000

def update_dac(spi, value, channel):
    if channel not in range(0, 8):
        raise ValueError("Channel must be between 0 and 7")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")
    
    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF
    
    spi.writebytes([high_byte, low_byte])
    print(f"Channel {channel} set to value {value}")

def initialize_unused_channels(active_channel):
    for ch in range(0, 8):
        if ch != active_channel:
            update_dac(spi, 0, ch)
            time.sleep(0.05)

if __name__ == "__main__":
    try:
        active_channel = 1  # DAC A (pin 2) is our active channel.
        print("Initializing unused channels to 0V...")
        initialize_unused_channels(active_channel)
        
        print("Starting single-channel DAC test on channel 1 (DAC A)...")
        print("Setting channel 1 to mid-scale value (128)...")
        update_dac(spi, 128, active_channel)
        time.sleep(2)
        
        print("Performing ramp test on channel 1...")
        for val in range(0, 256, 5):
            update_dac(spi, val, active_channel)
            time.sleep(0.1)
        for val in range(255, -1, -5):
            update_dac(spi, val, active_channel)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
