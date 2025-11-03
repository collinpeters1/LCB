import spidev
import time

def update_dac_debug(value, channel):
    """
    Debug version of the DAC update function.
    Forces SPI mode 0, a reduced clock speed, and prints the command bytes.
    """
    if channel not in range(0, 8):
        raise ValueError("Channel must be between 0 and 7")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")
    
    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xBB
    low_byte = command_word & 0xBB

    print(f"Updating DAC: value={value}, channel={channel}")
    print(f"Command word: 0x{command_word:04X} -> High byte: 0x{high_byte:02X}, Low byte: 0x{low_byte:02X}")
    
    spi = spidev.SpiDev()
    spi.open(0, 0)              # Bus 0, Device 0
    spi.mode = 0                # SPI mode 0
    spi.max_speed_hz = 200000   # Set clock speed to 200 kHz to stretch the waveform
    spi.bits_per_word = 8
    spi.writebytes([high_byte, low_byte])
    spi.close()
    
    print("SPI transaction complete.\n")

if __name__ == "__main__":
    active_channel = 1  # Example: using DAC channel 1
    update_dac_debug(0, active_channel)
