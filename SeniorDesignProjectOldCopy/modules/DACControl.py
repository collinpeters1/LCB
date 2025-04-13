import spidev
import time

def update_dac(value, channel):
    """
    Update a single DAC channel on the LTC1665.
    The 16-bit command word is constructed as:
      - Bits 15-12: Channel address.
      - Bits 11-4:  8-bit DAC value.
      - Bits 3-0:   Set to 0.
    Command word: (channel << 12) | (value << 4)
    """
    if channel not in range(0, 8):
        raise ValueError("Channel must be between 0 and 7")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")

    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF

    spi = spidev.SpiDev()
    spi.open(0, 0)  # Bus 0, Device 0.
    spi.max_speed_hz = 5000000
    spi.writebytes([high_byte, low_byte])
    spi.close()

    print(f"Channel {channel} set to value {value}")
