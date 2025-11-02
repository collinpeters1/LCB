import spidev
import time

def update_dac(value, channel):
    """
    Update a single DAC channel on the LTC1665.
    Channel must be from 1 (DAC A) to 8 (DAC H).
    """
    if channel not in range(1, 9):
        raise ValueError("Channel must be between 1 and 8")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")

    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF

    spi = spidev.SpiDev()
    spi.open(1, 0)  # Use SPI1 instead of SPI0
    spi.max_speed_hz = 2000000
    spi.mode = 0
    spi.writebytes([high_byte, low_byte])
    spi.close()

    print(f"Channel {channel} set to value {value}")
