import spidev
import time
import RPi.GPIO as GPIO

# Use GPIO 25 as the manual chip enable (active low)
CE_PIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(CE_PIN, GPIO.OUT)
GPIO.output(CE_PIN, GPIO.HIGH)  # inactive initially

# Open SPI connection on bus 0, device 0.
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 5000000
# Do not set spi.no_cs; instead, disconnect the default CE pin from the DAC.

def update_dac(value, channel):
    """
    Update a single DAC channel on the LTC1665.
    
    The 16-bit command word is:
      - Bits 15-12: Channel address.
      - Bits 11-4:  8-bit DAC value.
      - Bits 3-0:   Don't care (0).
    
    :param value: Integer 0-255 representing the DAC output.
    :param channel: DAC channel number (0-7).
    """
    if channel not in range(0, 8):
        raise ValueError("Channel must be between 0 and 7")
    if not (0 <= value <= 255):
        raise ValueError("Value must be between 0 and 255")
    
    command_word = (channel << 12) | (value << 4)
    high_byte = (command_word >> 8) & 0xFF
    low_byte = command_word & 0xFF
    
    # Manually assert chip enable using GPIO 25.
    GPIO.output(CE_PIN, GPIO.LOW)
    time.sleep(0.001)  # short settling delay
    
    spi.writebytes([high_byte, low_byte])
    time.sleep(0.001)  # allow SPI transaction to complete
    
    GPIO.output(CE_PIN, GPIO.HIGH)
    print(f"Channel {channel} set to value {value}")

# Test parameters
active_channel = 1  # using channel 1 for DAC A
# With a 5V reference, 1.5V corresponds to roughly:
value_1_5 = int((1.5 / 5.0) * 255)  # around 77
value_0 = 0

cycles = 4
delay = 3.75  # Each state lasts ~3.75 seconds (total ~30 seconds)

try:
    for i in range(cycles):
        print(f"Cycle {i+1}: Setting voltage to 1.5V")
        update_dac(value_1_5, active_channel)
        time.sleep(delay)
        
        print(f"Cycle {i+1}: Setting voltage to 0V")
        update_dac(value_0, active_channel)
        time.sleep(delay)
finally:
    spi.close()
    GPIO.cleanup()
