import spidev
import time

def spi_loopback_test():
    """
    Test the SPI bus in loopback mode with a very low clock speed.
    Connect MOSI (GPIO10) directly to MISO (GPIO9) with a jumper.
    """
    spi = spidev.SpiDev()
    spi.open(0, 0)         # Use SPI bus 0, device 0.
    spi.mode = 0           # SPI mode 0: CPOL=0, CPHA=0.
    spi.max_speed_hz = 100000  # Set a very slow clock (100 kHz) for clear transitions.
    spi.bits_per_word = 8

    # Known test data.
    tx_data = [0x55, 0xAA, 0x00, 0xFF]
    print("Transmitting data:", [hex(b) for b in tx_data])
    
    rx_data = spi.xfer2(tx_data)
    print("Received data:", [hex(b) for b in rx_data])
    spi.close()

if __name__ == "__main__":
    spi_loopback_test()
    time.sleep(1)
