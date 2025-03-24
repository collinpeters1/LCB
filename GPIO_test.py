import gpiod
import time
chip = gpiod.Chip('gpiochip0')
CS0_pin = 3 
CS1_pin = 2
CS0 = chip.get_line(CS0_pin)
CS0.request(consumer="CS0", type=gpiod.LINE_REQ_DIR_OUT)
CS0.set_value(0)
CS1 = chip.get_line(CS1_pin)
CS1.request(consumer="CS1", type=gpiod.LINE_REQ_DIR_OUT)
CS1.set_value(0)
time.sleep(5)
CS0.set_value(1)
CS1.set_value(1)
time.sleep(5)
CS0.set_value(0)
CS1.set_value(0)
CS0.release()
CS1.release()