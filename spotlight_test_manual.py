import cv2
import threading
import time
from picamera2 import Picamera2,Preview
from picamera2.encoders import H264Encoder, Quality
from libcamera import Transform
import subprocess
import os
import numpy as np
import serial
import socket
import spidev
import gpiod

#https://forums.raspberrypi.com/viewtopic.php?t=197801
#https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
#https://www.pythontutorial.net/python-concurrency/python-threading/

terminate = False  # Global variable to control end of program

output_dir = r"/home/tsgc/SB/SB"  


#This function inititalizes the SPI bus. Chip select is manual. 
def spi_init():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    global F1_pin 
    global F2_pin 
    global F1
    global F2 
    F1_pin = 26
    F2_pin = 12
    CS0_pin = 2
    CS1_pin = 3
    
    chip = gpiod.Chip('gpiochip0')
    F1 = chip.get_line(F1_pin)
    F1.request(consumer="F1", type=gpiod.LINE_REQ_DIR_OUT)
    F1.set_value(1)

    chip = gpiod.Chip('gpiochip0')
    F2 = chip.get_line(F2_pin)
    F2.request(consumer="F2", type=gpiod.LINE_REQ_DIR_OUT)
    F2.set_value(1)

    spi = spidev.SpiDev()
    bus = 0
    device = 0
    spi.open(bus, device)
    spi.max_speed_hz = 5000000 
    chip = gpiod.Chip('gpiochip0')
    CS0 = chip.get_line(CS0_pin)
    CS0.request(consumer="CS0", type=gpiod.LINE_REQ_DIR_OUT)
    CS0.set_value(1)
    CS1 = chip.get_line(CS1_pin)
    CS1.request(consumer="CS1", type=gpiod.LINE_REQ_DIR_OUT)
    CS1.set_value(1)
    #reset dac values to 0
    #startup sequence
    dac1a_val = 255
    dac1b_val = 255
    dac2a_val = 255
    dac2b_val = 255
    dac_update()
    time.sleep(0.5)
    dac1a_val = 0
    dac1b_val = 0
    dac2a_val = 0
    dac2b_val = 0
    dac_update()
    time.sleep(5)
    dac1a_val = 255
    dac1b_val = 255
    dac2a_val = 255
    dac2b_val = 255
    dac_update()
    time.sleep(0.5)
    dac1a_val = 0
    dac1b_val = 0
    dac2a_val = 0
    dac2b_val = 0
    dac_update()
    time.sleep(0.5)
    dac1a_val = 255
    dac1b_val = 255
    dac2a_val = 255
    dac2b_val = 255
    dac_update()
    time.sleep(0.5)
    dac1a_val = 25
    dac1b_val = 25
    dac2a_val = 25
    dac2b_val = 25
    dac_update()
    dac1a_val = 0
    dac1b_val = 0
    dac2a_val = 0
    dac2b_val = 0
    dac_update()
    

    

def spi_deinit():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    global F1_pin 
    global F2_pin 
    global F1
    global F2 
    CS0.release()
    CS1.release()
    F1.release()
    F2.release()
    spi.close()
#IMPORTANT - MUST USE WRITEBYTES AND INTEGER VALUES IN THE FOLLOWING FORMAT
def dac1a_write():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    LOADA = 0x21
    '''
    if (dac1a_val != 13):
        if(dac1a_val <= 13 and dac1a_val > 0):
            dac1a_val = 0
        if(dac1a_val > 50):
            dac1a_val = 50
    '''
    CS0.set_value(0)
    spi.writebytes([LOADA,dac1a_val])
    CS0.set_value(1)
def dac1b_write():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    '''
    if (dac1b_val != 13):
        if(dac1b_val <= 13 and dac1b_val > 0):
            dac1b_val = 0
        if(dac1b_val > 50):
            dac1b_val = 50
    '''
    LOADB = 0x22
    CS0.set_value(0)
    spi.writebytes([LOADB,dac1b_val])
    CS0.set_value(1)
def dac2a_write():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    '''
    if (dac2a_val != 13):
        if(dac2a_val <= 11 and dac2a_val > 0):
            dac2a_val = 0
        if(dac2a_val > 50):
            dac2a_val = 50
    '''
    LOADA = 0x21
    CS1.set_value(0)
    spi.writebytes([LOADA,dac2a_val])
    CS1.set_value(1)
def dac2b_write():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    '''
    if (dac2b_val != 13):
        if(dac2b_val <= 13 and dac2b_val > 0):
            dac2b_val = 0
        if(dac2b_val > 50):
            dac2b_val = 50
    '''
    LOADB = 0x22
    CS1.set_value(0)
    spi.writebytes([LOADB,dac2b_val])
    CS1.set_value(1)
def dac_update():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    dac1a_write()
    dac1b_write()
    dac2a_write()
    dac2b_write()
  



global CS0
global CS1 
global dac1a_val
global dac1b_val
global dac2a_val
global dac2b_val
global CS0_pin
global spi
global CS1_pin
#1a threshold = 13
#1b is 13
#2a 11
#
spi_init()
while True:
    y = input("Enter a light (1a,2b etc) to alter or type all: ")
    y = y.strip()
    x = input("Enter a value to set the dac: ")
    x = x.strip()
    x = int(x)
    if str(y) == "1a":
        dac1a_val= x
    elif str(y) == "1b":
        dac1b_val= x
    elif str(y) == "2a":
        dac2a_val= x
    elif str(y) == "2b":
        dac2b_val = x
    elif str(y) == "all":
        dac1a_val= x
        dac1b_val= x #-1
        dac2a_val = x
        dac2b_val= x #-1
    else:
        pass

    dac_update()


    time.sleep(0.1)
spi_deinit()
