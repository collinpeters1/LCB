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
    


#This function was found online
def convert_h264_to_mp4(input_file, output_file):
    command = ['ffmpeg', '-y', '-i', os.path.join(output_dir, input_file), 
               '-c:v', 'copy', '-format', 'mp4', os.path.join(output_dir, output_file)]
    subprocess.run(command)

#This function was found in the picamera 2 documentation, I  edited it and got some info from online to help
def capture_video():
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    #os.system("v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev0")
    #time.sleep(0.01)
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration())
    #picam2.start_preview(Preview.DRM, x=0, y=0, width=1920, height=1080)
    encoder = H264Encoder()
    print("Starting video capture")
    video_path = os.path.join(output_dir, 'hdr_video.h264')
    picam2.start_recording(encoder, video_path, quality=Quality.HIGH)
    #picam2.start_preview(Preview.QTGL, x=100, y=200, width=800, height=600,
    #transform=Transform(hflip=1))
    global terminate
    while not terminate:  # Checks if termination is requested
        time.sleep(0.5)  # Short sleep to allow for responsive termination checks
    picam2.stop_recording()
    print("Video capture finished")
    

def capture_images(interval):
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    cap = cv2.VideoCapture(0)
    print("Starting Image capture")
    print("Enter 'stop' to exit program: ")
    global terminate
    while not terminate:  # Checks if termination is requested
        ret, frame = cap.read()
        if ret:
            print("image captured")
            process_image(frame)
        time.sleep(interval)
    cap.release()

def process_image(frame):
    global CS0
    global CS1 
    global dac1a_val
    global dac1b_val
    global dac2a_val
    global dac2b_val
    global CS0_pin
    global spi
    global CS1_pin
    global flag_1a
    global flag_1b
    global flag_2a
    global flag_2b
    global F1_pin 
    global F2_pin 
    global F1
    global F2 
    # Load pic
    image = frame
    # HSV conversion
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Select all pixels for value channel
    v_channel = hsv_image[:, :, 2]
    # Threshold the image, dont use the first return value which is just the threshold value
    threshold = 15
    dummy_value, threshold_image = cv2.threshold(v_channel, threshold, 255, cv2.THRESH_BINARY)

    dark_pixels = np.count_nonzero(threshold_image)
    total_pixels = v_channel.size
    percentage_dark = 100 - ((dark_pixels / total_pixels) * 100)
    # This part of the code segments the image and calculates darkness per segment.


    final_image = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2BGR)
    dimensions = threshold_image.shape
    div_val = dimensions[1] / 4
    segments = sorted({int(0), int(div_val), int(div_val*2), int(div_val*3),int(div_val*4)})
    percentage_dark_quadrants = [0, 0, 0, 0]
    
    target_darkness = 10
    #This is the value that the dacs will step at
    step = 1
    max_value = 25
    delay = 0.1
    threshold = 7
    old_dac1a_val = dac1a_val
    old_dac1b_val = dac1b_val
    old_dac2a_val = dac2a_val
    old_dac2b_val = dac2b_val
    
    
    #this splits the pic into four and calculates darkness
    for i in range(4):
        #calculate darkness
        quadrant_dark_pixels = np.count_nonzero(threshold_image[0:dimensions[0], segments[i]:segments[i+1]])
        percentage_dark_quadrants[i] = 100 - ((quadrant_dark_pixels / (total_pixels / 4)) * 100)
        cv2.putText(final_image, f'{percentage_dark_quadrants[i]:.2f}%', (segments[i]+180, dimensions[0]//2), cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 3) 
        darkness_offset = percentage_dark_quadrants[i] - target_darkness
        bit_position = 2 * (3-i)
        
        if (i == 1):
            if(darkness_offset > threshold and dac1a_val <= max_value - step):
                flag_1a = True
                if( dac1a_val == 0):
                    dac1a_val = 15
                    dac_update()
                else:
                    dac1a_val += step
                    dac_update()
                
            elif(darkness_offset < (-1 * threshold) and dac1a_val >= 0 + step):
                flag_1a = True
                dac1a_val -= step
                if dac1a_val < 15:
                    dac1a_val = 0
                dac_update()
            else:
                pass
     
        if (i == 0):
            if(darkness_offset > threshold and dac1b_val <= max_value - step):
                flag_1b = True
                if( dac1b_val == 0):
                    dac1b_val = 15
                    dac_update()
                else:
                    dac1b_val += step
                    dac_update()
                        
            elif(darkness_offset < (-1 * threshold) and dac1b_val >= 0 + step):
                flag_1b = True
                dac1b_val -= step
                if dac1b_val <= 15:
                    dac1b_val = 0
                dac_update()
            else:
                pass
   
        if (i == 3):
            if(darkness_offset > threshold and dac2a_val <= max_value - step):
                flag_2a = True
                if dac2a_val == 0:
                    dac2a_val = 13
                else:
                    dac2a_val += step
                    dac_update()
            elif(darkness_offset < (-1 * threshold) and dac2a_val >= 0 + step):
                flag_2a = True
                dac2a_val -= step
                if dac2a_val < 13:
                    dac2a_val = 0
                dac_update()
            else:
                pass

        if (i == 2) :
            if(darkness_offset > threshold and dac2b_val <= max_value - step):
                flag_2b = True
                if dac2b_val == 0:
                    dac2b_val = 15
                else:
                    dac2b_val += step
                    dac_update()
            elif(darkness_offset < (-1 * threshold) and dac2b_val >= 0 + step):
                flag_2b = True
                dac2b_val -= step
                if dac2b_val < 15:
                    dac2b_val = 0
                dac_update()
            else:
                pass


    cv2.imshow("Annotated Image", final_image)
    cv2.waitKey(1000)


#same as above



def main():
    interval = 0.001
    spi_init()
    global terminate
    video_thread = threading.Thread(target=capture_video)
    image_thread = threading.Thread(target=capture_images, args=(interval,))
    video_thread.start()
    image_thread.start()

    while not terminate:
        user_input = input()
        if user_input.lower() == "stop":
                terminate = True
                cv2.destroyAllWindows()

    video_thread.join()
    image_thread.join()
    convert_h264_to_mp4('hdr_video.h264', 'test.mp4')


    spi_deinit()
    
if __name__ == "__main__":
    main()
