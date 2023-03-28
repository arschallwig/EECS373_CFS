#!/usr/bin/env python3

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
import Jetson.GPIO as GPIO

import time
import serial

# Pin Definitions:
signal_pin = 18

def main():
    net = detectNet("ssd-mobilenet-v2", threshold=0.5)
    camera = videoSource("csi://0")      # '/dev/video0' for V4L2
    display = videoOutput("display://0") # 'my_video.mp4' for file

    while display.IsStreaming():
        img = camera.Capture()
        detections = net.Detect(img)
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))


#     print("Preparing GPIO Pins")

#     # Pin setup
#     GPIO.setmode(GPIO.BOARD)
#     GPIO.setup(signal_pin, GPIO.IN)
#    # Prepare UART serial overhead
#     serial_port = serial.Serial(
#         port = "/dev/ttyTHS1",
#         baudrate=115200,
#         bytesize=serial.EIGHTBITS,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#     )
#     time.sleep(1)

#     # Prepare CV 
#     net = detectNet("pednet", threshold=0.5)
#     camera = videoSource("csi://0")
#     display = videoOutput("display://0")
#     num_peds = 0

#     # Run first frame of detection
#     img = camera.Capture()
#     detections = net.Detect(img)
#     num_peds += len(detections)
#     display.Render(img)

#     print("got here")

#     # Wait for the shared signal to go high
#     GPIO.wait_for_edge(signal_pin, GPIO.RISING)
#     # time.sleep(10)

#     # Run second frame of detection
#     img = camera.Capture()
#     detections = net.Detect(img)
#     num_peds += len(detections)
#     display.Render(img)

#     # Send count over uart connection to STM32
#     serial_port.write(num_peds)

#     # Clean up GPIO
#     serial_port.close()
#     GPIO.cleanup() #TODO will this even clean up the wake up/sleep interrupt pin?

if __name__ == '__main__':
    main()
