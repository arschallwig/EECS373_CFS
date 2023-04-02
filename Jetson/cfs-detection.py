#!/usr/bin/env python3

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
import Jetson.GPIO as GPIO

import time
import serial

# Pin Definitions:
i_signal_pin = 23 # C8 on the STM
o_signal_pin_out = 24 #C9 on the STM

def main():
    print("Preparing GPIO Pins")

    # Pin setup
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(i_signal_pin, GPIO.IN)
    GPIO.setup(o_signal_pin_out, GPIO.OUT, initial=GPIO.HIGH)

    # Wait for the shared signal to go high - first detection
    print("Waiting for first detection signal")
    GPIO.wait_for_edge(i_signal_pin, GPIO.RISING)
    GPIO.output(o_signal_pin_out, GPIO.LOW)

    # Prepare UART serial overhead
    serial_port = serial.Serial(
        port = "/dev/ttyTHS1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    time.sleep(1)

    # Prepare CV 
    net = detectNet("pednet", threshold=0.3)
    camera = videoSource("csi://0")
    # display = videoOutput("display://0")
    num_peds = 0

    # Run first frame of detection
    print("-------------------- RUNNING FIRST DETECTION --------------------")
    img = camera.Capture()
    detections = net.Detect(img)
    num_peds += len(detections)
    # display.Render(img)

    # Done with first detection, set shared signal low
    GPIO.output(o_signal_pin_out, GPIO.HIGH)

    # Wait for the shared signal to go high - second detection
    print("Waiting for second detection signal")
    GPIO.wait_for_edge(i_signal_pin, GPIO.RISING)
    GPIO.output(o_signal_pin_out, GPIO.LOW)


    # Run second frame of detection
    print("-------------------- RUNNING SECOND DETECTION --------------------")
    img = camera.Capture()
    detections = net.Detect(img)
    num_peds += len(detections)
    # display.Render(img)

    # Send count over uart connection to STM32
    print("-------------------- DETECTED " + str(num_peds) + " PEDS --------------------")
    serial_port.write(num_peds.to_bytes(4, 'little'))
    
    # Done with second detection, set shared signal low
    GPIO.output(o_signal_pin_out, GPIO.HIGH)

    # # Clean up GPIO
    serial_port.close()
    GPIO.cleanup() #TODO will this even clean up the wake up/sleep interrupt pin?

if __name__ == '__main__':
    main()
