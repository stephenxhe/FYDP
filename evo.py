#!/usr/bin/env python3
###### TeraRanger Evo Example Code STD #######
#                                            #
# All rights reserved Terabee France (c) 2018#
#                                            #
############ www.terabee.com #################

import serial
import serial.tools.list_ports
import sys
import crcmod.predefined  # To install: pip install crcmod
import cv2
import threading

BLUE = (255, 0, 0)
RED = (0, 0, 255)
LINEWIDTH = 10

def findEvo():
    # Find Live Ports, return port name if found, NULL if not
    print('Scanning all live ports on this PC')
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # print p # This causes each port's information to be printed out.
        if "5740" in p[2]:
            print('Evo found on port ' + p[0])
            return p[0]
    return 'NULL'

def openEvo(portname):
    print('Attempting to open port...')
    # Open the Evo and catch any exceptions thrown by the OS
    print(portname)
    evo = serial.Serial(portname, baudrate=115200, timeout=2)
    # Send the command "Binary mode"
    set_bin = (0x00, 0x11, 0x02, 0x4C)
    # Flush in the buffer
    evo.flushInput()
    # Write the binary command to the Evo
    evo.write(set_bin)
    # Flush out the buffer
    evo.flushOutput()
    print('Serial port opened')
    return evo

def get_evo_range(evo_serial):
    crc8_fn = crcmod.predefined.mkPredefinedCrcFun('crc-8')
    # Read one byte
    data = evo_serial.read(1)
    if data == b'T':
        # After T read 3 bytes
        frame = data + evo_serial.read(3)
        if frame[3] == crc8_fn(frame[0:3]):
            # Convert binary frame to decimal in shifting by 8 the frame
            rng = frame[1] << 8
            rng = rng | (frame[2] & 0xFF)
        else:
            return "CRC mismatch. Check connection or make sure only one progam access the sensor port."
    # Check special cases (limit values)
    else:
        return "Wating for frame header"

    # Checking error codes
    if rng == 65535: # Sensor measuring above its maximum limit
        dec_out = float('inf')
    elif rng == 1: # Sensor not able to measure
        dec_out = float('nan')
    elif rng == 0: # Sensor detecting object below minimum range
        dec_out = -float('inf')
    else:
        # Convert frame in meters
        dec_out = rng / 1000.0
    return dec_out

def evo_Thread(evo):
    print('start')
    while True:
        try:
            dist = get_evo_range(evo)
            print(dist)
        except serial.serialutil.SerialException:
            print("Device disconnected (or multiple access on port). Exiting...")

cap = cv2.VideoCapture(0)    						# camera selection
cap.set(cv2.CAP_PROP_FPS, 60)           # Set camera FPS to 60 FPS
cv2.namedWindow("Webcam capture")
windowCenterX = int(cv2.getWindowImageRect("Webcam capture")[2]/2)
windowCenterY = int(cv2.getWindowImageRect("Webcam capture")[3]/2)

if __name__ == "__main__":

    print('Starting Evo data streaming')
    # Get the port the evo has been connected to
    port = findEvo()

    if port == 'NULL':
        print("Sorry couldn't find the Evo. Exiting.")
        sys.exit()
    else:
        evo = openEvo(port)
    
    evoThread = threading.Thread(target=evo_Thread, args=(evo,))
    evoThread.start()
    while True:
        # draw center cross
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        cv2.line(image, (windowCenterX+LINEWIDTH, windowCenterY), (windowCenterX-LINEWIDTH, windowCenterY), BLUE, 2)
        cv2.line(image, (windowCenterX, windowCenterY+LINEWIDTH), (windowCenterX, windowCenterY-LINEWIDTH), BLUE, 2)

        cv2.imshow('MediaPipe Face Detection', image)
        cv2.waitKey(1)

    evo.close()
    cap.release()
    sys.exit()
