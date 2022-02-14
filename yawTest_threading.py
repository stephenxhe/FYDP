import sys
import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import serial
import serial.tools.list_ports
import threading
from threading import Lock
from queue import Queue
import atexit
import crcmod.predefined

data_lock = Lock()
refPt = [None,None]
BLUE = (255, 0, 0)
RED = (0,0,255)
GREEN = (0,255,0)

yaw_tolerance = 5 # [degrees]

valid_targets = ['car', 'bus', 'person']

# cv
# this can be split into separate threads i think
def cv_thread(pq, yq):
    cap = cv.VideoCapture(0)                # camera selection
    cap.set(cv.CAP_PROP_FPS, 30)            # Set camera FPS to 60 FPS
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)  # Set the width of the camera image to 1280
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)  # Set the vertical width of the camera image to 720
    whT = 320                               # yolov3 image size

    cv.namedWindow("Webcam capture")
    windowCenterX = int(cv.getWindowImageRect("Webcam capture")[2]/2)
    yq.put(windowCenterX)
    windowCenterY = int(cv.getWindowImageRect("Webcam capture")[3]/2)
    pq.put(windowCenterY)

    def mouseCallback(event, x, y, flags, param):
        global refPt
        if event == cv.EVENT_LBUTTONUP:
            refPt = [x, y]
        elif event == cv.EVENT_RBUTTONUP:
            refPt = [None, None]
    
    cv.setMouseCallback("Webcam capture", mouseCallback)    # capture mouse clicks for selecting objects

    classesFile = 'coco.names'
    classNames = []
    with open(classesFile, 'rt') as f:
        classNames = f.read().rstrip('\n').split('\n')

    modelConfiguration = 'yolov3.cfg'
    modelWeights = 'yolov3.weights'
    net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)

    confidenceThresh = 0.1  # min confidence to draw a box
    nmsThreshold = 0.5      # lower num = more aggressive, fewer boxes

    def isInBox(pointX, pointY, boundX, boundY, boundW, boundH):
        if pointX is None or pointY is None:
            return False
        if pointX < boundX or pointY < boundY or pointX > boundX + boundW or pointY > boundY + boundH:
            return False
        return True

    def findObjects(outputs,img):
        global refPt
        hT, wT, _ = img.shape
        boundingBoxes, classIdxs, confidences = [], [], []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classIdx = np.argmax(scores)
                confidence = scores[classIdx]

                if confidence > confidenceThresh:
                    w,h = int(detection[2]*wT), int(detection[3]*hT)                # pixel values
                    x,y = int(detection[0]*wT-0.5*w), int(detection[1]*hT-0.5*h)    # top left corner of box

                    boundingBoxes.append([x,y,w,h])
                    classIdxs.append(classIdx)
                    confidences.append(float(confidence))
        indicesToKeep = cv.dnn.NMSBoxes(boundingBoxes, confidences, confidenceThresh, nmsThreshold)       # removes overlapping boxes

        for i in indicesToKeep:
            if classNames[classIdxs[i]] in valid_targets:
                box = boundingBoxes[i]
                x,y,w,h = box[0], box[1], box[2], box[3]

                if isInBox(refPt[0], refPt[1], x,y,w,h):
                    refPt = [int(x+0.5*w), int(y+0.5*h)]
                    print(f"{refPt[0]} {windowCenterX}")

                    if abs(refPt[0] - windowCenterX) <= 50:
                        cv.rectangle(img, (x,y), (x+w, y+h), GREEN, 2)
                        cv.putText(img, 'Target Vehicle', (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
                    else:
                        cv.rectangle(img, (x,y), (x+w, y+h), RED, 2)
                        cv.putText(img, 'Target Vehicle', (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

                elif refPt[0] is None and refPt[1] is None:
                    cv.rectangle(img, (x,y), (x+w, y+h), BLUE, 2)
                    cv.putText(img, f'{classNames[classIdxs[i]]} {int(confidences[i]*100)}%', (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)
                # else:
                #     refPt = [None, None]

    def drawTarget(img, point):
        x, y = point[0], point[1]
        
        if x is not None and y is not None:
            if abs(x - windowCenterX) <= 50:
                # vertical line
                cv.line(img, (x, 0), (x, 2*windowCenterY), GREEN, 2)
                # horizontal line
                cv.line(img, (0, y), (2*windowCenterX, y), GREEN, 2)
            else:
                # vertical line
                cv.line(img, (x, 0), (x, 2*windowCenterY), RED, 2)
                # horizontal line
                cv.line(img, (0, y), (2*windowCenterX, y), RED, 2)
    
    # main loop
    pTime = 0
    while True:
        success, img = cap.read()
        
        if success:
            blob = cv.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)      # read docs for params
            net.setInput(blob)

            layerNames = net.getLayerNames()
            outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
            
            outputs = net.forward(outputNames)

            findObjects(outputs, img)
            drawTarget(img, refPt)

            # show fps
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
            cv.putText(img, f'FPS: {int(fps)}', (20,20), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

            # # center cross
            cv.line(img, (windowCenterX+10, windowCenterY), (windowCenterX-10, windowCenterY), BLUE, 2)
            cv.line(img, (windowCenterX, windowCenterY+10), (windowCenterX, windowCenterY-10), BLUE, 2)
            cv.imshow("Webcam capture", img)
            cv.waitKey(1)

# rangefinder
def tof_thread():
    # default evo functions
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

    # main thread code
    print('Starting Evo data streaming')
    # Get the port the evo has been connected to
    port = findEvo()

    if port == 'NULL':
        print("Sorry couldn't find the Evo. Exiting.")
        sys.exit()
    else:
        evo = openEvo(port)
    
    # main loop
    while True:
        try:
            dist = get_evo_range(evo)
            print(f"{dist} m")
        except serial.serialutil.SerialException:
            print("Device disconnected (or multiple access on port). Exiting...")

# serial helper functions
def serial_write(data: str, device: serial.Serial):
    device.write(bytes(data, 'utf-8'))

def serial_read(device: serial.Serial):
    return device.readline()

# serial
def serial_thread(pq, yq, arduino):
    windowCenterX = yq.get()
    windowCenterY = pq.get()

    # main loop
    while True:
        if refPt != [None, None]:
            yaw = int((((refPt[0]-windowCenterX)/windowCenterX)*45) + 45)
            pitch = int((((windowCenterY-refPt[1])/windowCenterY)*25) + 25)
            if abs(yaw-45) > yaw_tolerance:
                serial_write(str(yaw), arduino)
                # print(f"{yaw=}")
        time.sleep(0.25)

def exitfunc(arduino: serial.Serial):
    arduino.write(bytes("exiting", 'utf-8'))

if __name__ == "__main__":
    print("starting ...")
    arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
    # arduino.write(bytes("starting", 'utf-8'))

    # if arduino.readline() != "success":
    #     print("failed to connect to arduino")
    #     sys.exit()

    pq = Queue(maxsize=1)
    yq  = Queue(maxsize=1)
    cvThread = threading.Thread(target=cv_thread, args=(pq, yq, ))
    serialThread = threading.Thread(target=serial_thread, args=(pq, yq, arduino))
    evoThread = threading.Thread(target=tof_thread, args=())

    cvThread.start()
    serialThread.start()
    evoThread.start()

    # atexit.register(exitfunc, arduino=arduino)