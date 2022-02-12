import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import serial
import threading
from threading import Lock
from queue import Queue

data_lock = Lock()
refPt = [None,None]
BLUE = (255, 0, 0)
RED = (0,0,255)

valid_targets = ['car', 'bus', 'person']

# cv
# this can be split into separate threads i think
def cv_thread(pq, yq):
    cap = cv.VideoCapture(0)                # camera selection
    cap.set(cv.CAP_PROP_FPS, 60)            # Set camera FPS to 60 FPS
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
                    cv.rectangle(img, (x,y), (x+w, y+h), RED, 2)
                    cv.putText(img, 'Target Vehicle', (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)
                    
                    # track target
                    refPt = [int(x+0.5*w), int(y+0.5*h)]
                elif refPt[0] is None and refPt[1] is None:
                    cv.rectangle(img, (x,y), (x+w, y+h), BLUE, 2)
                    cv.putText(img, f'{classNames[classIdxs[i]]} {int(confidences[i]*100)}%', (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)
                else:
                    refPt = [None, None]

    def drawTarget(img, point):
        x, y = point[0], point[1]
        
        if x is not None and y is not None:
            # vertical line
            cv.line(img, (x, 0), (x, 2*windowCenterY), RED, 2)
            # horizontal line
            cv.line(img, (0, y), (2*windowCenterX, y), RED, 2)
    
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
    print("tof thread")

# serial
def serial_thread(pq, yq):
    windowCenterX = yq.get()
    windowCenterY = pq.get()
    arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)

    while True:
        if refPt != [None, None]:
            yaw = int((((refPt[0]-windowCenterX)/windowCenterX)*45) + 45)
            pitch = int((((windowCenterY-refPt[1])/windowCenterY)*25) + 25)
            print(f"{yaw=} {pitch=}")
            #     print(f"{pitch=} {yaw=}")
            # print(f"serial got {refPt[0]}")
            arduino.write(bytes(str(yaw), 'utf-8'))
        # time.sleep(0.5)

if __name__ == "__main__":
    print("starting ...")
    pq = Queue(maxsize=1)
    yq  = Queue(maxsize=1)
    cvThread = threading.Thread(target=cv_thread, args=(pq, yq, ))
    serialThread = threading.Thread(target=serial_thread, args=(pq, yq,))

    cvThread.start()
    serialThread.start()