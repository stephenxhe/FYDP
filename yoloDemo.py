import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import serial

BLUE = (255, 0, 0)
RED = (0,0,255)

arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)

cap = cv.VideoCapture(0)    # camera selection
cap.set(cv.CAP_PROP_FPS, 60)           #Set camera FPS to 60 FPS
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) #Set the width of the camera image to 1280
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720) #Set the vertical width of the camera image to 720
whT = 320                   # yolov3 image size

classesFile = 'coco.names'
classNames = []
with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

modelConfiguration = 'yolov3-tiny.cfg'
modelWeights = 'yolov3-tiny.weights'

net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

confidenceThresh = 0.4  # min confidence to draw a box
nmsThreshold = 0.2      # lower num = more aggressive, fewer boxes

refPt = [None,None]     # this is what we will send to the aiming / LIDAR

def isInBox(pointX, pointY, boundX, boundY, boundW, boundH):
    if pointX is None or pointY is None:
        return False
    if pointX < boundX or pointY < boundY or pointX > boundX + boundW or pointY > boundY + boundH:
        return False
    return True

def findObjects(outputs,img):
    global refPt
    hT, wT, cT = img.shape
    boundingBoxes = []
    classIdxs = []
    confidences = []

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
        if classNames[classIdxs[i]] == 'car' or classNames[classIdxs[i]] == 'bus' or classNames[classIdxs[i]] == 'truck' or True:
        # if classNames[classIdxs[i]] == 'cup':
            box = boundingBoxes[i]
            x,y,w,h = box[0], box[1], box[2], box[3]

            if isInBox(refPt[0], refPt[1], x,y,w,h):
                cv.rectangle(img, (x,y), (x+w, y+h), RED, 2)
                cv.putText(
                    img, 
                    'Target Vehicle',
                    (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2
                )
                refPt = [int(x+0.5*w), int(y+0.5*h)]
            elif refPt[0] is None and refPt[1] is None:
            # else:
                cv.rectangle(img, (x,y), (x+w, y+h), BLUE, 2)
                cv.putText(
                    img, 
                    f'{classNames[classIdxs[i]]} {int(confidences[i]*100)}%',
                    (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2
                )

def mouseCallback(event, x, y, flags, param):
    global refPt
    if event == cv.EVENT_LBUTTONUP:
        refPt = [x,y]
    elif event == cv.EVENT_RBUTTONUP:
        refPt = [None, None]

cv.namedWindow("Webcam capture")                            # create a window
cv.setMouseCallback("Webcam capture", mouseCallback)    # capture mouse clicks for selecting objects

windowCenterX = int(cv.getWindowImageRect("Webcam capture")[2]/2)
windowCenterY = int(cv.getWindowImageRect("Webcam capture")[3]/2)
lineWidth = 10

# helper func
def drawPoint(img, point):
    if point[0] is not None and point[1] is not None:
        cv.line(img, (point[0], point[1]+lineWidth), (point[0], point[1]-lineWidth), RED, 2)
        cv.line(img, (point[0]+lineWidth, point[1]), (point[0]-lineWidth, point[1]), RED, 2)

        cv.line(img, (windowCenterX, windowCenterY), (point[0], point[1]), RED, 2)

        targetx, targety = windowCenterX-point[0], windowCenterY-point[1]

        pitch = ((windowCenterY-point[1])/windowCenterY)*(51/2) + (51/2)
        yaw = ((point[0]-windowCenterX)/windowCenterX)*(90/2) + (90/2)

        print(pitch, yaw)

        arduino.write(bytes(str(pitch), 'utf-8'))
        # time.sleep(0.5)
        print(arduino.readline())

        cv.putText(img, f'x{targetx} y{targety}', (windowCenterX+10,windowCenterY-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

pTime = 0
while True:
    success, img = cap.read()
    
    if success:
        blob = cv.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)      # read docs for params
        net.setInput(blob)

        layerNames = net.getLayerNames()
        outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
        
        outputs = net.forward(outputNames)

        # findObjects(outputs, img)
        drawPoint(img, refPt)

        # center cross
        cv.line(img, (windowCenterX+lineWidth, windowCenterY), (windowCenterX-lineWidth, windowCenterY), BLUE, 2)
        cv.line(img, (windowCenterX, windowCenterY+lineWidth), (windowCenterX, windowCenterY-lineWidth), BLUE, 2)
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv.putText(img, f'FPS: {int(fps)}', (20,20), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)
        cv.imshow("Webcam capture", img)
        cv.waitKey(1)