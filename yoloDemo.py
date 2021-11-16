import cv2 as cv
import numpy as np

# count = cv.cuda.getCudaEnabledDeviceCount()
# print(count)

BLUE = (255, 0, 0)
RED = (0,0,255)

cap = cv.VideoCapture(0)    # camera selection
whT = 320                   # yolov3 image size

classesFile = 'coco.names'
classNames = []
with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

modelConfiguration = 'yolov4-leaky.cfg'
modelWeights = 'yolov4-leaky.weights'

net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

confidenceThresh = 0.5  # min confidence to draw a box
nmsThreshold = 0.1      # lower num = more aggressive, fewer boxes

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
        if classNames[classIdxs[i]] == 'car' or classNames[classIdxs[i]] == 'bus' or classNames[classIdxs[i]] == 'truck':
            box = boundingBoxes[i]
            x,y,w,h = box[0], box[1], box[2], box[3]

            if isInBox(refPt[0], refPt[1], x,y,w,h):
                cv.rectangle(img, (x,y), (x+w, y+h), RED, 2)
                refPt = [int(x+0.5*w), int(y+0.5*h)]
            else:
                cv.rectangle(img, (x,y), (x+w, y+h), BLUE, 2)
            cv.putText(
                img, 
                f'{classNames[classIdxs[i]]} {int(confidences[i]*100)}%',
                (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2
            )

def captureMouseClick(event, x, y, flags, param):
    global refPt
    if event == cv.EVENT_LBUTTONUP:
        refPt = [x,y]
    elif event == cv.EVENT_RBUTTONUP:
        refPt = [None, None]

cv.namedWindow("Webcam capture")                            # create a window
cv.setMouseCallback("Webcam capture", captureMouseClick)    # capture mouse clicks for selecting objects

windowWidth = cv.getWindowImageRect("Webcam capture")[2]
windowHeight = cv.getWindowImageRect("Webcam capture")[3]

# helper func
def drawPoint(img, point):
    if point[0] is not None and point[1] is not None:
        cv.line(img, (point[0], point[1]+5), (point[0], point[1]-5), RED, 1)
        cv.line(img, (point[0]+5, point[1]), (point[0]-5, point[1]), RED, 1)

        cv.line(img, (int(windowWidth/2), int(windowHeight/2)), (point[0], point[1]), RED, 1)

        cv.line(img, (int(windowWidth/2)+5, int(windowHeight/2)), (int(windowWidth/2)-5, int(windowHeight/2)), RED, 1)
        cv.line(img, (int(windowWidth/2), int(windowHeight/2)+5), (int(windowWidth/2), int(windowHeight/2)-5), RED, 1)

while True:
    success, img = cap.read()
    
    if success:
        blob = cv.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)      # read docs for params
        net.setInput(blob)

        layerNames = net.getLayerNames()
        outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
        
        outputs = net.forward(outputNames)

        findObjects(outputs, img)
        drawPoint(img, refPt)
        
        cv.imshow("Webcam capture", img)
        cv.waitKey(1)