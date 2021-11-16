import cv2 as cv
import numpy as np

RED = (255, 0, 0)

cap = cv.VideoCapture(1)    # camera selection
whT = 320       # yolov3 image size

classesFile = 'coco.names'
classNames = []
with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

modelConfiguration = 'yolov3.cfg'
modelWeights = 'yolov3.weights'

net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

confidenceThresh = 0.2  # min confidence to draw a box
nmsThreshold = 0.5      # lower num = more aggressive, fewer boxes

def findObjects(outputs,img):
    hT, wT, cT = img.shape
    boundingBox = []
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

                boundingBox.append([x,y,w,h])
                classIdxs.append(classIdx)
                confidences.append(float(confidence))
    indicesToKeep = cv.dnn.NMSBoxes(boundingBox, confidences, confidenceThresh, nmsThreshold)       # removes overlapping boxes
    print(indicesToKeep)
    for i in indicesToKeep:
        box = boundingBox[i]
        x,y,w,h = box[0], box[1], box[2], box[3]

        cv.rectangle(img, (x,y), (x+w, y+h), RED, 2)
        cv.putText(
            img, 
            f'{classNames[classIdxs[i]]} {int(confidences[i]*100)}%',
            (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2
        )


while True:
    success, img = cap.read()

    blob = cv.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)      # read docs for params
    net.setInput(blob)

    layerNames = net.getLayerNames()
    outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
    
    outputs = net.forward(outputNames)

    findObjects(outputs, img)

    cv.imshow('Webcam Capture', img)
    cv.waitKey(1)