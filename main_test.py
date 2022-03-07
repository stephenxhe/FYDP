from cv2 import WINDOW_AUTOSIZE
from src.evo import Evo
import sys
import cv2
import numpy as np
import time
import serial
import serial.tools.list_ports
import threading
from threading import Condition, Lock
from src.pitch import Trajectory
from pynput import keyboard

# CONFIG FILES
CLASSES_FILE = "./model_data/coco/coco.names"
MODEL_CONFIGURATION = "./config/yolov3.cfg"
MODEL_WEIGHTS = "./model_data/yolov3.weights"

BAUD_RATE = 250000

data_lock = Lock()
distance = 0
theta = 500

BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

yaw_tolerance = 2  # [degrees]

valid_targets = ['stop sign', 'car', 'bus', 'truck']

viewport_width = 1280
viewport_height = 720

centreX = int(viewport_width/2)
centreY = int(viewport_height/2)

evoWarning = False
arduinoWarning = False

class VehicleFinder:
    CONFIDENCE_THRESHOLD = 0.5  # min confidence to draw a box
    NMS_THRESHOLD = 0.2  # lower num = more aggressive, fewer boxes

    def __init__(self):
        self.classNames = []
        self.set_class_names()
        self.refPt = [None, None]
        self.boundingBoxes = []
        self.classIdxs = []
        self.confidences = []

    def set_class_names(self):
        with open(CLASSES_FILE, "rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

    def isInBox(self, pointX, pointY, boundX, boundY, boundW, boundH):
        if pointX is None or pointY is None:
            return False
        if (
            pointX < boundX
            or pointY < boundY
            or pointX > boundX + boundW
            or pointY > boundY + boundH
        ):
            return False
        return True

    def mouseCallback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            self.refPt = [x, y]
        elif event == cv2.EVENT_RBUTTONUP:
            self.refPt = [None, None]
        elif event == cv2.EVENT_MBUTTONUP:
            self.fire(x, y)

    def mark_vehicle(self, indicesToKeep, img):
        for i in indicesToKeep:
            if self.classNames[self.classIdxs[i]] in valid_targets:
                box = self.boundingBoxes[i]
                x, y, w, h = box[0], box[1], box[2], box[3]

                if self.isInBox(self.refPt[0], self.refPt[1], x, y, w, h):
                    self.refPt = [int(x + 0.5 * w), int(y + 0.5 * h)]

                    if abs(self.refPt[0] - centreX) <= 75:
                        cv2.rectangle(img, (x, y), (x + w, y + h), GREEN, 2)
                        cv2.putText(
                            img,
                            "Target Vehicle",
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            GREEN,
                            2,
                        )
                    else:
                        cv2.rectangle(img, (x, y), (x + w, y + h), RED, 2)
                        cv2.putText(
                            img,
                            "Target Vehicle",
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            RED,
                            2,
                        )

                elif self.refPt[0] is None and self.refPt[1] is None:
                    cv2.rectangle(img, (x, y), (x + w, y + h), BLUE, 2)
                    cv2.putText(
                        img,
                        f"{self.classNames[self.classIdxs[i]]}",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        BLUE,
                        2,
                    )

    def findObjects(self, outputs, img):
        hT, wT, _ = img.shape
        self.boundingBoxes, self.classIdxs, self.confidences = [], [], []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classIdx = np.argmax(scores)
                confidence = scores[classIdx]

                if confidence > self.CONFIDENCE_THRESHOLD:
                    w, h = int(detection[2] * wT), int(
                        detection[3] * hT
                    )  # pixel values
                    x, y = int(detection[0] * wT - 0.5 * w), int(
                        detection[1] * hT - 0.5 * h
                    )  # top left corner of box

                    self.boundingBoxes.append([x, y, w, h])
                    self.classIdxs.append(classIdx)
                    self.confidences.append(float(confidence))
        indicesToKeep = cv2.dnn.NMSBoxes(
            self.boundingBoxes,
            self.confidences,
            self.CONFIDENCE_THRESHOLD,
            self.NMS_THRESHOLD,
        )  # removes overlapping boxes

        self.mark_vehicle(indicesToKeep, img)

    def _set_camera_config(self, cap, width, height):
        print("set camera config")
        # cap.set(cv2.CAP_PROP_FPS, 60)  # Set camera FPS to 60 FPS
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # Set the width of the camera image to 1280
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # Set the vertical width of the camera image to 720

    def drawTarget(self, img, point):
        x, y = point[0], point[1]

        if x is not None and y is not None:
            if abs(x - centreX) <= 75:
                # vertical line
                cv2.line(img, (x, 0), (x, viewport_height), GREEN, 2)
                # horizontal line
                cv2.line(img, (0, y), (viewport_width, y), GREEN, 2)

                with data_lock:
                    cv2.putText(img, f"Distance: {distance} m", (x, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2,)
            else:
                # vertical line
                cv2.line(img, (x, 0), (x, viewport_height), RED, 2)
                # horizontal line
                cv2.line(img, (0, y), (viewport_width, y), RED, 2)

                with data_lock:
                    cv2.putText(
                        img,
                        f"Distance: {distance} m",
                        (x, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        RED,
                        2,
                    )

    def cv_thread(self):
        cap = cv2.VideoCapture(0)  # camera selection
        whT = 320  # yolov3 image size

        net = cv2.dnn.readNetFromDarknet(MODEL_CONFIGURATION, MODEL_WEIGHTS)

        cv2.namedWindow("Webcam capture", flags=WINDOW_AUTOSIZE)
        cv2.setMouseCallback("Webcam capture", self.mouseCallback)  # capture mouse clicks for selecting objects
        self._set_camera_config(cap, viewport_width, viewport_height)

        # main loop
        global arduinoWarning, evoWarning
        while True:
            success, img = cap.read()

            if success:
                blob = cv2.dnn.blobFromImage(img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)  # read docs for params
                net.setInput(blob)

                layerNames = net.getLayerNames()
                outputNames = [layerNames[i - 1] for i in net.getUnconnectedOutLayers()]

                outputs = net.forward(outputNames)

                self.findObjects(outputs, img)
                self.drawTarget(img, self.refPt)

                # # center cross
                cv2.line(img, (centreX + 10, centreY), (centreX - 10, centreY), BLUE, 2)
                cv2.line(img, (centreX, centreY + 10), (centreX, centreY - 10), BLUE, 2)

                if arduinoWarning:
                    cv2.putText(img, "WARNING: No arduino connected", (10,20), 
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        RED,
                        2,)
                if evoWarning:
                    cv2.putText(img, "WARNING: No ToF sensor connected", (10,50), 
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        RED,
                        2,)

                cv2.imshow("Webcam capture", img)
                cv2.waitKey(1)

    # serial helper functions
    def serial_write(self, data: str, device: serial.Serial):
        device.write(bytes(data, "utf-8"))

    def serial_read(self, device: serial.Serial):
        res = device.readline()
        return res

    # serial
    def serial_thread(self):
        comport = "COM5"
        try:
            arduino = serial.Serial(port=comport, baudrate=BAUD_RATE, timeout=0.1)
        except serial.serialutil.SerialException:
            print(f"Couldn't find arduino on {comport}")
            global arduinoWarning
            arduinoWarning = True
            sys.exit()

        curYaw = 90
        curPitch = 90

        serialLock = Condition()

        def on_press(key):
            global theta
            try:
                k = key.char  # single-char keys
            except:
                k = key.name  # other keys
            if k == 'space':
                if self.refPt != [None, None]:
                    print('fire')
                    # traj = Trajectory(distance*np.cos(curPitch-90), distance*np.sin(curPitch-90))
                    fire = 1
                    # theta = traj.solve()
                    # pitch = int(theta+90)

                    # these values shouldn't matter
                    yaw = abs(90-int(((self.refPt[0]/centreX)*90)/2)+45)
                    pitch = abs(90-(int((((self.refPt[1]/centreY)*50))/2))+25)


                    print(f"{pitch=} {yaw=}")

                    pitch = pitch << 8
                    fire = (fire << 15) | pitch 
                    data = yaw | fire
    
                    serialLock.acquire()
                    self.serial_write(str(data) + "\n", arduino)
                    serialLock.release()

        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        # main loop
        while True:
            if self.refPt != [None, None]:
                # pitch and yaw calcualted using fixed camera
                yaw = abs(90-int(((self.refPt[0]/centreX)*90)/2)+45)
                pitch = abs(90-(int((((self.refPt[1]/centreY)*50))/2))+25)
                fire = 0

                if abs(curYaw - yaw) > 1 or abs(curPitch - pitch) > 1:
                    curYaw = yaw
                    curPitch = pitch

                    pitch = pitch << 8
                    fire = (fire << 15) | pitch 
                    data = yaw | fire

                    serialLock.acquire()
                    self.serial_write(str(data) + "\n", arduino)
                    serialLock.release()
                   
            time.sleep(0.1)

# rangefinder
def tof_thread():
    # main thread code
    evo_obj = Evo()

    print("Starting Evo data streaming")
    # Get the port the evo has been connected to
    port = evo_obj.findEvo()

    if port is None:
        print("ERROR: Couldn't find the Evo. Exiting.")
        global evoWarning
        evoWarning = True
        sys.exit()
    else:
        evo = evo_obj.openEvo(port)

    # main loop
    while True:
        try:
            global distance
            dist = evo_obj.get_evo_range(evo)
            with data_lock:
                distance = dist
            # print(f"{dist} m")
        except serial.serialutil.SerialException:
            print("ERROR: Device disconnected (or multiple access on port). Exiting...")
            sys.exit()


if __name__ == "__main__":
    # Testing
    print("-----------STARTING---------------")

    vehicle_finder = VehicleFinder()
    cvThread = threading.Thread(target=vehicle_finder.cv_thread)
    serialThread = threading.Thread(target=vehicle_finder.serial_thread, args=())
    evoThread = threading.Thread(target=tof_thread, args=())

    cvThread.start()
    serialThread.start()
    evoThread.start()
