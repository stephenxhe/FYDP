from cv2 import WINDOW_AUTOSIZE
from src.evo import Evo
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

# CONFIG FILES
CLASSES_FILE = "./config/object.names"
MODEL_CONFIGURATION = "./config/yolov4-leaky.cfg"
MODEL_WEIGHTS = "./config/yolov4-leaky.weights"

BAUD_RATE = 250000

data_lock = Lock()
distance = 0

BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

yaw_tolerance = 2  # [degrees]

valid_targets = ['bottle', 'backpack', 'stop sign', 'car', 'bus', 'truck']

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
        self.pq = Queue(maxsize=1)
        self.yq = Queue(maxsize=1)

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
        if event == cv.EVENT_LBUTTONUP:
            self.refPt = [x, y]
        elif event == cv.EVENT_RBUTTONUP:
            self.refPt = [None, None]

    def mark_vehicle(self, indicesToKeep, img):
        for i in indicesToKeep:
            if self.classNames[self.classIdxs[i]] in valid_targets:
                box = self.boundingBoxes[i]
                x, y, w, h = box[0], box[1], box[2], box[3]

                if self.isInBox(self.refPt[0], self.refPt[1], x, y, w, h):
                    self.refPt = [int(x + 0.5 * w), int(y + 0.5 * h)]
                    # print(f"{self.refPt[0]} {self.WINDOW_CENTER_X}")

                    if abs(self.refPt[0] - self.WINDOW_CENTER_X) <= 75:
                        cv.rectangle(img, (x, y), (x + w, y + h), GREEN, 2)
                        cv.putText(
                            img,
                            "Target Vehicle",
                            (x, y - 10),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            GREEN,
                            2,
                        )
                    else:
                        cv.rectangle(img, (x, y), (x + w, y + h), RED, 2)
                        cv.putText(
                            img,
                            "Target Vehicle",
                            (x, y - 10),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            RED,
                            2,
                        )

                elif self.refPt[0] is None and self.refPt[1] is None:
                    cv.rectangle(img, (x, y), (x + w, y + h), BLUE, 2)
                    cv.putText(
                        img,
                        f"{self.classNames[self.classIdxs[i]]} {int(self.confidences[i]*100)}%",
                        (x, y - 10),
                        cv.FONT_HERSHEY_SIMPLEX,
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
        indicesToKeep = cv.dnn.NMSBoxes(
            self.boundingBoxes,
            self.confidences,
            self.CONFIDENCE_THRESHOLD,
            self.NMS_THRESHOLD,
        )  # removes overlapping boxes

        self.mark_vehicle(indicesToKeep, img)

    def _set_camera_config(self, cap):
        print("set camera config")
        # cap.set(cv.CAP_PROP_FPS, 60)  # Set camera FPS to 60 FPS
        cap.set(
            cv.CAP_PROP_FRAME_WIDTH, 1280
        )  # Set the width of the camera image to 1280
        cap.set(
            cv.CAP_PROP_FRAME_HEIGHT, 720
        )  # Set the vertical width of the camera image to 720

    def cv_thread(self):
        cap = cv.VideoCapture(0)  # camera selection
        self._set_camera_config(cap)
        whT = 320  # yolov3 image size

        net = cv.dnn.readNetFromDarknet(MODEL_CONFIGURATION, MODEL_WEIGHTS)

        def drawTarget(img, point):
            x, y = point[0], point[1]

            if x is not None and y is not None:
                if abs(x - self.WINDOW_CENTER_X) <= 75:
                    # vertical line
                    cv.line(img, (x, 0), (x, 2 * self.WINDOW_CENTER_Y), GREEN, 2)
                    # horizontal line
                    cv.line(img, (0, y), (2 * self.WINDOW_CENTER_X, y), GREEN, 2)

                    with data_lock:
                        cv.putText(img, f"Distance: {distance} m", (x, y + 20), cv.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2,)
                else:
                    # vertical line
                    cv.line(img, (x, 0), (x, 2 * self.WINDOW_CENTER_Y), RED, 2)
                    # horizontal line
                    cv.line(img, (0, y), (2 * self.WINDOW_CENTER_X, y), RED, 2)

                    with data_lock:
                        cv.putText(
                            img,
                            f"Distance: {distance} m",
                            (x, y + 20),
                            cv.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            RED,
                            2,
                        )

        cv.namedWindow("Webcam capture", flags=WINDOW_AUTOSIZE)
        cv.setMouseCallback(
            "Webcam capture", self.mouseCallback
        )  # capture mouse clicks for selecting objects
        self.WINDOW_CENTER_X = int(cv.getWindowImageRect("Webcam capture")[2] / 2)
        self.WINDOW_CENTER_Y = int(cv.getWindowImageRect("Webcam capture")[3] / 2)

        width = int(cv.getWindowImageRect("Webcam capture")[2])
        height = int(cv.getWindowImageRect("Webcam capture")[3])

        print(f"{width=} {height=}")

        self.yq.put(self.WINDOW_CENTER_X)
        self.pq.put(self.WINDOW_CENTER_Y)

        # main loop
        pTime = 0
        while True:
            success, img = cap.read()

            if success:
                blob = cv.dnn.blobFromImage(
                    img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False
                )  # read docs for params
                net.setInput(blob)

                layerNames = net.getLayerNames()
                outputNames = [layerNames[i - 1] for i in net.getUnconnectedOutLayers()]

                outputs = net.forward(outputNames)

                self.findObjects(outputs, img)
                drawTarget(img, self.refPt)

                # show fps
                cTime = time.time()
                fps = 1 / (cTime - pTime)
                pTime = cTime

                # # center cross
                cv.line(
                    img,
                    (self.WINDOW_CENTER_X + 10, self.WINDOW_CENTER_Y),
                    (self.WINDOW_CENTER_X - 10, self.WINDOW_CENTER_Y),
                    BLUE,
                    2,
                )
                cv.line(
                    img,
                    (self.WINDOW_CENTER_X, self.WINDOW_CENTER_Y + 10),
                    (self.WINDOW_CENTER_X, self.WINDOW_CENTER_Y - 10),
                    BLUE,
                    2,
                )

                cv.imshow("Webcam capture", img)
                cv.waitKey(1)

    # serial helper functions
    def serial_write(self, data: str, device: serial.Serial):
        device.write(bytes(data, "utf-8"))

    def serial_read(self, device: serial.Serial):
        res = device.readline()
        return res

    # serial
    def serial_thread(self, pq, yq, arduino):
        windowCenterX = yq.get()
        windowCenterY = pq.get()

        # main loop
        while True:
            if self.refPt != [None, None]:
                # pitch and yaw are sent as OFFSETS from the current position
                
                yaw = int((((self.refPt[0] - windowCenterX) / windowCenterX) * 45) + 45)
                if abs(yaw - 45) < 4:
                    yaw = 45

                pitch = int(
                    (((windowCenterY - self.refPt[1]) / windowCenterY) * 25) + 25
                )
                if abs(pitch - 25) < 4:
                    pitch = 25

                pitch = pitch << 8
                data = pitch | yaw
                self.serial_write(str(data) + "\n", arduino)
                time.sleep(0.1)
                # _ = self.serial_read(arduino)

            time.sleep(0.25)

# rangefinder
def tof_thread():
    # main thread code
    evo_obj = Evo()

    print("Starting Evo data streaming")
    # Get the port the evo has been connected to
    port = evo_obj.findEvo()

    if port is None:
        print("ERROR: Couldn't find the Evo. Exiting.")
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
    arduino = serial.Serial(port="COM4", baudrate=BAUD_RATE, timeout=0.1)

    vehicle_finder = VehicleFinder()
    cvThread = threading.Thread(target=vehicle_finder.cv_thread)
    serialThread = threading.Thread(
        target=vehicle_finder.serial_thread,
        args=(vehicle_finder.pq, vehicle_finder.yq, arduino),
    )
    evoThread = threading.Thread(target=tof_thread, args=())

    cvThread.start()
    # serialThread.start()
    # evoThread.start()
