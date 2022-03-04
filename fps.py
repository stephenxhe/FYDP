# import the necessary packages
from threading import Thread, Condition
import cv2
from cv2 import FONT_HERSHEY_COMPLEX as hershey, WINDOW_AUTOSIZE
import numpy as np
import serial
import time

# from deep_sort import nn_matching
# from deep_sort.detection import Detection
# from deep_sort.tracker import Tracker
# from deep_sort import generate_detections as gdet
# from yolov3.utils import Load_Yolo_model, image_preprocess, postprocess_boxes, nms, draw_bbox, read_class_names

frameLock = Condition()
frame = None

bboxLock = Condition()
boundingBoxes = []
classIdxs = []
confidences = []
indicesToKeep = []

refPtLock = Condition()
refPt = [None, None]

BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BAUD_RATE = 250000

cv2.namedWindow("Webcam capture", flags=WINDOW_AUTOSIZE)
width = int(cv2.getWindowImageRect("Webcam capture")[2])
height = int(cv2.getWindowImageRect("Webcam capture")[3])
centreX = int(width/2)
centreY = int(height/2)

class WebcamVideoStream:
	def __init__(self, classes, src=1, validTargets = ['person']):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.stream.read()
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
		with open(classes, "rt") as f:
			self.classNames = f.read().rstrip("\n").split("\n")
		self.validTargets = validTargets
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		global frame

		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()

			frameLock.acquire()
			frame = self.frame
			frameLock.release()
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

	def read(self):
		global boundingBoxes, classIdxs, confidences, indicesToKeep, refPt

		bboxLock.acquire()
		_boundingBoxes = boundingBoxes
		_classIdxs = classIdxs
		_confidences = confidences
		_indicesToKeep = indicesToKeep
		bboxLock.release()

		img = self.frame

		for i in _indicesToKeep:
			if self.classNames[_classIdxs[i]] in self.validTargets:
				box = _boundingBoxes[i]
				x, y, w, h = box[0], box[1], box[2], box[3]

				refPtLock.acquire()
				if self.isInBox(refPt[0], refPt[1], x, y, w, h):
					refPt = [int(x + 0.5 * w), int(y + 0.5 * h)]
				refPtLock.release()

				cv2.rectangle(img, (x, y), (x + w, y + h), BLUE, 2)
				cv2.putText(img, f"{self.classNames[_classIdxs[i]]}", (x, y - 10), hershey, 0.6, BLUE, 2,)
		
		# return the frame most recently read with gui drawn on
		return img
	
	def mouseCallback(self, event, x, y, flags, param):
		global refPt
		if event == cv2.EVENT_LBUTTONUP:
			refPtLock.acquire()
			refPt = [x, y]
			refPtLock.release()
		elif event == cv2.EVENT_RBUTTONUP:
			refPtLock.acquire()
			refPt = [None, None]
			refPtLock.release()

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
	
class YOLO:
	def __init__(self, config, weights, confidence, threshold):
		
		self.config = config
		self.weights = weights
		self.confidence = confidence
		self.threshold = threshold
	
	def object_detection(self):
		global frame, boundingBoxes, classIdxs, confidences, indicesToKeep
		net = cv2.dnn.readNetFromDarknet(self.config, self.weights)

		while True:
			frameLock.acquire()
			image = frame
			frameLock.release()

			blob = cv2.dnn.blobFromImage(image, 1 / 255, (320, 320), [0, 0, 0], 1, crop=False)
			net.setInput(blob)
			layerNames = net.getLayerNames()
			outputNames = [layerNames[i - 1] for i in net.getUnconnectedOutLayers()]
			outputs = net.forward(outputNames)
			hT, wT, _ = image.shape
			_boundingBoxes, _classIdxs, _confidences = [], [], []

			for output in outputs:
				for detection in output:
					scores = detection[5:]
					classIdx = np.argmax(scores)
					confidence = scores[classIdx]

					if confidence > self.confidence:
						w, h = int(detection[2] * wT), int(detection[3] * hT)  # pixel values
						x, y = int(detection[0] * wT - 0.5 * w), int(detection[1] * hT - 0.5 * h)  # top left corner of box

						_boundingBoxes.append([x, y, w, h])
						_classIdxs.append(classIdx)
						_confidences.append(float(confidence))
			_indicesToKeep = cv2.dnn.NMSBoxes(_boundingBoxes, _confidences, self.confidence, self.threshold,)

			bboxLock.acquire()
			boundingBoxes = _boundingBoxes
			classIdxs = _classIdxs
			confidences = _confidences
			indicesToKeep = _indicesToKeep
			bboxLock.release()
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.object_detection, args=()).start()
		return self

class SerialThread:
	def __init__(self) -> None:
		pass

	def update(self):
		global width, height
		arduino = serial.Serial(port="COM4", baudrate=BAUD_RATE, timeout=0.1)
		while True:
			refPtLock.acquire()
			if refPt != [None, None]:
				yaw = int((refPt[0]/width)*90)
				pitch = int(abs((refPt[1]/height)*50-50))
				refPtLock.release()

				if abs(yaw - 45) < 5:
					yaw = 45
				if abs(pitch - 25) < 10:
					pitch = 25
				
				pitch = pitch << 8
				data = pitch | yaw

				if yaw == 45:
					time.sleep(0.1)
					continue
				else:
					arduino.write(bytes(str(data), 'utf-8'))
				
				while arduino.readline().decode('UTF-8') == "":
					time.sleep(0.1)
				print(arduino.readline().decode('UTF-8'))
			else:
				refPtLock.release()

			time.sleep(0.5)


	def start(self):
		Thread(target=self.update, args=()).start()
		return self


CLASSES_FILE = "./config/object.names"
MODEL_CONFIGURATION = "./config/yolov3.cfg"
MODEL_WEIGHTS = "./config/yolov3.weights"
CONFIDENCE_THRESHOLD = 0.5  # min confidence to draw a box
NMS_THRESHOLD = 0.2  # lower num = more aggressive, fewer boxes

vs = WebcamVideoStream(
	classes=CLASSES_FILE, 
	src=1,
	validTargets=['person']
).start()

cv2.setMouseCallback("Webcam capture", vs.mouseCallback)  # capture mouse clicks for selecting objects

frame = vs.read()

yolo = YOLO(
	config=MODEL_CONFIGURATION, 
	weights=MODEL_WEIGHTS,
	confidence=CONFIDENCE_THRESHOLD,
	threshold=NMS_THRESHOLD
).start()

# loop over some frames...this time using the threaded stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 400 pixels
	image = vs.read()

	# center cross
	cv2.line(image, (centreX + 10, centreY), (centreX - 10, centreY), BLUE, 2,)
	cv2.line(image, (centreX, centreY + 10), (centreX, centreY - 10), BLUE, 2,)

	refPtLock.acquire()
	if refPt != [None, None]:
		cv2.line(image, (refPt[0], 0), (refPt[0], 2 * centreY), GREEN, 2) # vertical line
		cv2.line(image, (0, refPt[1]), (2 * centreX, refPt[1]), GREEN, 2) # horizontal line
	refPtLock.release()

	# show image
	cv2.imshow("Webcam capture", image)
	key = cv2.waitKey(1) & 0xFF