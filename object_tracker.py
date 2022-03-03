#================================================================
#
#   File name   : object_tracker.py
#   Author      : PyLessons
#   Created date: 2020-09-17
#   Website     : https://pylessons.com/
#   GitHub      : https://github.com/pythonlessons/TensorFlow-2.x-YOLOv3
#   Description : code to track detected object from video or webcam
#
#================================================================
from threading import Lock
import os
from turtle import width
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import cv2
from cv2 import WINDOW_AUTOSIZE
import numpy as np
import tensorflow as tf
from yolov3.utils import Load_Yolo_model, image_preprocess, postprocess_boxes, nms, draw_bbox, read_class_names
from yolov3.configs import *
import time
import serial
import sys
import threading
from pynput import keyboard

from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker
from deep_sort import generate_detections as gdet

from src.evo import Evo

video_path   = ""

BLUE = (255, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)

BAUD_RATE = 250000

class VehicleFinder:
    def __init__(self):
        if video_path:
            self.vid = cv2.VideoCapture(video_path) # detect on video
        else:
            self.vid = cv2.VideoCapture(0) # detect from webcam
            # self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # Set the width of the camera image to 1280
            # self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # Set the vertical width of the camera image to 720
        self.width = int(self.vid.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # shared values
        self.refPtLock = Lock()
        self.distance = -float("inf")
        self.refPt = [None, None]
        self.fire = False
    
    def Arduino_thread(self):
        arduino = serial.Serial(port="COM4", baudrate=BAUD_RATE, timeout=0.1)
        arduino.flushInput()
        arduino.flushOutput()
        pitch_tolerance = 2
        yaw_tolerance = 2

        def on_press(key):
            try:
                k = key.char  # single-char keys
            except:
                k = key.name  # other keys
            if k == 'space':
                if self.refPt != [None, None]:
                # yaw = [0:90]
                    with self.refPtLock:
                        currX, currY = self.refPt[0], self.refPt[1]
                    
                    # pitch and yaw are inaccurate bc of how we're calculating it 
                    yaw = int((currX/self.width)*90)
                    if abs(yaw - 45) < yaw_tolerance:
                        yaw = 45
                    
                    # pitch = [0:50]
                    pitch = int((currY/self.height)*50)
                    if abs(pitch - 25) < pitch_tolerance:
                        pitch = 25

                    print(f"{currY=} {pitch=} {currX=} {yaw=}")
                    pitch = pitch << 8
                    data = pitch | yaw

                    if yaw == 45:
                        time.sleep(0.01)
                    else:
                        arduino.write(bytes(str(data), 'utf-8'))
                        with self.refPtLock:
                            self.refPt = [int(self.width/2), int(self.height/2)]
                    
                    # while arduino.readline().decode('UTF-8') == "":
                    #     time.sleep(0.1)
                    # print(arduino.readline().decode('UTF-8'))
                # time.sleep(0.5)

        listener = keyboard.Listener(on_press=on_press)
        listener.start()  # start to listen on a separate thread
        # listener.join()  # remove if main thread is polling self.keysx``

        time.sleep(5)
        print('--- arduino ready ---')
        # while True:
        #     if self.refPt != [None, None]:
        #         # yaw = [0:90]
        #         with self.refPtLock:
        #             currX, currY = self.refPt[0], self.refPt[1]
        #         yaw = int((currX/self.width)*90)
        #         if abs(yaw - 45) < yaw_tolerance:
        #             yaw = 45
                
        #         # pitch = [0:50]
        #         pitch = int((currY/self.height)*50)
        #         if abs(pitch - 25) < pitch_tolerance:
        #             pitch = 25

        #         print(f"{currY=} {pitch=} {currX=} {yaw=}")
        #         pitch = pitch << 8
        #         data = pitch | yaw

        #         if yaw == 45:
        #             time.sleep(0.1)
        #             continue
        #         else:
        #             arduino.write(bytes(str(data), 'utf-8'))
                
        #         while arduino.readline().decode('UTF-8') == "":
        #             time.sleep(0.1)
        #         print(arduino.readline().decode('UTF-8'))
        #     time.sleep(0.5)
    
    def Tof_thread(self):
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
                dist = evo_obj.get_evo_range(evo)
                self.distance = dist
            except serial.serialutil.SerialException:
                print("ERROR: Device disconnected (or multiple access on port). Exiting...")
                sys.exit()
        
    def Object_tracking(self, Yolo, video_path, output_path, input_size=YOLO_INPUT_SIZE, show=True, CLASSES=YOLO_COCO_CLASSES, score_threshold=0.5, iou_threshold=0.3, rectangle_colors=RED, Track_only = ["person"]):
        # Definition of the parameters
        max_cosine_distance = 0.5
        nn_budget = None
        
        #initialize deep sort object
        model_filename = 'model_data/mars-small128.pb'
        encoder = gdet.create_box_encoder(model_filename, batch_size=1)
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
        tracker = Tracker(metric)

        times, times_2 = [], []

        # by default VideoCapture returns float instead of int
        fps = int(self.vid.get(cv2.CAP_PROP_FPS))
        codec = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_path, codec, fps, (self.width, self.height)) # output_path must be .mp4

        NUM_CLASS = read_class_names(CLASSES)
        key_list = list(NUM_CLASS.keys()) 
        val_list = list(NUM_CLASS.values())

        def mouseCallback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONUP:
                with self.refPtLock:
                    self.refPt = [x, y]
            elif event == cv2.EVENT_RBUTTONUP:
                with self.refPtLock:
                    self.refPt = [None, None]
                print('--- clear selection ---')

        cv2.namedWindow("Output", flags=WINDOW_AUTOSIZE)
        cv2.setMouseCallback("Output", mouseCallback)

        while True:
            _, frame = self.vid.read()

            try:
                original_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                original_frame = cv2.cvtColor(original_frame, cv2.COLOR_BGR2RGB)
            except:
                break
            
            image_data = image_preprocess(np.copy(original_frame), [input_size, input_size])
            #image_data = tf.expand_dims(image_data, 0)
            image_data = image_data[np.newaxis, ...].astype(np.float32)

            t1 = time.time()
            if YOLO_FRAMEWORK == "tf":
                pred_bbox = Yolo.predict(image_data)
            elif YOLO_FRAMEWORK == "trt":
                batched_input = tf.constant(image_data)
                result = Yolo(batched_input)
                pred_bbox = []
                for key, value in result.items():
                    value = value.numpy()
                    pred_bbox.append(value)
            
            #t1 = time.time()
            #pred_bbox = Yolo.predict(image_data)
            t2 = time.time()
            
            pred_bbox = [tf.reshape(x, (-1, tf.shape(x)[-1])) for x in pred_bbox]
            pred_bbox = tf.concat(pred_bbox, axis=0)

            bboxes = postprocess_boxes(pred_bbox, original_frame, input_size, score_threshold)
            bboxes = nms(bboxes, iou_threshold, method='nms')

            # extract bboxes to boxes (x, y, width, height), scores and names
            boxes, scores, names = [], [], []
            for bbox in bboxes:
                if len(Track_only) !=0 and NUM_CLASS[int(bbox[5])] in Track_only or len(Track_only) == 0:
                    boxes.append([bbox[0].astype(int), bbox[1].astype(int), bbox[2].astype(int)-bbox[0].astype(int), bbox[3].astype(int)-bbox[1].astype(int)])
                    scores.append(bbox[4])
                    names.append(NUM_CLASS[int(bbox[5])])

            # Obtain all the detections for the given frame.
            boxes = np.array(boxes) 
            names = np.array(names)
            scores = np.array(scores)
            features = np.array(encoder(original_frame, boxes))
            detections = [Detection(bbox, score, class_name, feature) for bbox, score, class_name, feature in zip(boxes, scores, names, features)]

            # Pass detections to the deepsort object and obtain the track information.
            tracker.predict()
            tracker.update(detections)

            # Obtain info from the tracks
            tracked_bboxes = []
            for track in tracker.tracks:
                if not track.is_confirmed() or track.time_since_update > 5:
                    continue 
                bbox = track.to_tlbr() # Get the corrected/predicted bounding box
                class_name = track.get_class() #Get the class name of particular object
                tracking_id = track.track_id # Get the ID for the particular track
                index = key_list[val_list.index(class_name)] # Get predicted object index by object name
                tracked_bboxes.append(bbox.tolist() + [tracking_id, index]) # Structure data, that we could use it with our draw_bbox function

            # draw detection on frame
            with self.refPtLock:
                image, self.refPt = draw_bbox(original_frame, tracked_bboxes, self.refPt, CLASSES=CLASSES, tracking=True)

            if self.refPt != [None, None]:
                # print(f"draw crosshair at {self.refPt}")
                # vertical line
                cv2.line(image, (self.refPt[0], 0), (self.refPt[0], self.height), GREEN, 2)
                # horizontal line
                cv2.line(image, (0, self.refPt[1]), (self.width, self.refPt[1]), GREEN, 2)
            
            # center cross
            cv2.line(image, (int(0.5*self.width) + 10, int(0.5*self.height)), (int(0.5*self.width) - 10, int(0.5*self.height)), BLUE, 2,)
            cv2.line(image, (int(0.5*self.width), int(0.5*self.height) + 10), (int(0.5*self.width), int(0.5*self.height) - 10), BLUE, 2, )

            t3 = time.time()
            times.append(t2-t1)
            times_2.append(t3-t1)
            
            times = times[-20:]
            times_2 = times_2[-20:]

            ms = sum(times)/len(times)*1000
            fps = 1000 / ms
            fps2 = 1000 / (sum(times_2)/len(times_2)*1000)
            
            image = cv2.putText(image, "Time: {:.1f} FPS".format(fps), (0, 30), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)

            # draw original yolo detection
            #image = draw_bbox(image, bboxes, CLASSES=CLASSES, show_label=False, rectangle_colors=rectangle_colors, tracking=True)

            # print("Time: {:.2f}ms, Detection FPS: {:.1f}, total FPS: {:.1f}".format(ms, fps, fps2))
            if output_path != '': out.write(image)
            if show:
                cv2.imshow('Output', image)
                if cv2.waitKey(25) & 0xFF == ord("q"):
                    cv2.destroyAllWindows()
                    break
                
        cv2.destroyAllWindows()

if __name__ == "__main__":
    yolo = Load_Yolo_model()
    tracker = VehicleFinder()

    cvThread = threading.Thread(target=tracker.Object_tracking, args=(yolo, video_path, ""), )
    arduinoThread = threading.Thread(target=tracker.Arduino_thread, )
    tofThread = threading.Thread(target=tracker.Tof_thread, )

    cvThread.start()
    arduinoThread.start()
    # cvThread.start()
    # tracker.Object_tracking(yolo, video_path, "", input_size=YOLO_INPUT_SIZE, show=True, iou_threshold=0.1, rectangle_colors=RED, Track_only = ["person"])
