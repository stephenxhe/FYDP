import cv2
import mediapipe as mp
import time
import sys
import serial
import serial.tools.list_ports
from .evo import Evo

BLUE = (255, 0, 0)
RED = (0, 0, 255)
LINEWIDTH = 10

mpDetection = mp.solutions.mediapipe.python.solutions.face_detection
mpDrawing = mp.solutions.mediapipe.python.solutions.drawing_utils

cap = cv2.VideoCapture(0)  # camera selection
cap.set(cv2.CAP_PROP_FPS, 60)  # Set camera FPS to 60 FPS
cv2.namedWindow("Webcam capture")
windowCenterX = int(cv2.getWindowImageRect("Webcam capture")[2] / 2)
windowCenterY = int(cv2.getWindowImageRect("Webcam capture")[3] / 2)


pTime = 0
if __name__ == "__main__":

    evo_obj = Evo()
    print("Starting Evo data streaming")
    # Get the port the evo has been connected to
    port = evo_obj.findEvo()

    if port == "NULL":
        print("Sorry couldn't find the Evo. Exiting.")
        sys.exit()
    else:
        evo = evo_obj.openEvo(port)

    with mpDetection.FaceDetection(
        model_selection=0, min_detection_confidence=0.5
    ) as face_detection:
        while cap.isOpened():
            # get image
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            # To improve performance, optionally mark the image as not writeable to pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = face_detection.process(image)

            # Draw the face detection annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.detections:
                for detection in results.detections:
                    mpDrawing.draw_detection(image, detection)

            # draw center cross
            cv2.line(
                image,
                (windowCenterX + LINEWIDTH, windowCenterY),
                (windowCenterX - LINEWIDTH, windowCenterY),
                BLUE,
                2,
            )
            cv2.line(
                image,
                (windowCenterX, windowCenterY + LINEWIDTH),
                (windowCenterX, windowCenterY - LINEWIDTH),
                BLUE,
                2,
            )

            # draw fps
            cTime = time.time()
            fps = 1 / (cTime - pTime)
            pTime = cTime
            cv2.putText(
                image,
                f"FPS: {int(fps)}",
                (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                RED,
                2,
            )

            # get ToF reading
            try:
                dist = evo_obj.get_evo_range(evo)
                print(dist)
                # if type(dist) == float:
                # cv2.putText(image, f'Distance: {float(dist)}', (windowCenterX,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)
            except serial.serialutil.SerialException:
                print("Device disconnected (or multiple access on port). Exiting...")
                break

            cv2.imshow("MediaPipe Face Detection", image)

            if cv2.waitKey(1) & 0xFF == 27:
                break

        evo.close()
        cap.release()
        sys.exit()

# # For webcam input:
with mpDetection.FaceDetection(
    model_selection=0, min_detection_confidence=0.5
) as face_detection:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image)

        # Draw the face detection annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.detections:
            for detection in results.detections:
                mpDrawing.draw_detection(image, detection)
        # Flip the image horizontally for a selfie-view display.
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        # center cross
        cv2.line(
            image,
            (windowCenterX + LINEWIDTH, windowCenterY),
            (windowCenterX - LINEWIDTH, windowCenterY),
            RED,
            2,
        )
        cv2.line(
            image,
            (windowCenterX, windowCenterY + LINEWIDTH),
            (windowCenterX, windowCenterY - LINEWIDTH),
            RED,
            2,
        )
        cv2.putText(
            image, f"FPS: {int(fps)}", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2
        )
        cv2.imshow("MediaPipe Face Detection", image)

        if cv2.waitKey(1) & 0xFF == 27:
            break
cap.release()
