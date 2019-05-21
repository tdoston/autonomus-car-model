import cv2
import sys

def cascade_func(pgirl_cas, l_cas, park_cas, ped_cas, r_cas, stop_cas, skip):
    girl_pedestrian_cascade = "xml_signs/cascade_girl_pedestrian.xml"
    left_cascade = "xml_signs/cascade_left.xml"
    parking_cascade = "xml_signs/cascade_parking.xml"
    pedestrian_cascade = "xml_signs/pedestrian.xml"
    right_cascade = "xml_signs/cascade_right.xml"
    stop_cascade = "xml_signs/cascade_stop.xml"
    skip_cascade = "xml_signs/cascade_pedestrian2.xml"

    girlPedestrianCascade = cv2.CascadeClassifier(girl_pedestrian_cascade)
    leftCascade = cv2.CascadeClassifier(left_cascade)
    parkingCascade = cv2.CascadeClassifier(parking_cascade)
    pedestrianCascade = cv2.CascadeClassifier(pedestrian_cascade)
    rightCascade = cv2.CascadeClassifier(right_cascade)
    stopCascade = cv2.CascadeClassifier(stop_cascade)
    skipCascade = cv2.CascadeClassifier(skip_cascade)

    # video_capture = cv2.VideoCapture("xml_signs/runner_video.avi")
    # video_capture = cv2.VideoCapture("xml_signs/pedestrian_video.avi")
    # video_capture = cv2.VideoCapture("xml_signs/right_parking_glance_1.avi")
    # video_capture = cv2.VideoCapture("xml_signs/stop_video.avi")
    # video_capture = cv2.VideoCapture("xml_signs/stop_video_2.avi")
    video_capture = cv2.VideoCapture(0)

    pgirl_cas = 0
    l_cas = 0
    park_cas = 0
    ped_cas = 0
    r_cas = 0
    stop_cas = 0

    while True:

        # Capture frame-by-frame
        ret, frame = video_capture.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 1
        girl_pedestrian = girlPedestrianCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 2
        left_sign = leftCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 3
        parking_sign = parkingCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 4
        pedestrian_sign = pedestrianCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 5
        right_sign = rightCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 6
        stop_sign = stopCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        # 7
        skip = skipCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )




        # Draw a rectangle around the faces
        for (x, y, w, h) in girl_pedestrian:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            pgirl_cas.value = 1
            print("Girl Pedestrian")

        for (x, y, w, h) in left_sign:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            l_cas.value = 1
            print("Left sign")

        for (x, y, w, h) in parking_sign:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            park_cas.value = 1
            print("Parking sign")

        for (x, y, w, h) in pedestrian_sign:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            ped_cas.value = 1
            print("Pedestrian sign")

        for (x, y, w, h) in right_sign:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            r_cas.value = 1
            print("Right sign")

        for (x, y, w, h) in right_sign:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            skip.value = 1
            print("Stop sign")

        # Display the resulting frame
        # cv2.imshow('Video', frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        # python3 home/pi/capstone/sensors/main.py
