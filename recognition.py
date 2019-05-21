import time

import cv2
import numpy as np
from scipy.stats import itemfreq


# I took this solution from:
# https://stackoverflow.com/questions/43111029/how-to-find-the-average-colour-of-an-image-in-python-with-opencv#43111221
def get_dominant_color(image, n_colors):
    pixels = np.float32(image).reshape((-1, 3))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
    flags = cv2.KMEANS_RANDOM_CENTERS
    flags, labels, centroids = cv2.kmeans(
        pixels, n_colors, None, criteria, 10, flags)
    palette = np.uint8(centroids)
    return palette[np.argmax(itemfreq(labels)[:, -1])]


clicked = False
def onMouse(event, x, y, flags, param):
    global clicked
    if event == cv2.EVENT_LBUTTONUP:
        clicked = True





def sign(sign_stop, sign_right, sign_left, sign_forward):
    cameraCapture = cv2.VideoCapture(0)  # Put here ID of your camera (/dev/videoN)
    cv2.namedWindow('camera')
    cv2.setMouseCallback('camera', onMouse)
    queue_dict = {}
    # Read and process frames in loop
    success, frame = cameraCapture.read()
    sign_stop.value = 0
    sign_right.value = 0
    sign_left.value = 0
    sign_forward.value = 0

    while success and not clicked:
        cv2.waitKey(1)
        success, frame = cameraCapture.read()

        # Conversion to gray is required to speed up calculations, we would detect
        # the same circles in BGR and GRAY anyways.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Then we blur the entire frame to prevent accidental false circle
        # detections
        img = cv2.medianBlur(gray, 37)
        # Finally, OpenCV built-in algorithm searching for circles. Arguments are a
        # bit tricky. The most useful are minDist (equals to 50 in this example)
        # and param{1,2}. First one represents distance between centers of detected
        # circles so we never have multiple circles in one place. However,
        # increasing this parameter too much may prevent detection of some circles.
        # Increasing param1 increases count of detected circles. Increasing param2
        # drops more false circles.
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,
                                  1, 50, param1=120, param2=40)

        if not circles is None:
            circles = np.uint16(np.around(circles))
            # Filter the biggest circle, we don't want far signs to be detected
            # instead of close ones.
            max_r, max_i = 0, 0
            for i in range(len(circles[:, :, 2][0])):
                if circles[:, :, 2][0][i] > 50 and circles[:, :, 2][0][i] > max_r:
                    max_i = i
                    max_r = circles[:, :, 2][0][i]
            x, y, r = circles[:, :, :][0][max_i]
            # This check prevents program crash when trying to index list out of
            # its range. We actually cut a square with the whole circle inside.
            if y > r and x > r:
                square = frame[y-r:y+r, x-r:x+r]

                dominant_color = get_dominant_color(square, 2)
                global c_stop
                c_stop = 0
                global c_left
                c_left = 0
                global c_right
                c_right = 0
                global c_forward
                c_forward = 0
                global c_forward_left
                c_forward_left = 0
                global c_forward_right
                c_forward_right = 0


                # copied_img = frame
                # def detect_zebra(frame):
                #     height = frame.shape[0]
                #     width = frame.shape[1]
                #     cropped = copied_img[:height, 0:width]
                #     hls = cv2.cvtColor(cropped, cv2.COLOR_RGB2HLS)
                #     white_lower = np.array([0, 0, 200])
                #     white_upper = np.array([145, 60, 255])
                #
                #     white_mask = cv2.inRange(hls, white_lower, white_upper)
                #     nonblack = cv2.countNonZero(white_mask)
                #     # mask = cv2.bitwise_and(yellow_mask, white_mask)
                #     masked = cv2.bitwise_and(frame, frame, mask=white_mask)
                #     # return masked_img
                #     return True if nonblack >= 1 else False
                # detect_zebra()

                if dominant_color[2] > 100:
                    sign_stop.value = 1
                    print("STOP")
                    # Stop sign is red, so we check if there is a lot of red color
                    # in circle.
                    queue_dict['stop'] = time.time()
                    # if time.time() - queue_dict['stop'] >= 0.4:
                    c_stop = c_stop + 1
                    if c_stop == 5:
                        c_stop = 0
                        print("STOP ------------------")
                elif dominant_color[0] > 80:
                    # Other signs are blue.

                    # Here we cut 3 zones from the circle, then count their
                    # dominant color and finally compare.
                    zone_0 = square[square.shape[0]*3//8:square.shape[0]
                                    * 5//8, square.shape[1]*1//8:square.shape[1]*3//8]
                    zone_0_color = get_dominant_color(zone_0, 1)

                    zone_1 = square[square.shape[0]*1//8:square.shape[0]
                                    * 3//8, square.shape[1]*3//8:square.shape[1]*5//8]
                    zone_1_color = get_dominant_color(zone_1, 1)

                    zone_2 = square[square.shape[0]*3//8:square.shape[0]
                                    * 5//8, square.shape[1]*5//8:square.shape[1]*7//8]
                    zone_2_color = get_dominant_color(zone_2, 1)

                    if zone_1_color[2] < 60:
                        if sum(zone_0_color) > sum(zone_2_color):
                            sign_left.value = 1
                            print("LEFT")
                            queue_dict['left'] = time.time()
                            # if time.time() - queue_dict['left'] >= 0.4:
                            c_left = c_left + 1
                            if c_left == 5:
                                c_left = 0
                                print("LEFT ------------------")
                        else:
                            if zone_1_color[2] < 60:
                                if sum(zone_0_color) > sum(zone_2_color):
                                    sign_right.value = 1
                                    print("RIGHT")
                                    queue_dict['right'] = time.time()
                                    # if time.time() - queue_dict['right'] >= 0.4:
                                    c_right = c_right + 1
                                    if c_right == 5:
                                        c_right = 0
                                        print("RIGHT ------------------")
                    else:
                        if sum(zone_1_color) > sum(zone_0_color) and sum(zone_1_color) > sum(zone_2_color):
                            if zone_1_color[2] < 60:
                                if sum(zone_0_color) > sum(zone_2_color):
                                    sign_forward.value = 1
                                    print("FORWARD")
                                    queue_dict['forward'] = time.time()
                                    # if time.time() - queue_dict['forward'] >= 0.4:
                                    c_forward = c_forward + 1
                                    if c_forward == 5:
                                        c_forward = 0
                                        print("FORWARD ------------------")
                        elif sum(zone_0_color) > sum(zone_2_color):
                            sign_left.value = 1
                            print("FORWARD AND LEFT")
                            c_forward_left = c_forward_left + 1
                            if c_forward_left == 5:
                                c_forward_left = 0
                                print("FORWARD LEFT ------------------")
                        else:
                            sign_right.value = 1
                            print("FORWARD AND RIGHT")
                            c_forward_right = c_forward_right + 1
                            if c_forward_right == 5:
                                c_forward_right = 0
                                print("FORWARD RIGHT ------------------")
                else:
                    print("N/A")

            # Draw all detected circles on the screen
            for i in circles[0, :]:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        # cv2.imshow('camera', frame)


    cv2.destroyAllWindows()
    cameraCapture.release()
