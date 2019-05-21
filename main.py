import numpy as np
from multiprocessing import Process, Pipe
from multiprocessing import Value
import RPi.GPIO as GPIO
import allcases
import time
import cv2
import sign_cascade

HIGH = True
LOW = False
# setup GPIO pins
GPIO.setmode(GPIO.BCM)  # set GPIO numbering mode
GPIO.setwarnings(False)

IR_RIGHT_PIN = 27
IR_LEFT_PIN = 22

IR_LINE_RIGHT_PIN = 19
IR_LINE_LEFT_PIN = 26
pins = {18, 23, 24, 25}
enable_pins = {12, 13}

TRIG = 20
ECHO = 21

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# ensure that the Trigger pin is set low, and give the sensor a second to settle.
GPIO.output(TRIG, LOW)
time.sleep(1)

GPIO.setup(IR_RIGHT_PIN, GPIO.IN)
GPIO.setup(IR_LEFT_PIN, GPIO.IN)
GPIO.setup(IR_LINE_RIGHT_PIN, GPIO.IN)
GPIO.setup(IR_LINE_LEFT_PIN, GPIO.IN)
global duty_cycleL
global duty_cycleR

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, HIGH)

for en_pin in enable_pins:
    GPIO.setup(en_pin, GPIO.OUT)

pwmA = GPIO.PWM(12, 100)  # Initialize PWM on pwm pin to 100Hz frequency
pwmB = GPIO.PWM(13, 100)


def goForward():
    duty_cycleL = 30
    duty_cycleR = 30
    pwmA.start(duty_cycleL)

    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def goForwardSlow():
    duty_cycleL = 20
    duty_cycleR = 20
    pwmA.start(duty_cycleL)

    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def goBackward():
    duty_cycleL = 35
    duty_cycleR = 35
    pwmA.start(duty_cycleL)

    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, HIGH)
    GPIO.output(23, LOW)
    GPIO.output(24, HIGH)
    GPIO.output(25, LOW)


def turnLeftCam(cl):
    duty_cycleL = 10
    if cl != None:
        if cl > 10:
            duty_cycleL = cl - cl / 8
    # duty_cycleR = duty_cycleL/8
    duty_cycleR = 5
    # print("cl: {}".format(duty_cycleR))
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnRightCam(cr):
    duty_cycleR = 10
    if cr != None:
        if cr > 10:
            duty_cycleR = cr - cr / 8
    # duty_cycleL = duty_cycleR/8
    duty_cycleL = 5
    # print("cr: {}".format(duty_cycleR))
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnLeftCam11():
    print("------------------------LLLL--------------------")
    duty_cycleL = 55
    duty_cycleR = 8
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnRightCam11():
    print("-------------------RRRRR-----------------")
    duty_cycleL = 8
    duty_cycleR = 55
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnRight():
    duty_cycleL = 50
    duty_cycleR = 50
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, HIGH)
    GPIO.output(25, LOW)


def turnLeft():
    duty_cycleL = 50
    duty_cycleR = 50
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, HIGH)
    GPIO.output(23, LOW)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnRightP():
    duty_cycleL = 50
    duty_cycleR = 50
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, HIGH)
    GPIO.output(25, LOW)


def turnLeftP():
    duty_cycleL = 50
    duty_cycleR = 50
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, HIGH)
    GPIO.output(23, LOW)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def stopMotor():
    GPIO.output(18, LOW)
    GPIO.output(23, LOW)
    GPIO.output(24, LOW)
    GPIO.output(25, LOW)


######################################


def sensors(sline_right, sline_left, sobstacle_right, sobstacle_left):
    print("Process 1 started")
    while True:
        sline_right.value = GPIO.input(IR_LINE_LEFT_PIN)
        sline_left.value = GPIO.input(IR_LINE_RIGHT_PIN)
        sobstacle_right.value = GPIO.input(IR_RIGHT_PIN)
        sobstacle_left.value = GPIO.input(IR_LEFT_PIN)
        data = "{}    {}{}{}{}".format(distance_data.value, line_right.value, line_left.value, obstacle_right.value, obstacle_left.value)
        # print(data)
        # data = "{}{}{}{}".format(line_right, line_left, obstacle_right, obstacle_left)
        # print("sen: {}".format(data))
        # print()


def ultrasonic(distance_data, distance_flag):
    print("Process 2 started")
    # ensure that the Trigger pin is set low, and give the sensor a second to settle.
    GPIO.output(TRIG, LOW)
    time.sleep(1)

    def getDistance():
        pulse_end = 0.0
        pulse_start = 0.0
        GPIO.output(TRIG, HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, LOW)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = (pulse_duration * 33000) / 2
        distance = round(distance, 2)
        # print(distance)
        # print(distance)
        return distance
    while 1:
        dis_flag = 0
        dis = getDistance()
        # print(dis)
        distance_data.value = dis
        if dis < 25:
            dis_flag = 1
        # distance_flag.value = dis_flag


def cam_func(scam_right, scam_left, scam_forward):
    def predict_turn(lines):
        left_slope = []
        right_slope = []
        if (lines is None):
            pass
            scam_right.value = 0
            scam_left.value = 0
            # goForward()
            # print("Going forward")
        else:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                intercept = parameters[1]
                if (slope < 0):
                    # print("left slope: ", slope)
                    left_slope.append(slope)
                else:
                    # print("right slope: ", slope)
                    right_slope.append(slope)
            right_slope_avg = np.average(right_slope)
            # print("right slope avg: ", right_slope_avg)
            left_slope_avg = np.average(left_slope)
            # print("left slope avg: ", left_slope_avg)
            if (right_slope_avg > 0.6 or abs(left_slope_avg) > 0.6):
                cam_forward.value = 1
                scam_right.value = 0
                scam_left.value = 0
                # goForward()
                # print("Cam Going forward")
            elif (right_slope_avg < 0.6):
                pl = float(right_slope_avg)
                pll = abs(pl)*100
                # turnLeftCam(pll)
                # turnLeft()
                scam_right.value = 1
                # time.sleep(0.1)
                # print("Cam Turning Left: {}".format(pll))
            elif (abs(left_slope_avg) < 0.6):
                scam_left.value = 1
                pr = float(left_slope_avg)
                prr = abs(pr) * 100
                # turnRightCam(prr)
                # turnRight()
                # time.sleep(0.1)
                # print("Cam Turnng Right {}".format(prr))

    def make_coordinates(image, line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1 * (3 / 5))
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)

        return np.array([x1, y1, x2, y2])

    def average_slop_intercept(image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            # print("slope: ", slope)
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        # print("left fit:", left_fit)
        # print("right fit", right_fit)
        left_fit_average = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        try:
            left_line = make_coordinates(image, left_fit_average)
            right_line = make_coordinates(image, right_fit_average)
            return np.array([left_line, right_line]), slope
        except Exception as e:
            print(e, '\n')
            return None

    def color_filter(image):
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        white_lower = np.array([0, 110, 0])
        white_upper = np.array([255, 255, 255])
        yellow_lower = np.array([10, 0, 190])
        yellow_upper = np.array([100, 255, 255])

        yellow_mask = cv2.inRange(hls, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hls, white_lower, white_upper)
        mask = cv2.bitwise_or(yellow_mask, white_mask)
        masked = cv2.bitwise_and(image, image, mask=yellow_mask)

        return masked

    def canny(image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny_value = cv2.Canny(blur, 30, 70)

        return canny_value

    def display_lines(image, lines):
        line_img = np.zeros_like(image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 10)

        return line_img

    def region_of_interest(canny_img):
        height = canny_image.shape[0]
        width = canny_image.shape[1]
        polygons = np.array([
            [(0, 400), (0, height), (width, height), (width, 400)]
        ])
        mask = np.zeros_like(canny_img)
        cv2.fillPoly(mask, polygons, 255)

        # isolating region of interest
        masked_img = cv2.bitwise_and(canny_img, mask)

        return masked_img

    cap = cv2.VideoCapture(0)

    try:
        while True:
            _, frame = cap.read()
            filtered_image = color_filter(frame)
            canny_image = canny(frame)
            cropped_image = region_of_interest(canny_image)

            # Hough line Transform
            lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180, 100,
                                    np.array([]), minLineLength=40, maxLineGap=5)
            # print("lines: ", lines)

            predict_turn(lines)
            # averaged_lines = average_slop_intercept(frame, lines)



            # line_image = display_lines(frame, lines)
            # combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
            # cv2.imshow("result", combo_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("CTRL+C is pressed for ending")

    pwmA.stop()  # stop pwm
    pwmB.stop()
    GPIO.cleanup()

    cap.release()
    cv2.destroyAllWindows()


def bin(inp):
    b = "{}".format(inp)
    if b == '0':
        return '1'
    else:
        return '0'

def main_func(distance_data, line_right, line_left, obstacle_right, obstacle_left, cam_right, cam_left, cam_forward,
              distance_flag, pgirl_cas, l_cas, park_cas, ped_cas, r_cas, stop_cas, skip):
    u_flag = True
    pedestrian_flag = True
    direction_flag = False
    ped_time = 0
    dist_flag = "{}".format(distance_flag.value)
    init_time = time.time()
    # global u_flag
    skipskip = True
    skipskip_time = 0
    while 1:
        cascade = "{}{}{}{}{}{}".format(pgirl_cas.value, l_cas.value, park_cas.value, ped_cas.value, r_cas.value, stop_cas.value)
        data = "{}{}{}{}{}{}".format(bin(line_right.value), bin(line_left.value), bin(obstacle_right.value), bin(obstacle_left.value),
                                     cam_left.value, cam_right.value)
        skip_lane = "{}".format(skip.value)
        cam_r = "{}".format(cam_right.value)
        cam_l = "{}".format(cam_left.value)
        cam_f = "{}".format(cam_forward.value)
        sign_data = "{}{}{}{}".format(sign_stop.value, sign_right.value, sign_left.value, sign_forward.value)

        try:
            if data == "000000":
                goForward()
            elif skip_lane == "1":
                goForward()
                print("skip pedestrian lane")
                time.sleep(2.5)
            elif data == allcases.camera_right: # and (time.time() - init_time > 25):
                turnRightCam11()
            elif data == allcases.camera_left: # and (time.time() - init_time > 25):
                turnLeftCam11()
            elif data[0] == "1" and data[1] == "1":
                goForward()
                time.sleep(2)
            elif data in allcases.right:
                if data[0] == "1":
                    turnRightP()
                    time.sleep(0.21)
                    print("Turn Right line")
                else:
                    turnRight()
                    time.sleep(0.19)
                    # if (time.time() - init_time) > 25:
                    #     global direction_flag
                    #     direction_flag = True
                    print("Turn Right obstacle")
            elif data in allcases.left:
                if data[1] == "1":
                    turnLeftP()
                    time.sleep(0.21)
                    print(" Turn Left line")
                else:
                    turnLeft()
                    time.sleep(0.19)
                    # if (time.time() - init_time) > 25:
                    # if obstacle_left
                    #     global direction_flag
                    #     direction_flag = True
                    print("Turn left obstacle")

            elif data in allcases.back_turn_right: # and (time.time() - init_time > 25):
                goBackward()
                time.sleep(0.7)
                turnLeft()
                time.sleep(0.7)
            elif data in allcases.back_turn_left: # and u_flag
                goBackward()
                time.sleep(0.7)
                turnRight()
                time.sleep(0.7)
            elif data in allcases.back_left: # and (time.time() - init_time > 25): #1101  # 1110
                goBackward()
                time.sleep(0.7)
                turnLeft()
                time.sleep(0.8)
                goForward()
                time.sleep(0.5)
                turnRight()
                time.sleep(0.5)
            elif data in allcases.back_right: # and (time.time() - init_time > 25):
                goBackward()
                time.sleep(0.7)
                turnRight()
                time.sleep(0.8)
                goForward()
                time.sleep(0.5)
                turnLeft()
                time.sleep(0.5)
            elif cascade[0] == "1":  # girl
                stopMotor()
                time.sleep(4)
            elif cascade[4] == "1":  # right direction
                goForward()
                time.sleep(0.2)
                turnRightCam()
                time.sleep(0.3)
                goForward()
                time.sleep(0.2)
            elif cascade[1] == "1":  # left direction
                goForward()
                time.sleep(0.2)
                turnLeftCam()
                time.sleep(0.3)
                goForward()
                time.sleep(0.2)
            elif cascade[2] == "1":  # parking
                goForward()
                time.sleep(0.6)
                stopMotor()
                time.sleep(15)
            elif cascade[3] == "1":  # pedestrian sign
                flag_skip_pedlane = True  # time skip 15 sek
                goForwardSlow()
                # setPWM_slow()
                # global skipskip
                # skipskip = False
                time.sleep(0.5)

            elif cascade[5] == "1":  # stop sign
                stopMotor()
                time.sleep(2)
            elif (data[2] == "1" or data[3] == "1"):  # and u_flag
                stopMotor()
                time.sleep(1)
                goBackward()
                time.sleep(0.3)
            # elif data in ("0011", "1001", "0001", "0110", "0010", "1100", "0100", "1000", "0000"):
            else:
                # stopMotor()
                print("Else Stopped")

        except KeyboardInterrupt:
            print("CTRL+C is pressed for ending")


# def fork2():
#     # print("Distance data: {}".format(distance_data))
#     # p_main = Process(target=main_func, args=(distance_data,))
#     p_main = Process(target=main_func)
#     p_main.start()
#     p_main.join()


if __name__ == '__main__':
    try:
        distance_data = Value('d')
        distance_flag = Value('i')
        line_right = Value('i')
        line_left = Value('i')
        obstacle_right = Value('i')
        obstacle_left = Value('i')
        cam_right = Value('i')
        cam_left = Value('i')
        cam_forward = Value('i')

        sign_stop = Value('i')
        sign_right = Value('i')
        sign_left = Value('i')
        sign_forward = Value('i')

        pgirl_cas = Value('i')
        l_cas = Value('i')
        park_cas = Value('i')
        ped_cas = Value('i')
        r_cas = Value('i')
        stop_cas = Value('i')
        skip = Value('i')

        p_sensor = Process(target=sensors, args=(line_right, line_left, obstacle_right, obstacle_left,))
        p_ultrasonic = Process(target=ultrasonic, args=(distance_data, distance_flag))
        p_cam = Process(target=cam_func, args=(cam_right, cam_left, cam_forward, ))
        # p_sign = Process(target=recognition.sign, args=(sign_stop, sign_right, sign_left, sign_forward,))
        p_main = Process(target=main_func, args=(distance_data, line_right, line_left, obstacle_right, obstacle_left,
                                                 cam_right, cam_left, cam_forward,
                                                distance_flag, pgirl_cas, l_cas, park_cas, ped_cas, r_cas, stop_cas,
                                                 skip,))
        p_sign_cascade = Process(target=sign_cascade.cascade_func, args=(pgirl_cas, l_cas, park_cas, ped_cas, r_cas, stop_cas, skip))
        processes_fork1 = (p_sensor, p_ultrasonic, p_main, p_cam, p_sign_cascade)

        # p_cam.start()
        # p_cam.join()

        for p1 in processes_fork1:
            p1.start()

        for p1 in processes_fork1:
            p1.join()
    except KeyboardInterrupt:
        p_cam.terminate()
        # p_sign.terminate()
        p_sensor.terminate()
        p_ultrasonic.terminate()
        p_main.terminate()
        p_sign_cascade.terminate()
        print('\nCamera process stopped Ctrl+C')

pwmA.stop()  # stop pwm
pwmB.stop()
GPIO.cleanup()  # resets GPIO ports used back to input mode




# python3 home/pi/capstone/sensors/main.py
#       echo 1 > /proc/sys/vm/drop_caches
#   sync;      echo 2 > /proc/sys/vm/drop_caches could not close the output stream for file

