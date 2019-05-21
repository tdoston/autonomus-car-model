import numpy as np
from multiprocessing import Process, Pipe
from multiprocessing import Value
import RPi.GPIO as GPIO
import time
import cv2

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
    duty_cycleL = 20
    duty_cycleR = 20
    pwmA.start(duty_cycleL)

    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnLeft():
    duty_cycleL = 30
    duty_cycleR = 30
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, HIGH)
    GPIO.output(23, LOW)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)

def turnLeftCam():
    duty_cycleL = 48
    duty_cycleR = 5
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)


def turnRightCam():
    duty_cycleL = 5
    duty_cycleR = 48
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle

    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, LOW)
    GPIO.output(25, HIGH)

def turnRight():
    duty_cycleL = 40
    duty_cycleR = 40
    pwmA.start(duty_cycleL)
    pwmB.start(duty_cycleR)  # start pwm with 0% duty cycle
    GPIO.output(18, LOW)
    GPIO.output(23, HIGH)
    GPIO.output(24, HIGH)
    GPIO.output(25, LOW)


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
        data = "{}    {}{}{}{}".format(distance_data.value, line_right.value, line_left.value, obstacle_right.value,
                                 obstacle_left.value)
        # print(data)
        # data = "{}{}{}{}".format(line_right, line_left, obstacle_right, obstacle_left)
        # print("sen: {}".format(data))
        # print()


def ultrasonic(distance_data):
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
        return distance
    while 1:
        distance_data.value = getDistance()


def main_func(distance_data, line_right, line_left, obstacle_right, obstacle_left,):
    while 1:
        data = "{}{}{}{}".format(line_right.value, line_left.value, obstacle_right.value, obstacle_left.value)
        # da = distance_data.value
        # print("distance: {} line_r: {} line_l: {}  obs_r: {} obs_l: {}".format(da, data[0], data[1], data[2], data[3]))
        try:
            if data == "1111":
                goForward()
                print("Go Forward")
            elif data in ("1011", "1110", "1010"):
                    turnLeftCam()
                    print("Turn left")
            elif data in ("0111", "1101", "0101"):
                    turnRightCam()
                    print("Turn Right")
            elif data in ("0011", "1001", "0001", "0110", "0010", "1100", "0100", "1000", "0000"):
                stopMotor()
                print("Stopped")

        except KeyboardInterrupt:
            print("CTRL+C is pressed for ending")


if __name__ == '__main__':
    try:
        distance_data = Value('d')
        line_right = Value('i')
        line_left = Value('i')
        obstacle_right = Value('i')
        obstacle_left = Value('i')

        p_sensor = Process(target=sensors, args=(line_right, line_left, obstacle_right, obstacle_left,))
        p_ultrasonic = Process(target=ultrasonic, args=(distance_data,))
        p_main = Process(target=main_func, args=(distance_data, line_right, line_left, obstacle_right, obstacle_left,))
        processes_fork1 = (p_sensor, p_ultrasonic, p_main,)

        for p1 in processes_fork1:
            p1.start()

        for p1 in processes_fork1:
            p1.join()
    except KeyboardInterrupt:
        print('got Ctrl+C')

pwmA.stop()  # stop pwm
pwmB.stop()
GPIO.cleanup()  # resets GPIO ports used back to input mode

