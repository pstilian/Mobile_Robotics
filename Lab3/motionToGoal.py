
import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import decimal
import math
import Adafruit_PCA9685
import csv

import cv2 as cv
from ThreadedWebcam import ThreadedWebcam
from UnthreadedWebcam import UnthreadedWebcam

#Global Variables
startTime = time.time()
currentTime = 0

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second.
LSERVO = 0
RSERVO = 1

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

FPS_SMOOTHING = 0.9

# Window names
WINDOW1 = "Adjustable Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Default HSV ranges
# Note: the range for hue is 0-180, not 0-255
minH =   88; minS = 148; minV = 92;
maxH = 180; maxS = 255; maxV = 255;



def ctrlC(signum, frame):
    print("Exiting")
    
    GPIO.cleanup()
    # Write an initial value of 1.5, which keeps the servos stopped.
    # Due to how servos work, and the design of the Adafruit library,
    # The value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

    exit()

def readCSV():
    global LWSpeed, RWSpeed
    with open('LeftSpeedCalibration.csv', newline='') as LeftCalibrate:
        lReader = csv.reader(LeftCalibrate)
        LWSpeed = dict(map(float,x) for x in lReader) # pulls in each row as a key-value pair
        
    with open('RightSpeedCalibration.csv', newline= '') as RightCalibrate:
        rReader = csv.reader(RightCalibrate)
        RWSpeed = dict(map(float,x) for x in rReader) # pulls in each row as a key-value pair

def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff
        
# Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

    # makes sure RPS is always a positive number
    if rpsLeft < 0:
        rpsLeft = 0 - rpsLeft
    if rpsRight < 0:
        rpsRight = 0 - rpsRight

    # Calculating pwm values from the respective dictionaries
    lPwmValue = float(LWSpeed[rpsLeft])
    rPwmValue = float(RWSpeed[rpsRight])

    if ipsLeft < 0 and ipsRight < 0:
        # Setting appropiate speeds to the servos when going forwards
        pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))
    elif ipsLeft >= 0 and ipsRight >= 0:
        # Setting apporpiate speeds to the servos when going backwards
        pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))

# Sets boundary speed for robot movement
def saturationFunction(ips):
    controlSignal = ips
    if controlSignal > 7.1:
        controlSignal = 7.1
    elif controlSignal < -7.1:
        controlSignal = -7.1
    return controlSignal

###### OPEN CV FUNCTIONS #######

def onMinHTrackbar(val):
    # Calculate a valid minimum red value and re-set the trackbar.
    global minH
    global maxH
    minH = min(val, maxH - 1)
    cv.setTrackbarPos("Min Hue", WINDOW1, minH)

def onMinSTrackbar(val):
    global minS
    global maxS
    minS = min(val, maxS - 1)
    cv.setTrackbarPos("Min Sat", WINDOW1, minS)

def onMinVTrackbar(val):
    global minV
    global maxV
    minV = min(val, maxV - 1)
    cv.setTrackbarPos("Min Val", WINDOW1, minV)

def onMaxHTrackbar(val):
    global minH
    global maxH
    maxH = max(val, minH + 1)
    cv.setTrackbarPos("Max Hue", WINDOW1, maxH)

def onMaxSTrackbar(val):
    global minS
    global maxS
    maxS = max(val, minS + 1)
    cv.setTrackbarPos("Max Sat", WINDOW1, maxS)

def onMaxVTrackbar(val):
    global minV
    global maxV
    maxV = max(val, minV + 1)
    cv.setTrackbarPos("Max Val", WINDOW1, maxV)

    # Initialization functions

# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)

# Initialized servos to zero movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(2)

# Imports dictionary from the calibration CSV file
readCSV()

# Initialize the threaded camera
# You can run the unthreaded camera instead by changing the line below.
# Look for any differences in frame rate and latency.
camera = ThreadedWebcam() # UnthreadedWebcam()
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
# Otherwise, one will be generated.
# These values WILL need to be adjusted for accurate and fast blob detection.
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
if fs.isOpened():
    detector.read(fs.root())
else:
    print("WARNING: params file not found! Creating default file.")
    
    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
    detector.write(fs2)
    fs2.release()
    
fs.release()

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)


    ########################## MAIN LINE CODE ####################################