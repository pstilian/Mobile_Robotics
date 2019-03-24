
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

fps = 0.0
prev = 0.0
x_pos = 0

kpValue = 0.9

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
      elif ipsLeft >= 0 and ipsRight < 0:
            # Setting appropriate speedsto the servos while making a turn
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))

# Sets boundary speed for robot movement
def saturationFunction(ips):
	controlSignal = ips
	if controlSignal > 4.0:
		controlSignal = 4.0
	elif controlSignal < -4.0:
		controlSignal = -4.0
	return controlSignal

# Sets boundary speed for robot movement
def saturationFunctionGoalFacing(ips):
	controlSignal = ips
	if controlSignal > 0.5:
		controlSignal = 0.5
	elif controlSignal < -0.5:
		controlSignal = -0.5
	return controlSignal

# Pivots robot on an axis to make a left turn
def leftTurn():
      setSpeedsIPS(1.3,-1.3)
      time.sleep(1)
      setSpeedsIPS(0,0)

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

def faceGoal():
    find = True
    pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))
    while True:
        # Calculate FPS
        now = time.time()
        fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
        prev = now

        # Get a frame
        frame = camera.read()
        
        # Blob detection works better in the HSV color space 
        # (than the RGB color space) so the frame is converted to HSV.
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Create a mask using the given HSV range
        mask = cv.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
        
        # Run the SimpleBlobDetector on the mask.
        # The results are stored in a vector of 'KeyPoint' objects,
        # which describe the location and size of the blobs.
        keypoints = detector.detect(mask)
        
        # For each detected blob, draw a circle on the frame
        frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # Write text onto the frame
        cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        
        # Display the frame
        cv.imshow(WINDOW1, mask)
        cv.imshow(WINDOW2, frame_with_keypoints)

        if keypoints:
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            find = False
            
        if not keypoints:
            pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))


def motionToGoal():
	print("IM GOING THE GOALLLLLL!!!!")
	if x_pos > 260 and x_pos < 280
		sensorCount = 0

		# Gets Distance From Sensor
		fDistance = fSensor.get_distance()
		# Converts readings from milimeters to inches
		inchDistance = fDistance * 0.03937
   		# 0.394 is the conversion rate from cm to inches Determining error amount

    	# fError is the calculated respective error value aka the e(t) value
		error = 5.0 - inchDistance

    	# Control Signal aka u(t)  = Kp * e(t)
		controlSignal = kpValue * error

    	# Calculating new control signal value by running control signal through saturation function
		newSignal = saturationFunction(controlSignal)

		setSpeedsIPS(newSignal, newSignal)

    
    # Initialization functions

# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)

#fpsvalue for smoothing
FPS_SMOOTHING = 0.9

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

pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))


# Create trackbars
#cv.createTrackbar("Min Hue", WINDOW1, minH, 180, onMinHTrackbar)
#cv.createTrackbar("Max Hue", WINDOW1, maxH, 180, onMaxHTrackbar)
#cv.createTrackbar("Min Sat", WINDOW1, minS, 255, onMinSTrackbar)
#cv.createTrackbar("Max Sat", WINDOW1, maxS, 255, onMaxSTrackbar)
#cv.createTrackbar("Min Val", WINDOW1, minV, 255, onMinVTrackbar)
#cv.createTrackbar("Max Val", WINDOW1, maxV, 255, onMaxVTrackbar)

startFlag = False
selectCommand = ' '

# Holds program until command value is entered
while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag =True

# 
while startFlag:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    frame = camera.read()
    
    # Blob detection works better in the HSV color space 
    # (than the RGB color space) so the frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # Create a mask using the given HSV range
    mask = cv.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))
    
    # Run the SimpleBlobDetector on the mask.
    # The results are stored in a vector of 'KeyPoint' objects,
    # which describe the location and size of the blobs.
    keypoints = detector.detect(mask)
    
    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Write text onto the frame
    cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

    # Display the frame
    cv.imshow(WINDOW1, mask)
    cv.imshow(WINDOW2, frame_with_keypoints)

    # Prints FPS and number of blobs on string
    print("FPS : ", fps)
    print("Number of Blobs", len(keypoints))

    for keypoint in keypoints:
        x_pos = keypoint.pt[0]
        print("x: ", x_pos)

    if len(keypoints) < 1:
        pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))

    elif len(keypoints) >= 1:
    	motionToGoal()

    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

# Stop using camera
camera.stop()
# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()