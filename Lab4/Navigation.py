
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
import random

import cv2 as cv
from ThreadedWebcam import ThreadedWebcam

#Intialize maps
startTime = time.time()
currentTime = 0
dAxis = float(3.95)
LWSpeed = {}
RWSpeed = {}

# Used Values
lCount = 0
rCount = 0
TotalCount = (0, 0)
lSpeed = 0
rSpeed = 0
lRevolutions = 0
rRevolutions = 0
distanceT = 0 # Distance Traveled


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


########################### USED FUNCTIONS ###########################################


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
        RWSpeed = dict(map(float,x) for x in rReader) # pulls in each row as a key-value pai


# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global lCount, lRevolutions, lSpeed, currentTime
    lCount += 1
    lRevolutions = float(lCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime
    

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rCount, rRevolutions, rSpeed, currentTime
    rCount += 1
    rRevolutions = float(rCount / 32)
    currentTime = time.time() - startTime
    lSpeed = rRevolutions / currentTime

# Resets the tick count
def resetCounts():
    global TotalCount, Lcount, rCount, startTime
    lCount = 0
    rCount = 0
    startTime = time.time()

# Returns the previous tick counts as a touple
def getCounts():
    return (lCount, rCount)

# Returns instantious speeds for both left and right wheels as a touple
def getSpeeds():
    global lCount, rCount, currentTime, lSpeed, rSpeed
    currentTime = time.time() - startTime
    lSpeed = (lCount / 32) / currentTime
    rSpeed = (rCount / 32) / currentTime
    return (lSpeed, rSpeed)

# Function that sets up and initializes the econders for the robot
def initEncoders():
    # Set the pin numbering scheme to the numbering shown on the robot itself.
    GPIO.setmode(GPIO.BCM)
    # Set encoder pins as input
    # Also enable pull-up resistors on the encoder pins
    # This ensures a clean 0V and 3.3V is always outputted from the encoders.
    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Attach a rising edge interrupt to the encoder pins
    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# This function changes the values from 1.5 - 1.7 to 1.5-1.3 basically outputting inverse pwm values
def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff

 # Sets boundary speed for robot movement
def saturationFunction(ips):
	controlSignal = ips
	if controlSignal > 7.1:
		controlSignal = 7.1
	elif controlSignal < -7.1:
		controlSignal = -7.1
	return controlSignal

# Sets boundary speed for robot movement
def saturationFunctionWallFollowing(ips):
	controlSignal = ips
	if controlSignal > 0.5:
		controlSignal = 0.5
	elif controlSignal < -0.5:
		controlSignal = -0.5
	return controlSignal


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
      		#S Setting appropriate speeds tot he servos while turning
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))


def setSpeedsvw(v, w):
      velocityLeft1 = float(( v + ( w * dAxis)))
      velocityRight1 = float(( v - ( w * dAxis)))

      setSpeedsIPS(-velocityLeft1, -velocityRight1)


# Function for determinng robot actions throught the maze
def whatDo(lWallOpen, fWallOpen, rWallOpen):
	# Initialize "option" variable
	# 1 = Left turn, 2 = Move Forward, 3 = Right Turn
	option = 0

	if fInchDistance < 18:
		frontDist()

	if lWallOpen and fWallOpen and rWallOpen:
		choices = [1, 2, 3]
		option = random.choice(choices)

	elif fWallOpen and rWallOpen:
		choices = [2, 3]
		option = random.choice(choices)

	elif lWallOpen and rWallOpen:
		choices = [1, 3]
		option = random.choice(choices)

	elif lWallOpen and fWallOpen:
		choices = [1, 2]
		option = random.choice(choices)

	elif rWallOpen:
		option = 3

	elif fWallOpen:
		option = 2

	elif lWallOpen:
		option = 1

	else:
		option = 0

	## DETERMINE THE MOVEMENTS OF ROBOT
	# Option 1 demands left turn
	if option == 1:
		print("I'm turning left")
		leftPivot()

	# Option 2 demands move forward
	elif option == 2:
		# needs to move forward
		print("Moving Forward")
		#moveForward()

	# Option 3 demands right turn
	elif option == 3:
		print("I'm turning right")
		rightPivot()

	else:
		print("OH NO IM TRAPPED!! 0.0")
		turnAround()

def stop():
	pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
	pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
	time.sleep(1)


def leftPivot():
    global distanceT, lRevolutions, rRevolutions
    stop()

    pwm.set_pwm(LSERVO, 0, math.floor(1.46 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.42 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1


def rightPivot():
    global distanceT, lRevolutions, rRevolutions
    stop()

    pwm.set_pwm(LSERVO, 0, math.floor(1.58 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.54 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1

def turnAround():
    global distanceT, lRevolutions, rRevolutions
    stop()

    pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096))
    time.sleep(2.05)
    lRevolutions = 1.1
    rRevolutions = 1.1

# Maneuvers robot back to center of cell works like motion to goal / wall distance
def frontDist():
    global lRevolutions, rRevolutions

    fCount = 0

    while True:
        # Gets Distance From Sensor
        fDistance = fSensor.get_distance()
	# Converts readings from milimeters to inches
        inchDistance = fDistance * 0.03937
   	# 0.0394 is the conversion rate from mm to inches Determining error amount

   	# Calculating the front error
        fError = 8 - inchDistance

    	# Control Signal aka u(t)  = Kp * e(t)
        controlSignal = kpValue * fError

        # Calculating new control signal value by running control signal through saturation function
        newSignal = saturationFunction(controlSignal)

        setSpeedsIPS(newSignal, newSignal)

        # When robot is 8 inches from wall adjustment is complete
        if fError < 0.5 and fError > -0.5:
            break

        # makes sure robot doesnt get stuck from retardation
        fCount = fCount + 1
        if fCount > 70:
            break

    lRevolutions = 1.1
    rRevolutions = 1.1

def moveForward():
    global sCount

    #measure distances..
    fDistance = fSensor.get_distance()
    lDistance = lSensor.get_distance()
    rDistance = rSensor.get_distance()

	# Converts readings from milimeters to inches
    finchDistance = fDistance * 0.03937
    linchDistance = lDistance * 0.03937
    rinchDistance = rDistance * 0.03937

    # fError is the calculated respective error value aka the e(t) value
    fError = 7.0 - fInchDistance
    lError = 8.0 - lInchDistance
    rError = 8.0 - rInchDistance

    # Control Signal aka u(t)  = Kp * e(t)
    fControlSignal = kpValue * fError
    lControlSignal = kpValue * lError
    rControlSignal = kpValue * rError

    # Calculating new control signal value by running control signal through saturation function
    fNewSignal = saturationFunction(fControlSignal)
    lNewSignal = saturationFunctionWallFollowing(lControlSignal)
    rNewSignal = saturationFunctionWallFollowing(rControlSignal)

    if rInchDistance > lInchDistance and rInchDistance < 12.0:
    	setSpeedsvw(linearSpeed, lNewSignal/3)
    elif rInchDistance > lInchDistance and rInchDistance > 12.0:
    	setSpeedsvw(linearSpeed, lNewSignal/3)
    elif rInchDistance < lInchDistance and lInchDistance < 12.0:
    	setSpeedsvw(linearSpeed, -rNewSignal/3)
    elif rInchDistance < lInchDistance and lInchDistance > 12.0:
    	setSpeedsvw(linearSpeed, -rNewSignal/3)
    else:
    	setSpeedsvw(linearSpeed, 0)

    # Checks for obstacle to the front if 5 consecutive reading are made robot makes a left turn
    if fInchDistance < 5.0:

        sensorCount += 1

        if sensorCount > 4:
            frontDist()

    else:
        sensorCount = 0

################################################################################
########################### INITALIZATION FUNCTIONS ############################
################################################################################

# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)
initEncoders()

# Imports dictionary from the calibration CSV file
readCSV()


################################################################################
############################### MAIN LINE CODE #################################
################################################################################

# Utilizes 3 flags one for each detectable wall to see if there is an opening or not
# These Flags are lWallOpen, rWallOpen and fWallOpen

stop()

fWallOpen = False
rWallOpen = False
lWallOpen = False
startFlag = False
newCell = True

linearSpeed = 5
sCount = 0
selectCommand = ' '

# Holds program until command value is entered
while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag = True

# Reads Distance From Sensors
fDistance = fSensor.get_distance()
rDistance = rSensor.get_distance()
lDistance = lSensor.get_distance()

# Converts readings from centimeters to inches
fInchDistance = fDistance * 0.0394
rInchDistance = rDistance * 0.0394
lInchDistance = lDistance * 0.0394
# 0.394 is the conversion rate from millimeters to inches Determining error amount

# Initialize the current cell
if fInchDistance > 15:
	fWallOpen = True

if rInchDistance > 15:
	rWallOpen = True

if lInchDistance > 15:
	lWallOpen = True


#This loop controls random maze navigation
while True:

    # Reads Distance From Sensors
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    lDistance = lSensor.get_distance()

    # Converts readings from centimeters to inches
    fInchDistance = fDistance * 0.0394
    rInchDistance = rDistance * 0.0394
    lInchDistance = lDistance * 0.0394
    # 0.394 is the conversion rate from millimeters to inches Determining error amount

    if fInchDistance > 15:
    	fWallOpen = True

    if rInchDistance > 15:
    	rWallOpen = True

    if lInchDistance > 15:
    	lWallOpen = True

    # Records Distance Traveled by Robot
    distanceT = ( 8.20 * ((lRevolutions + rRevolutions) / 2) )

    if distanceT > 9 and newCell:
    	print("I have entered a new cell yay!")
    	newCell = False

    if distanceT > 18:

    	whatDo(lWallOpen, fWallOpen, rWallOpen)
    	newCell = True

    	distanceT = 0
    	resetCounts()

    fWallOpen = False
    rWallOpen = False
    lWallOpen = False

    moveForward()
