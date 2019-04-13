import Adafruit_PCA9685
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import math
import time
import csv
import random

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

#Values
lTickCount = 0
rTickCount = 0
lSpeed = 0
rSpeed = 0
currentTime = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()
LWSpeed = {}
RWSpeed = {}

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

def readCSV():
    global LWSpeed, RWSpeed
    with open('LeftSpeedCalibration.csv', newline='') as LeftCalibrate:
        lReader = csv.reader(LeftCalibrate)
        LWSpeed = dict(map(float,x) for x in lReader) # pulls in each row as a key-value pair
        
    with open('RightSpeedCalibration.csv', newline= '') as RightCalibrate:
        rReader = csv.reader(RightCalibrate)
        RWSpeed = dict(map(float,x) for x in rReader) # pulls in each row as a key-value pai

#Function that resets the total count of ticks
def resetCounts():
    global totalCountTuple, lTickCount, rTickCount, startTime
    lTickCount = 0
    rTickCount = 0
    startTime = time.time()

#Function that gets previous tick counts
def getCounts():
    return (lTickCount, rTickCount)

#Function that returns instantaneous left and right wheel speeds
def getSpeeds():
    global lTickCount, rTickCount, currentTime, lSpeed, rSpeed
    currentTime = time.time() - startTime
    lSpeed = (lTickCount / 32) / currentTime
    rSpeed = (rTickCount / 32) / currentTime
    return (lSpeed, rSpeed)

# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global lTickCount, lRevolutions, lSpeed, currentTime

    # Increasing tickcount and computing instantaneous values
    lTickCount = lTickCount + 1
    lRevolutions = float(lTickCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rTickCount, rRevolutions, rSpeed, currentTime

    # Increasing tickcount and computing instantaneous values
    rTickCount = rTickCount + 1
    rRevolutions = float(rTickCount / 32)
    currentTime = time.time() - startTime
    rSpeed = rRevolutions / currentTime

# Defining function that initializes the encoders
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

def ctrlC(signum, frame):
    print("DONE")
    GPIO.cleanup()

    ## Write an initial value of 1.5, which keeps the servos stopped.
    ## Due to how servos work, and the design of the Adafruit library,
    ## the value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    # Stop measurement for all sensors
    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()
    exit()

# This function changes the values from 1.5 - 1.7 to 1.5-1.3 basically outputting inverse pwm values
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
            #S Setting appropriate speeds tot he servos while turning
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))

# Function to set appropiate boundaries for front sensor
def saturationFunction(ips):
    controlSignal = ips

    # Value of 0.5 is used to limit the range of speeds possible and was reached through trial and error
    if controlSignal > 0.5:
        controlSignal = 0.5
    elif controlSignal < -0.5:
        controlSignal = -0.5
    return controlSignal

# Function to set appropiate boundaries for right sensor
def saturationFunctionRight(inches):
    controlSignal = inches

    #Value of 0.5 is used to limit the range of speeds possible and was reached through trial and error
    if controlSignal > 0.5:
        controlSignal = 0.5
    elif controlSignal < -0.5:
        controlSignal = -0.5
    return controlSignal

# Function to set appropiate boundaries for front sensor
def saturationFunctionFront(ips):
    controlSignal = ips
    if controlSignal > 7.1:
        controlSignal = 7.1
    elif controlSignal < -7.1:
        controlSignal = -7.1
    return controlSignal

def frontDist():
    global lRevolutions, rRevolutions

    frontCount = 0

    while True:
        # Reading in from sensor
        fDistance = fSensor.get_distance()

        # Transforming readings to inches
        inchesDistance = fDistance * 0.0393700787

        # Calculating respective error
        errorFrontAd = 8 - inchesDistance

        # Computing the control signal
        controlSignal = kpValue * errorFrontAd

        # Running control signals through saturation function
        newSignal = saturationFunctionFront(controlSignal)

        # Setting speed of the robot with the newly computed values
        setSpeedsIPS(newSignal, newSignal)

        if errorFrontAd < 0.5 and errorFrontAd > -0.5:
            break

        frontCount = frontCount + 1
        #print(frontCount)

        if frontCount > 80:
            break

    lRevolutions = 1.1
    rRevolutions = 1.1

def setSpeedsvw(v, w):
    leftSpeed1 = (v + (w*3.95))
    rightSpeed1 = (v - (w*3.95))
    setSpeedsIPS(-leftSpeed1, -rightSpeed1)

def stop():
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    time.sleep(1)

# Check if specific flags are set to decide action.
def whatDo(leftWallOpen, frontWallOpen, rightWallOpen):
    option = 0

    if inchesDFront < 18:
        frontDist()

    if leftWallOpen and frontWallOpen and rightWallOpen:
        choices = [1, 2, 3]
        option = random.choice(choices)
        #stop()

    elif frontWallOpen and rightWallOpen:
        choices = [2, 3]
        option = random.choice(choices)
        #stop()

    elif leftWallOpen and rightWallOpen:
        choices = [1, 3]
        option = random.choice(choices)
        #stop()

    elif leftWallOpen and frontWallOpen:
        choices = [1, 2]
        option = random.choice(choices)
        #stop()

    elif rightWallOpen:
        option = 3
        #stop()

    elif frontWallOpen:
        option = 2
        #stop()

    elif leftWallOpen:
        option = 1
        #stop()

    else:
        option = 0
        #stop()

    #CHECK WHAT MOVE I MUST DO!
    if option == 1:
        #turn left
        print("LEFT TURN")
        leftPivot()
    elif option == 2:
        #keep moving forward
        print("MOVE FORWARD")
    elif option == 3:
        #right turn
        print("RIGHT TURN")
        rightPivot()
    else:
        #360 turn and move forward
        print("WALL IN ALL THREE PLACES")
        turnAround()

def moveForward():
    global sensorCount

    # Reading in from sensors
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    lDistance = lSensor.get_distance()

    # Transforming readings to inches
    inchesDistanceFront = fDistance * 0.0393700787
    inchesDistanceRight = rDistance * 0.0393700787
    inchesDistanceLeft = lDistance * 0.0393700787

    # Calculating respective errors
    errorf = 7.0 - inchesDistanceFront
    errorr = 8.0 - inchesDistanceRight
    errorl = 8.0 - inchesDistanceLeft

    # Computing the control signals
    controlSignalf = kpValue * errorf
    controlSignalr = kpValue * errorr
    controlSignall = kpValue * errorl

    # Running control signals through saturation functions
    newSignalf = saturationFunction(controlSignalf)
    newSignalr = saturationFunctionRight(controlSignalr)
    newSignall = saturationFunctionRight(controlSignall)


    if inchesDistanceRight > inchesDistanceLeft and inchesDistanceRight < 12.0:
        setSpeedsvw(linearSpeed,newSignall/3)
    elif inchesDistanceRight > inchesDistanceLeft and inchesDistanceRight > 12.0:
        setSpeedsvw(linearSpeed,newSignall/3)
    elif inchesDistanceRight < inchesDistanceLeft and inchesDistanceLeft < 12.0:
        setSpeedsvw(linearSpeed,-newSignalr/3)
    elif inchesDistanceRight < inchesDistanceLeft and inchesDistanceLeft > 12.0:
        setSpeedsvw(linearSpeed,-newSignalr/3)
    else:
        setSpeedsvw(linearSpeed,0)

    # Checking if there is an object approaching from the front
    if inchesDistanceFront < 5.0:
        # Increasing reading count
        sensorCount += 1

        if sensorCount > 4:
            frontDist()

    # Clearing sensor count for continous small front readings
    else:
        sensorCount = 0

def leftPivot():
    global distanceT, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.47 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.41 / 20 * 4096))
    time.sleep(1.2)
    lRevolutions = 1.1
    rRevolutions = 1.1

def rightPivot():
    global distanceT, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.59 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.53 / 20 * 4096))
    time.sleep(1.2)
    lRevolutions = 1.1
    rRevolutions = 1.1

def turnAround():
    global distanceT, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096))
    time.sleep(2.05)
    lRevolutions = 1.1
    rRevolutions = 1.1

################################################################################
########################### INITALIZATION FUNCTIONS ############################
################################################################################


## Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

# Initializing encoders
initEncoders()

# Imports dictionary from the calibration CSV file
readCSV()

# Declaring variable to keep track of distance traveled
distanceT = 0

# Declaring the kp value to be used
kpValue = 0.7

# Sleeping the motors before starting the movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(0.5)

# Waiting for user to enter the required key in order to start the movement
startFlag = False
selectCommand = ' '
# Holds program until command value is entered
while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag = True

# Declaring constant linear speed that will be used during the movement
linearSpeed = 5

# Declaring a variable to keep track of front sensor big readings
sensorCount = 0

#Booleans to determine if walls are open?
frontWallOpen = False
leftWallOpen = False
rightWallOpen = False
newCell = True


################################################################################
############################### MAIN LINE CODE #################################
################################################################################

while True:
 
    # Waiting for user to enter the required key in order to start the movement
    startFlag = False
    selectCommand = ' '
    # Holds program until command value is entered
    while selectCommand != 's':
        selectCommand = input("""
        Enter \'l\' to make me spin left
        Enter \'l\' to make me spin right or
        Enter \'l\' to make me spin all the way around
        """)
        if selectCommand == 'l':
            leftPivot()
        if selectCommand == 'r':
            rightPivot()
        if selectCommand == 'q':
            turnAround()

        startFlag = True