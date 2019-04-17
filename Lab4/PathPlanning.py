import Adafruit_PCA9685
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import math
import time
import cv2 as cv
import csv
import random
import numpy as np
from ThreadedWebcam import ThreadedWebcam

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

FPS_SMOOTHING = 0.9

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

# Window names
WINDOW1 = "Adjustable Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)

# Setting upper and lower values for yellow, pink green and blue
upper_yellow = np.array([28, 255, 255])
lower_yellow = np.array([24, 137, 78])

upper_pink = np.array([180, 255, 255])
lower_pink = np.array([136, 139, 77])

upper_green = np.array([43, 255, 255])
lower_green = np.array([35, 139, 77])

upper_blue = np.array([103, 255, 255])
lower_blue = np.array([65, 0, 77])

fps, prev = 0.0, 0.0

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
    pwm.set_pwm(LSERVO, 0, math.floor(1.46 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.42 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # Update direction of robot movement
    if direction == 'N':
        direction = 'W'
    elif direction == 'W':
        direction = 'S'
    elif direction == 'S':
        direction = 'E'
    elif direction == 'E':
        direction = 'N'

def rightPivot():
    global distanceT, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.58 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.54 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # Update direction of robot movement
    if direction == 'N':
        direction = 'E'
    elif direction == 'W':
        direction = 'N'
    elif direction == 'S':
        direction = 'W'
    elif direction == 'E':
        direction = 'S'


def turnAround():
    global distanceT, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096))
    time.sleep(2.05)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # Update direction of robot movement
    if direction == 'N':
        direction = 'S'
    elif direction == 'W':
        direction = 'E'
    elif direction == 'S':
        direction = 'N'
    elif direction == 'E':
        direction = 'W'

#################################################################################
            
# You don't have to understand how this function works
def printMaze(maze, hRes = 4, vRes = 2):
    assert hRes > 0, "Invalid horizontal resolution"
    assert vRes > 0, "Invalid vertical resolution"

    # Get the dimensions of the maze drawing
    hChars = 4 * (hRes + 1) + 2
    vChars = 4 * (vRes + 1) + 1
    
    # Store drawing into a list
    output = [" "] * (hChars * vChars - 1)
    
    # Draw top border
    for i in range(1, hChars - 2):
        output[i] = "_"
    
    # Draw bottom border
    for i in range(hChars * (vChars - 1) + 1, hChars * (vChars - 1) + hChars - 2):
        output[i] = "¯"
    
    # Draw left border
    for i in range(hChars, hChars * (vChars - 1), hChars):
        output[i] = "|"
        
    # Draw right border
    for i in range(2 * hChars - 2, hChars * (vChars - 1), hChars):
        output[i] = "|"

    # Draw newline characters
    for i in range(hChars - 1, hChars * vChars - 1, hChars):
        output[i] = "\n"
    
    # Draw dots inside maze
    for i in range((vRes + 1) * hChars, hChars * (vChars - 1), (vRes + 1) * hChars):
        for j in range(hRes + 1, hChars - 2, hRes + 1):
            output[i + j] = "·"
    
    # Draw question marks if cell is unvisited

    for i in range(4):
        for j in range(4):
            cellNum = i * 4 + j
            if maze[cellNum].visited:
                continue
            origin = (i * hChars * (vRes + 1) + hChars + 1) + (j * (hRes + 1))
            for k in range(vRes):
                for l in range(hRes):
                    output[origin + k * hChars + l] = "?"
    
    # Draw horizontal walls
    for i in range(3):
        for j in range(4):
            cellNum = i * 4 + j
            origin = ((i + 1) * hChars * (vRes + 1) + 1) + (j * (hRes + 1))
            hWall = maze[cellNum].south
            for k in range(hRes):
                output[origin + k] = "-" if hWall == 'W' else " " if hWall == 'O' else "?"
    
    # Draw vertical walls
    for i in range(4):
        for j in range(3):
            cellNum = i * 4 + j
            origin = hChars + (hRes + 1) * (j + 1) + i * hChars * (vRes + 1)
            vWall = maze[cellNum].east
            for k in range(vRes):
                output[origin + k * hChars] = "|" if vWall == 'W' else " " if vWall == 'O' else "?"

    # Print drawing
    print(''.join(output))


class Cell:
    def __init__(self, west, north, east, south, visited = False):
        # There are 4 walls per cell
        # Wall values can be 'W', 'O', or '?' (wall, open, or unknown)
        self.west = west
        self.north = north
        self.east = east
        self.south = south
        
        # Store whether or not the cell has been visited before
        self.visited = visited

        
# Helper function that verifies all the walls of the maze
def detectMazeInconsistencies(maze):
    # Check horizontal walls
    for i in range(3):
        for j in range(4):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 4
            hWall1 = maze[pos1].south
            hWall2 = maze[pos2].north       
            assert hWall1 == hWall2, " Cell " + str(pos1) + "'s south wall doesn't equal cell " + str(pos2) + "'s north wall! ('" + str(hWall1) + "' != '" + str(hWall2) + "')"
    
    # Check vertical walls
    for i in range(4):
        for j in range(3):
            pos1 = i * 4 + j
            pos2 = i * 4 + j + 1
            vWall1 = maze[pos1].east
            vWall2 = maze[pos2].west
            assert vWall1 == vWall2, " Cell " + str(pos1) + "'s east wall doesn't equal cell " + str(pos2) + "'s west wall! ('" + str(vWall1) + "' != '" + str(vWall2) + "')"


# This is the most important function of this program. Updates the maze walls based
# on current cell position
def updateMaze(left, front, right, direction, currentCell):
    global maze

    currentCellIndex = currentCell - 1
    maze[currentCellIndex].visited = True

    leftIndex = -1
    upIndex = -1
    rightIndex = -1
    downIndex = -1

    if currentCellIndex % 4 > 0:
        leftIndex = currentCellIndex - 1

    if (currentCellIndex + 1) % 4 > 0:
        rightIndex = currentCellIndex + 1

    if currentCellIndex >= 4:
        upIndex = currentCellIndex - 4

    if currentCellIndex < 12:
        downIndex = currentCellIndex + 4


    #### WEST #####

    if direction == 'W':

        if left == True: 
            maze[currentCellIndex].south = 'O'
        else:
            maze[currentCellIndex].south = 'W'

        if front == True:
            maze[currentCellIndex].west = 'O'
        else:
            maze[currentCellIndex].west = 'W'

        if right == True:
            maze[currentCellIndex].north = 'O'
        else:
            maze[currentCellIndex].north = 'W'

        maze[currentCellIndex].east = 'O'

    #### NORTH ####

    if direction == 'N':

        if left == True:
            maze[currentCellIndex].west = 'O'
        else:
            maze[currentCellIndex].west = 'W'

        if front == True:
            maze[currentCellIndex].north = 'O'
        else:
            maze[currentCellIndex].north = 'W'

        if right == True:
            maze[currentCellIndex].east = 'O'
        else:
            maze[currentCellIndex].east = 'W'

        maze[currentCellIndex].south = 'O'

    #### EAST #####

    if direction == 'E':

        if left == True:
            maze[currentCellIndex].south = 'O'
        else:
            maze[currentCellIndex].south = 'W'

        if front == True:
            maze[currentCellIndex].east = 'O'
        else:
            maze[currentCellIndex].east = 'W'

        if right == True:
            maze[currentCellIndex].north = 'O'
        else:
            maze[currentCellIndex].north = 'W'

        maze[currentCellIndex].west = 'O'

    #### SOUTH ####

    if direction == 'S':

        if left == True:
            maze[currentCellIndex].east = 'O'
        else:
            maze[currentCellIndex].east = 'W'

        if front == True:
            maze[currentCellIndex].south = 'O'
        else:
            maze[currentCellIndex].south = 'W'

        if right == True:
            maze[currentCellIndex].west = 'O'
        else:
            maze[currentCellIndex].west = 'W'

        maze[currentCellIndex].north = 'O'


    if leftIndex >= 0:
        maze[leftIndex].east = maze[currentCellIndex].west

    if upIndex >= 0:
        maze[upIndex].south = maze[currentCellIndex].north


#Updates the current Cell number ( 1 -16 )
def updateCell(direction, cell):
    global currentCell

    if direction == 'W':
        currentCell = cell -1
    elif direction =='N':
        currentCell = cell - 4
    elif direction == 'E':
        currentCell = cell + 1
    elif direction == 'S':
        currentCell = cell + 4

    maze[currentCell - 1].visited = True


################################################################################
########################### Camera Functions ###################################
################################################################################
def pauseMovement():
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

    fDistance = fSensor.get_distance()

    inchesDFront = fDistance * 0.0394

    if inchesDFront < 18:
        frontDist()

def checkCamera():
    global fps, prev, lRevolutions, rRevolutions, yellow, blue, green, pink
    camCount = 0

    # Reading in distance from front sensor and converting to inches
    fDistance = fSensor.get_distance()
    inchesDFront = fDistance * 0.0393700787      

    if inchesDFront < 18:
        frontDist()

    while camCount < 10:
        camCount += 1
        #print("checking camera feed") 

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
        mask_yellow = cv.inRange(frame_hsv, (lower_yellow), (upper_yellow))
        mask_pink = cv.inRange(frame_hsv, (lower_pink), (upper_pink))
        mask_green = cv.inRange(frame_hsv, (lower_green), (upper_green))
        mask_blue = cv.inRange(frame_hsv, (lower_blue), (upper_blue))

        # Setting the mask values for yellow, pink, green and blue
        # Run the SimpleBlobDetector on the mask.
        # The results are stored in a vector of 'KeyPoint' objects,
        # which describe the location and size of the blobs.
        keypoints_yellow = detector.detect(mask_yellow)
        keypoints_pink = detector.detect(mask_pink)
        keypoints_green = detector.detect(mask_green)
        keypoints_blue = detector.detect(mask_blue)
    
        # For each detected blob, draw a circle on the frame
        frame_with_keypoints_yellow = cv.drawKeypoints(frame, keypoints_yellow, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        frame_with_keypoints_pink = cv.drawKeypoints(frame, keypoints_pink, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        frame_with_keypoints_green = cv.drawKeypoints(frame, keypoints_green, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        frame_with_keypoints_blue = cv.drawKeypoints(frame, keypoints_blue, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
        # Write text onto the frame
        cv.putText(frame_with_keypoints_yellow, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        cv.putText(frame_with_keypoints_yellow, "{} blobs".format(len(keypoints_yellow)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        cv.putText(frame_with_keypoints_pink, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        cv.putText(frame_with_keypoints_pink, "{} blobs".format(len(keypoints_pink)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        cv.putText(frame_with_keypoints_green, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        cv.putText(frame_with_keypoints_green, "{} blobs".format(len(keypoints_green)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        cv.putText(frame_with_keypoints_blue, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
        cv.putText(frame_with_keypoints_blue, "{} blobs".format(len(keypoints_blue)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

        for keypoint in keypoints_yellow:
            print(" I see yellow!")
            # set the current cell as this colored cell
            yellow = currentCell

        for keypoint in keypoints_pink:
            print(" I see pink!")
            # set the current cell as this colored cell
            pink = currentCell

        for keypoint in keypoints_green:
            print(" I see green!")
            # set the current cell as this colored cell
            green = currentCell


        for keypoint in keypoints_blue:
            print(" I see blue!")
            # set the current cell as this colored cell
            blue = currentCell


        # Check for user input
        c = cv.waitKey(1)
        if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
            camera.stop()
            break

    lRevolutions = 1.1
    rRevolutions = 1.1


def turnToWall():
    global lRevolutions, rRevolutions

    fDistance = fSensor.get_distance()

    inchesDFront = fDistance * 0.0394

    if currentCell == 1:
        if direction == 'N':
            checkCamera()
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()
        elif direction == 'W':
            checkCamera()
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()

    elif currentCell == 2 or currentCell == 3:
        if direction == 'N':
            checkCamera()
        elif direction == 'E':
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()
        elif direction == 'W':
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()

    elif currentCell == 4:
        if direction == 'N':
            checkCamera()
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()
        elif direction == 'E':
            checkCamera()
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()

    elif currentCell == 5 or currentCell == 9:
        if direction == 'W':
            checkCamera()
        elif direction == 'N':
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()
        elif direction == 'S':
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()

    elif currentCell == 8 or currentCell == 12:
        if direction == 'E':
            checkCamera()
        elif direction == 'N':
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()
        elif direction == 'S':
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()

    elif currentCell == 13:
        if direction == 'S':
            checkCamera()
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()
        elif direction == 'W':
            checkCamera()
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()

    elif currentCell == 14 or currentCell == 15:
        if direction == 'S':
            checkCamera()
        elif direction == 'E':
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()
        elif direction == 'W':
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()

    elif currentCell == 16:
        if direction == 'S':
            checkCamera()
            leftPivot()
            pauseMovement()
            checkCamera()
            rightPivot()
            pauseMovement()
        elif direction == 'E':
            checkCamera()
            rightPivot()
            pauseMovement()
            checkCamera()
            leftPivot()
            pauseMovement()

    lRevolutions = 1.1
    rRevolutions = 1.1



################################################################################


def mapping():
    global sensorCount, leftWallOpen, frontWallOpen, rightWallOpen, newCell, inchesDLeft, inchesDFront, inchesDRight, direction, currentCell
    stop()

    currentCell = int(input("Please identify my starting cell (Numbered 1 - 16): "))
    direction = input("Please enter the robot's atarting orientation (N, S, E, W): ")

    # Waiting for user to enter the required key in order to start the movement
    startFlag = False
    selectCommand = ' '
    # Holds program until command value is entered
    while selectCommand != 's':
        selectCommand = input("Please enter \'s\' to begin robot movement: ")


    # Declaring a variable to keep track of front sensor big readings
    sensorCount = 0

    #Booleans to determine if walls are open?
    frontWallOpen = False
    leftWallOpen = False
    rightWallOpen = False
    newCell = True

    # Utilizes 3 flags one for each detectable wall to see if there is an opening or not
    # These Flags are leftWallOpen, rightWallOpen and frontWallOpen

    # Reading in from sensors
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    lDistance = lSensor.get_distance()

    # Transforming readings to inches
    inchesDFront = fDistance * 0.0393700787
    inchesDRight = rDistance * 0.0393700787
    inchesDLeft = lDistance * 0.0393700787

    #Constantly updates the flags while reading the sensors.
    if inchesDFront > 15:
        frontWallOpen = True

    if inchesDRight > 15:
        rightWallOpen = True

    if inchesDLeft > 15:
        leftWallOpen = True

    updateMaze(leftWallOpen, frontWallOpen, rightWallOpen, direction, currentCell)

    while True:
 
        # Reading in from sensors
        fDistance = fSensor.get_distance()
        rDistance = rSensor.get_distance()
        lDistance = lSensor.get_distance()

        # Transforming readings to inches
        inchesDFront = fDistance * 0.0393700787
        inchesDRight = rDistance * 0.0393700787
        inchesDLeft = lDistance * 0.0393700787

        #Constantly updates the flags while reading the sensors.
        if inchesDFront > 15:
            frontWallOpen = True

        if inchesDRight > 15:
            rightWallOpen = True

        if inchesDLeft > 15:
            leftWallOpen = True

        #Checking if the robot has already traveled the required distance
        distanceT = (8.20 * ((lRevolutions + rRevolutions) / 2))
    
        if distanceT > 9 and newCell:
            print("I have entered a new cell")
            newCell = False
            updateCell(direction, currentCell)

        if distanceT > 18:

            print("""***************************************
                """)
            print("Current Cell Number: ", currentCell)
            print("Current Orientation: ", direction)
            print("""
                ***************************************
                """)
            # Update and print the maze every time a new cell is entered
            updateMaze(leftWallOpen, frontWallOpen, rightWallOpen, direction, currentCell)

            if currentCell >= 1 and currentCell <= 16:
                turnToWall()

            printMaze(maze)
            print("""
                ***************************************""")
            whatDo(leftWallOpen, frontWallOpen, rightWallOpen)
            newCell = True

            distanceT = 0
            resetCounts()

        # The exit function for when all cells have been visited
        # for cell in maze:
        #     completionCounter = 0

        #     if cell.visited == True:
        #         completionCounter += 1
        #     else:
        #         completionCounter = 0

        #     if completionCounter == 16:


        frontWallOpen = False
        leftWallOpen = False
        rightWallOpen = False

        moveForward()

            

def loadMap():
    global yellow, blue, pink, green, maze



def mainMenu():
    global currentCell, direction

    #startFlag = False
    option = input("""
        ************************************************************************
        ************************* MAIN MENU ************************************
        ************************************************************************

        Please Select one of the following options...(1 -4)

        1. Change robot's current orientation and position
        2. Run the Mapping Algorithm
        3. Load a pre-built map
        4. Travel to a specific color tile

        Enter any other key to exit...

        """)
    if option == '1':
        currentCell = int(input("Please identify my starting cell (Numbered 1 - 16): "))
        direction = input("Please enter the robot's atarting orientation (N, S, E, W): ")

    elif option == '2':
        mapping()
    elif option == '3':
        loadMap()
    elif option == '4':
        #startFlag = True
        # goalColor variable stores which of the premapped grids to go to
        goalColor = input("""
            Please choose a colored cell to navigate to options are...
            Y for yellow
            P for pink
            G for green
            B for blue
            """)
    else:
        exit()

################################################################################
########################### MAZE INITALIZATION #################################
################################################################################


# Initialize the maze with a set of walls and visited cells
maze = [
    Cell('W','W','?','?', False), Cell('?','W','?','?', False), Cell('?','W','?','?', False), Cell('?','W','W','?', False),
    Cell('W','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','W','?', False),
    Cell('W','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','W','?', False),
    Cell('W','?','?','W', False), Cell('?','?','?','W', False), Cell('?','?','?','W', False), Cell('?','?','W','W', False)
]


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

mainMenu()