
import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import decimal
import math
import Adafruit_PCA9685



# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second.
LSERVO = 0
RSERVO = 1

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

# Used Values
lCount = 0
rCount = 0
TotalCount = (0, 0)
lSpeed = 0
rSpeed = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()
currentTime = 0
leftflag = False
rightflag = False



# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")

    GPIO.cleanup()
    # Write an initial value of 1.5, which keeps the servos stopped.
    # Due to how servos work, and the design of the Adafruit library,
    # The value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

    exit()

def checkValidSpeed():
    # Reads input values for distance to travel and time of travel
    print("How fast would you like to go in inches per second?")
    xInches = input("Enter desired distance of travel in inches: ")
    ySeconds = input("Enter the amount of time to travel the set distance in seconds: ")

    # Set boolean value for good values to false and calculate the desired speed in inches per second (ips)
    goodVal = False
    ips = (float(xInches) / float(ySeconds))

    distanceT = 0
    # Set Maximum possible value for inches per second
    maxSpeed = 6.5

    #This loop checks to see if input values for xInches and ySeconds are valid for this project
    while goodVal != True:
	    if ips > maxSpeed:
		    print("Input values cannot be used robot cannot achieve desired speed")
		    xInches = input("Enter desired distance of travel in inches")
		    ySeconds = input("Enter the amount of time to travel the set distance in seconds")
		    ips = (float(xInches) / float(ySeconds))
	    elif ips < 0:
		    print("These values cause robot to go in reverse. Only forward movement is allowed")
		    xInches = input("Enter desired distance of travel in inches")
		    ySeconds = input("Enter the amount of time to travel the set distance in seconds")
		    ips = (float(xInches) / float(ySeconds))
	    else:
		    goodVal = True

# Resets the tick count
def resetCounts():
    rCount = 0
    lCount = 0

# Returns the previous tick counts as a touple
def getCounts():
    return (lCount, rCount)

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

def setSpeedsRPS(rpsLeft, rpsRight):
    # needs to convert from RPS to PWM values
    global leftflag, rightflag
    # Calculating pwm values from the respective dictionaries
    left = round(float(rpsLeft), 2)#.55
    right = round(float(rpsRight), 2)#.55
    leftflag = False
    rightflag = False


    # Loop compares rpsLeft values with rps values from calibration document
    while leftflag != True:

        l = open("LeftSpeedCalibration.txt", "r")
        for line in l:
            currentLine = line.split(" ")
            rpsValue = round(float(currentLine[0]), 2)
            pwmValue = round(float(currentLine[1]), 2)

            if left == rpsValue:
                lPwmValue = pwmValue
                print("LPWMVALUE: ", lPwmValue)
                leftflag = True
                break
            elif left > 0.78:
                lPwmValue = 0
                leftflag = False
                break
        l.close()
        left = round(left + 0.01, 2)

    # Loop compares rpsRight values with rps values from calibration document
    while rightflag != True:

        r = open("RightSpeedCalibration.txt", "r")
        for line in r:
            currentLine = line.split(" ")
            rpsValue = round(float(currentLine[0]), 2)
            pwmValue = round(float(currentLine[1]), 2)

            if right == rpsValue:
                rPwmValue = pwmValue
                print("RPWMVALUE: ", rPwmValue)
                rightflag = True
                break
            elif right > 0.80:
                rPwmValue = 0
                rightflag = False
                break
        r.close()
        right = round(right + 0.01, 2)

    # If both right and left values are found then statement sets speeds to desired imputs
    if rightflag == True and leftflag == True:
        # Setting appropiate speeds to the servos
        pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))  
    
def setSpeedsIPS(ipsLeft, ipsRight):
    # Function sets speed of robot to move over a linear speed with a set angular velocity
    # v = inches per second         w = angular velocity
    # positive w values spin counterclockwise       negative w values spin clockwise
    rpsLeft = round(float(ipsLeft / 8.20), 2)
    rpsRight = round(float(ipsRight / 8.20), 2)
    #Calculates the PWM values by using RPS
    setSpeedsRPS(rpsLeft, rpsRight)



#******************************* MAINLINE CODE *****************************************************
# This program makees the robot move straight for 220 cm taking measurements
# every 20 cm from the left, right, and forward sensor, once measurements
# have been take we can then write them to a new file to create a scatter plot.

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library,
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

initEncoders()

# Takes input from user for distance and time to travel while also checking
# for bad input.
# checkValidSpeed()

# Open file to record data.
sensorOutput = open("LFRDistance.txt", "w+")

#set the distance to 0 prior to movement, set distanceInc to 7.87 due to
# 20 cm = 7.87, and stopDistance = 86.61 because 220 cm is the asked  
# distance to travel and 220 cm  = 86.61 inches.
distanceIn = 7.87402
distanceTraveled = 0
stopDistance = 86.6142

moveFlag = True
count = 0
# This for loop will step through and make the robot run in a straight line
# taking distance measurements every 7.87 inches from the left right and top
sensorOutput.write("***** Front Distance ******" "\n")
while moveFlag != False:
    # Get a measurement from each sensor
    lDistance = lSensor.get_distance()
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    distanceT = ( 8.20 * ((lRevolutions + rRevolutions) / 2) )

    pwm.set_pwm(LSERVO, 0, math.floor(1.6 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.4 / 20 * 4096));
    sensorOutput.write(str(fDistance) + "\n")
    if (distanceIn - distanceT) <= 0:
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

        # Print each measurement
        #sensorOutput.write( "Left Distance: " +  str(lDistance) + ", Right Distance: " + str(rDistance) + ", Forward Distance: "  + str(fDistance) +  "\n")
        print("Left: {}\tFront: {}\tRight: {}".format(lDistance, fDistance, rDistance))
        lRevolutions = 0
        rRevolutions = 0
        count += 1

    if count == 11:
        moveFlag = False



# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()