# Keep the robot in a safe location before running this program,
# See https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/ for more details.
# For full information on this project see the Git Repo at https://github.com/pstilian/Mobile_Robotics_Kinematics.git
import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO
import signal
import decimal

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second.
LSERVO = 0
RSERVO = 1

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

# innitialize map Rotation per second speed
LWSpeed = {}
RWSpeed = {}
leftflag = False
rightflag = False





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

# Resets the tick count
def resetCounts():
    global Lcount, rCount, startTime
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
        
# Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):
    # Function sets speed of robot to move over a linear speed with a set angular velocity
    # v = inches per second         w = angular velocity
    # positive w values spin counterclockwise       negative w values spin clockwise
    rpsLeft = round(float(ipsLeft / 8.20), 2)
    rpsRight = round(float(ipsRight / 8.20), 2)
    
    #Calculates the PWM values by using RPS



#******************************* MAINLINE CODE *****************************************************
# This program makees the robot hold position until the select button is pressed
# After robot movement has been initiated it will move forward in a straight for an input number of inches (xInches)
# The robot will also complete this task in the input number of seconds (ySeconds)

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

signal.signal(signal.SIGINT, ctrlC)
initEncoders()

# Reads input values for distance to travel and time of travel
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

selectCommand = ' '

while selectCommand != 's':
	selectCommand = input("Please enter \'s\' to begin robot movement:")
now = time.time()
while True:
    setSpeedsIPS(ips, ips)
    

    distanceT = ( 8.20 * ((lRevolutions + rRevolutions) / 2) )
    if (float(xInches) - float(distanceT)) <= 0.00:
    	pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
    	pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
    	itsTime = time.time()
    	print("Beep! Boop! Bop! I have traveled ", round(distanceT,2), " inches! I am AMAZING!!")
    	print("Time taken to travel: ", itsTime - now)
    	exit()

    #print("Number of Revolutions : ", (lRevolutions + rRevolutions) / 2)
    #print("Distance Traveled     : ", distanceT)