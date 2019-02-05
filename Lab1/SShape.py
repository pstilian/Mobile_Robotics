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
#keys = range(41)
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
    Lcounts = 0
    Rcounts = 0

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

# This function creates a calibration map comparing the servo input to output based on microseconds
# Measures the speeds of each wheel based on different input values
def calibrateSpeeds():
    # Initial start pwm value is at complete stop
    startVar = 1.3
    endVar = 1.71

    while startVar <= endVar:
        pwm.set_pwm(LSERVO, 0, math.floor(startVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(startVar / 20 * 4096))
        time.sleep(1)
        # Reset tick counts and print out speed corresponding to pwm values
        print(startVar, getSpeeds())

        time.sleep(4)

        currentSpeeds = getSpeeds()
        currentLeftSpeeds = currentSpeeds[0]
        currentRightSpeeds = currentSpeeds[1]
        # write pwm and speed values for startVar to dictionary
        LWSpeed[startVar] = currentLeftSpeeds
        RWSpeed[startVar] = currentRightSpeeds
        
        # Increment loop
        startVar += 0.01
        
        time.sleep(2)
        
        # Reset counts for next loop!
        resetCounts()


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
        left = round(left + 0.01, 2)#check remove

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
        right = round(right + 0.01, 2)#remove
        
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
    setSpeedsRPS(rpsLeft, rpsRight)


# Defines speeds in a clockwise rotation
def setSpeedsvw1(v, w):
	velocityLeft1 = ( v + ( w * dAxis))
	velocityRight1 = ( v - ( w * dAxis))

	setSpeedsIPS(velocityLeft1, velocityRight1)


# Defines speeds in a counter clockwise rotation
def setSpeedsvw2(v, w):
	velocityLeft2 = ( v + ( w * dAxis))
	velocityRight2 = ( v + ( w * dAxis))

	setSpeedsIPS(velocityLeft2, velocityRight2)



#******************************* MAINLINE CODE *****************************************************
# This program makees the robot move in two semicircles (one is inverted) which forms an S shape
# The first semicircle will be completed in a clockwise rotation and the second will be counterclockwise
# The user will be prompted for the Radius (cirRadius1, cirRadius2) and the time (cirTime) to complete the S-Shape

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
initEncoders()
signal.signal(signal.SIGINT, ctrlC)

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)



# Initalizing variables for radius and time
cirRadius1 = 0
cirRadius2 = 0
cirTime = 0
# Reads input values circle radius' and time for completion
cirRadius1 = input("Enter the desired radius for circle 1 (r1) in inches: ")
cirRadius2 = input("Enter the desired radius for circle 2 (r2) in inches: ")
cirTime = input("Enter the amount of time to travel the set distance in seconds: ")

# Compute the variables needed to conduct kinematic equations for robot movement
dAxis = float(3.95) #-------------------------------------need to measure D axis-----------
cirPath1 = float(3.14) * float(cirRadius1)
cirPath2 = float(3.14) * float(cirRadius2)
linearVelocity = ( float(cirPath1) + float(cirPath2) ) / float(cirTime)
omega1 = float(linearVelocity) / float(cirRadius1)
omega2 = float(linearVelocity) / float(cirRadius2)

# Initialize flag to track first circle movement
cirFlag = True

while cirFlag == True:
	# Set speeds for first circle
	setSpeedsvw1(linearVelocity, omega1)
	distanceT = ( 8.20 * ((lRevolutions + rRevolutions) / 2) )

	if (float(cirPath1) - float(distanceT)) <= 0.00:
    	pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
    	pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
    	itsTime = time.time()
    	print("first arc completed")
    	print("Time taken to travel: ", itsTime - now)

select commmand = ' '

while selectCommand != 's':
	selectCommand = input("Please enter \'s\' to begin robot movement: ")
now = time.time()

