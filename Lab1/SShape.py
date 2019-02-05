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
        # Reset tick counts and print out speed corresponding to pwm values

        time.sleep(3)

        currentSpeeds = getSpeeds()
        currentLeftSpeeds = currentSpeeds[0]
        currentRightSpeeds = currentSpeeds[1]
        # write pwm and speed values for startVar to dictionary
        LWSpeed[round(startVar,2)] = round(currentLeftSpeeds,3)
        RWSpeed[round(startVar,2)] = round(currentRightSpeeds,3)

        print(LWSpeed)
        print(RWSpeed)
                   
        # Increment loop
        startVar = round(startVar + 0.01,2)

        
        time.sleep(2)
        if startVar == 1.70:
            break
        
        # Reset counts for next loop!
        resetCounts()
    

def setSpeedsRPS(rpsLeft, rpsRight):
    lPwmValue = LWSpeed[rpsLeft]
    rPwmValue = LWSpeed[rpsRight]
    pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
    
    

# Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):
    # Function sets speed of robot to move over a linear speed with a set angular velocity
    
    # converts input to inches/sec sets variables rpsLeft and rpsRight to 3 decimal places
    rpsLeft = float(ipsLeft / 8.20)
    rpsRight = float(ipsRight / 8.20)
    round(rpsLeft, 3)
    round(rpsRight, 3)


    #binary search to find correct value
	#value closests
	# value = min(d.items(), key = lambda kv : abs(kv[1] - target))[0]
	#value = min(LWSpeed.items(), key = lambda kv: abs(kv[1] - target))[0]
	i = 1.3;
    while i < 1.7 :
		if(rpsLeft > LWSpeed[i] && rpsLeft < LWSpeed[i+.01])
			lKey = i;
		i = i + .01;

	while i < 1.7 :
		if(rpsRight > RWSpeed[i] && rpsRight < RWSpeed[i+.01])
			rKey = i;
		i = i + .01;
	
	
	#get.RWSpeed[rKey]



    #merp a drep
    lPwmValue = LWSpeed[lKey]
    rPwmValue = RWSpeed[rKey]
    pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))


# Defines speeds in a clockwise rotation
def setSpeedsvw1(v, w):
	velocityLeft1 = float(( v + ( w * dAxis)))
	velocityRight1 = float(( v - ( w * dAxis)))

	setSpeedsIPS(velocityLeft1, velocityRight1)


# Defines speeds in a counter clockwise rotation
def setSpeedsvw2(v, w):
	velocityLeft2 = float(( v + ( w * dAxis)))
	velocityRight2 = float(( v + ( w * dAxis)))

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

print("calibrating")

calibrateSpeeds()
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

print("calibration complete")

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
print("entering the loop")

while cirFlag == True:
    
    print("inside the loop")
	# Set speeds for first circle
    setSpeedsvw1(linearVelocity, omega1)
    distanceT = ( 8.20 * ((lRevolutions + rRevolutions) / 2) )

    if (float(cirPath1) - float(distanceT)) <= 0.00:
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
        itsTime = time.time()
        print("first arc completed")
        print("Time taken to travel: ", itsTime - now)

selectCommmand = ' '

while selectCommand != 's':
	selectCommand = input("Please enter \'s\' to begin robot movement: ")
now = time.time()

