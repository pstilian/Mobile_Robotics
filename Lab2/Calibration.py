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

LWSpeed = {}
RWSpeed = {}

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
    # open text files for reading
    l = open("LeftSpeedCalibration.txt", "w+")
    r = open("RightSpeedCalibration.txt", "w+")
    # Initial start pwm value is at complete stop
    startVar = 1.3
    endVar = 1.71

    while startVar <= endVar:
        pwm.set_pwm(LSERVO, 0, math.floor(startVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(startVar / 20 * 4096))
        time.sleep(1)
        # Reset tick counts and print out speed corresponding to pwm values
        print(startVar, getSpeeds())

        time.sleep(3)

        currentSpeeds = getSpeeds()
        currentLeftSpeeds = currentSpeeds[0]
        currentRightSpeeds = currentSpeeds[1]
        # write pwm and speed values for startVar to file
        l.write(str(currentLeftSpeeds) + " " + str(startVar) + "\n")
        r.write(str(currentRightSpeeds) + " " + str(startVar) + "\n")
        LWSpeed[setDifference(startVar)] = currentLeftSpeeds
        RWSpeed[setDifference(startVar)] = currentRightSpeeds
        
        # Increment loop
        startVar += 0.01
        
        time.sleep(1)
        
        # Reset counts for next loop!
        resetCounts()


    l.close()
    r.close()


#---------------------------------------MAINLINE TEXT------------------------------------------------------------
# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library,
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

signal.signal(signal.SIGINT, ctrlC)
initEncoders()
calibrateSpeeds()

# Prevent the program from exiting by adding a looping delay.
while True:
    time.sleep(1)
