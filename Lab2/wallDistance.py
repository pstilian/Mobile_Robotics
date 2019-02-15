
import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import decimal
import math
import Adafruit_PCA9685

# variables
leftflag = False
rightflag = False
dAxis = 3.95
lCount = 0
rCount = 0
TotalCount = (0, 0)
lSpeed = 0
rSpeed = 0
lRevolutions = 0
rRevolutions = 0
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

def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff

# Returns instantious speeds for both left and right wheels as a touple
def getSpeeds():
    global lCount, rCount, currentTime, lSpeed, rSpeed
    currentTime = time.time() - startTime
    lSpeed = (lCount / 32) / currentTime
    rSpeed = (rCount / 32) / currentTime
    return (lSpeed, rSpeed)

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
    setSpeedsRPS(rpsLeft, rpsRight)

# Defines speeds in a clockwise rotation
def setSpeedsvw(v, w):
	velocityLeft = ( v + ( w * dAxis))
	velocityRight = ( v - ( w * dAxis))

	setSpeedsIPS(velocityLeft, velocityRight)


#--------------------------------------MAINLINE CODE----------------------------------------------------
desiredDistance = 5.0
#kpValue = 0.9

# Initialized servos to zero movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(2)

# Command to begin running the program
startFlag = False
selectCommand = input("Please enter \'s\' to begin robot movement: ")
if selectCommand == "s":
	startFlag = True
else:
	print("Invalid Command Exiting the program")
	exit()


# While loop that monitors speed vs distance
while True:
    # Reads Distance From Sensor
    fDistance = fSensor.get_distance()
    pwm.set_pwm(LSERVO, 0, math.floor(1.6 / 20 * 4096))
    pwm.set_pwm(LSERVO, 0, math.floor(1.4 / 20 * 4096))

    # Converts readings from centimeters to inches
    inchDistance = fDistance * 0.394
    # 0.394 is the conversion rate from cm to inches Determining error amount
    # fError = desiredDistance - inchDistance
    
    if inchDistance <= desiredDistance:
       pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
       pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
       lSensor.stop_ranging()
       fSensor.stop_ranging()
       rSensor.stop_ranging()
       exit()

