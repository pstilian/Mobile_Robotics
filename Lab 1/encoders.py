# This program demonstrates usage of the digital encoders.
# After executing the program, manually spin the wheels and observe the output.
# See https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/ for more details.

import time
import RPi.GPIO as GPIO
import signal

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

#Used Values
lCount = 0
rCount = 0
TotalCount = (0,0)
lSpeed = 0
rSpeed = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()
currentTime = 0

# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
	global lCount, lRevolutions, lSpeed, currentTime
	lCount += 1
	lRevolutions = float(lCount / 32)
	currentTime = time.time() - startTime
	lSpeed = lRevolutions / currentTime
    #print("Left encoder ticked!")
    print ("Lticks: ", lCount)
    print ("LRevolutions: ", lRevolutions)
    print ("LTime: ", currentTime)
    print ("LSpeed: ", lSpeed)

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
	global rCount, rRevolutions, rSpeed, currentTime
	rCount += 1
	rRevolutions = float(rCount / 32)
	currentTime = time.time() -startTime
	lSpeed = rRevolutions / currentTime
    #print("Right encoder ticked!")
    print ("Rticks: ", rCount)
    print ("RRevolutions: ", rRevolutions)
    print ("RTime: ", currentTime)
    print ("RSpeed: ", rSpeed)

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    exit()

#Resets the tick count
def resetCounts():
	Lcounts = 0
	Rcounts = 0

#Returns the previous tick counts as a touple
def getCounts():
	return ( lCount, rCount)

#Returns instantious speeds for both left and right wheels as a touple
def getSpeeds():
	global lCount, rCount, currentTime, lSpeed, rSpeed
	currentTime = time.time() -startTime
	lSpeed = ( lCount / 32 ) / currentTime
	rSpeed = ( rCount / 32 ) / currentTime
	return ( lSpeed, rSpeed)

#Function that sets up and initializes the econders for the robot
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

# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)
initEncoders()
    
# Prevent the program from exiting by adding a looping delay.
while True:
    time.sleep(1)