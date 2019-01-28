# This program demonstrates usage of the servos.
# Keep the robot in a safe location before running this program,
# as it will immediately begin moving.
# See https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/ for more details.
# For full information on this project see the Git Repo at https://github.com/pstilian/Mobile_Robotics_Kinematics.git

import time
import Adafruit_PCA9685
import signal
import math
import encoders as ENCODER

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()

#This function changes the values from 1.5 - 1.7 to 1.5-1.3 basically outputting inverse pwm values 
def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff

#This function creates a calibration map comparing the servo input to output based on microseconds
#Measures the speeds of each wheel based on different input values 
def calibrateSpeeds():

    #Initial start pwm value is at complete stop
    startVar = 1.5

    #Loop runs starting from full stop to full speed in the forward direction
    #LSERVO runs from 1.5 to 1.3 (with 1.3 being max forward pwm)
    #RSERVO runs from 1.5 to 1.7 (with 1.7 being max forward pwm)
    while startVar < 1.71: 
        pwm.set_pwm(LSERVO, 0, math.floor( setDifference(startVar) / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(startVar / 20 * 4096));
        time.sleep(1)

        #Print out speed corresponding to pwm values
        print (startVar, ENCODER.getSpeeds())
        time.sleep(1)

        #Increment loop
        startVar += 0.01

        #Reset counts for next loop!
        ENCODER.resetCounts()

# How do we get the above code to write into a map?? ***********************************

#Sets speed of motors in revolutions per second
def setSpeedsRPS(rpsLeft, rpsRight):
    # needs to convert from RPS to PWM values

#Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):

#Function sets speed of robot to move over a linear speed with a set angular velocity
# v = inches per second         w = angular velocity
# positive w values spin counterclockwise       negative w values spin clockwise
def setSpeedsvw(v, w):






#******************************* MAINLINE CODE *****************************************************
#Currently rotates the robot in one direction, pauses for 4 seconds then roates in the other direction



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

while True:
    # Write a maximum value of 1.7 for each servo.
    # Since the servos are oriented in opposite directions,
    # the robot will end up spinning in one direction.
    # Values between 1.3 and 1.7 should be used.
    pwm.set_pwm(LSERVO, 0, math.floor(1.3 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.7 / 20 * 4096));
    time.sleep(4)
    
    # Write a minimum value of 1.4 for each servo.
    # The robot will end up spinning in the other direction.
    pwm.set_pwm(LSERVO, 0, math.floor(1.7 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.3 / 20 * 4096));
    time.sleep(4)