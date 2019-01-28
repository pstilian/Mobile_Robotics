#This program does a thing
# For full information on this project see the Git Repo at https://github.com/pstilian/Mobile_Robotics_Kinematics.git

import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO
import encoders as ENCODER
import servos as SERVO









#******************************* MAINLINE CODE *****************************************************
# This program makees the robot hold position until the select button is pressed
# After robot movement has been initiated it will move forward in a straight for an input number of inches (xInches)
# The robot will also complete this task in the input number of seconds (ySeconds)

# Reads input values for distance to travel and time of travel
xInches = input("Enter desired distance of travel in inches")
ySeconds = input("Enter the amount of time to travel the set distance in seconds")

# Set boolean value for good values to false and calculate the desired speed in inches per second (ips)
goodVal = False
ips = (float(xInches) / float(ySeconds))

# Set Maximum possible value for inches per second      NEED TO FIND MAX ROBOT SPEED
#maxSpeed = number???

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