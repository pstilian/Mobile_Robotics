# Mobile Robotics Spring 2019

### Lab 1 Kinematics

The purpose of this lab was to program our robot to move forward a set distance in a requested amount of time.
This meant calculating the velocity of the Robot based on the requested parameters. We implemented this task with
the file Forward.py. Forward.py determines the necessary velocity of each wheel from a text file generated using 
the file Calibration.py. This text file was also used to plot out our pwm values compared to wheel rotation/sec on
our lab report. The final tasking was to have the robot drive in an S-Shape. This is done using the file SShape.py.
This program performs it's own calculations and stores the calibration numbers in a dictionary. Afterwards it 
prompts the user for the desired radius of each section of the S as well as the desired completion time.

### Lab 2 Working
