# Mobile Robotics Spring 2019

### Lab 1 Kinematics

  The purpose of this lab was to program our robot to move forward a set distance in a requested amount of time.
This meant calculating the velocity of the Robot based on the requested parameters. We implemented this task with
the file Forward.py. Forward.py determines the necessary velocity of each wheel from a text file generated using 
the file Calibration.py. This text file was also used to plot out our pwm values compared to wheel rotation/sec on
our lab report. The final tasking was to have the robot drive in an S-Shape. This is done using the file SShape.py.
This program performs it's own calculations and stores the calibration numbers in a dictionary. Afterwards it 
prompts the user for the desired radius of each section of the S as well as the desired completion time.

### Lab 2 Wall Distance and Wall Following

  This lab utilizes the robots distance sensors to maintain a set distance from a wall as well as offer practice with
object detection. The two main programs we use are wallDistance.py and wallFollowing.py. The wall distance program will
make the robot seek a wall and stop itself at a distance of 5 inches. If it is moved closer or further away the robot 
will automatically go back to the 5 inch distance. The wall following program will track a wall to the right hand side
of the robot and move forward while keeping a distance of 5 inches from the wall. It can be easily modified to track a 
wall from the left side as well which will be implemented in Lab 3.


### Lab 3 Goal Tracking and Bug Algorithms

  This lab consists of three programs which each build off of each other. GoalFacing, Motion to Goal and BugAlgorithm. The Goal facing will prompt the robot to rotate slowly until it finds the desired "goal object" (In this case a red cylinder). At this point it will stop until either the robot or goal is moved. Motion To Goal runs the GoalFacing program and when it finds the goal object it approaches the object until it is 5 inches away. Finally the bug algorithm applies a Bug 0 algorithm to naviigate its way to a goal object avoiding hazards while also encapsulating the GoalFacing and Motion to Goal functions.

### Lab 4.1 Localization and Mapping

  This lab uses many of our previous functions to create two programs. Navigation.py and Mapping.py. Navigation.py will randomly navigate through amaze using only 90 and 180 degree turns. It will continue to wander randomly through the maze until the program is terminated. The second preogram is Mapping.py which also wanders through a maze made up of a 4 x 4 grid of 18 inch squares. While navigating the maze the robot will use it distance sensors to map out the walls of the maze and print them in the terminal window.
  
 ### Lab 4.2 Path Planning
 
 Working....

