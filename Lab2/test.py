
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
lCount = 0
rCount = 0
#TotalCount = (0, 0)
lSpeed = 0
rSpeed = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()
currentTime = 0
dAxis = float(3.95)
LWSpeed = {}
RWSpeed = {}

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
      lSensor.stop_ranging()
      fSensor.stop_ranging()
      rSensor.stop_ranging()

      exit()

def readCalibration():
    l = open("LeftSpeedCalibration.txt", "r")
    for line in l:
        currentLine = line.split(" ")
        LWSpeed[currentLine[0]] = currentLine[1]
    r = open("RightSpeedCalibration.txt", "r")
    for line in r:
        currentLine = line.split(" ")
        RWSpeed[currentLine[0]] = currentLine[1]

    print(LWSpeed)

    print(RWSpeed)

    l.close()
    r.close()

def setDifference(speed):
    diff = speed - 1.5
    return 1.5 - diff
        
# Sets speed of motors in Inches per econd
def setSpeedsIPS(ipsLeft, ipsRight):
      # Converting inches per second into revolutions per second
      rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
      rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

      if rpsLeft < 0:
            rpsLeft = 0 - rpsLeft
      if rpsRight < 0:
            rpsRight = 0 - rpsRight

      # Calculating pwm values from the respective dictionaries
      lPwmValue = float(LWSpeed[rpsLeft])
      rPwmValue = float(RWSpeed[rpsRight])

      if ipsLeft < 0 and ipsRight < 0:
            # Setting appropiate speeds to the servos when going forwards
            pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))
      elif ipsLeft >= 0 and ipsRight >= 0:
            # Setting apporpiate speeds to the servos when going backwards
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
      elif ipsLeft >= 0 and ipsRight < 0:
            # Setting appropriate speedsto the servos while making a turn
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
            pwm.set_pwm(LSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))


###############################
def saturationFunction(ips):
    controlSignal = ips
    if controlSignal > 0.5:
        controlSignal = 0.5
    elif controlSignal < -0.5:
        controlSignal = -0.5
    return controlSignal


def leftTurn():#####################
      setSpeedsIPS(1.3,-1.3)
      time.sleep(3)
      setSpeedsIPS(0,0)

def rightTurn():###################
      setSpeedsIPS(-1.3,1.3)
      time.sleep(3)
      setSpeedsIPS(0,0)

def setSpeedsvw(v, w):
      velocityLeft1 = float(( v + ( w * dAxis)))
      velocityRight1 = float(( v - ( w * dAxis)))

      setSpeedsIPS(-velocityLeft1, -velocityRight1)

desiredDistance = 5.0
kpValue = 0.9

# Attach the Ctrl+C signal interrupt and initialize encoders
signal.signal(signal.SIGINT, ctrlC)

# Initialized servos to zero movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

#--------------------------------------MAINLINE CODE----------------------------------------------------
startFlag = False
selectCommand = ' '

readCalibration()

while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag =True

# Set Linear Speed to 5 inches per second
linearSpeed = 5

# Setting a counter to keep track of BIG front sensor reading
sensorCount = 0

while True:

      # Reads Distance From Sensors
      fDistance = fSensor.get_distance()
      print(fDistance)
      rDistance = rSensor.get_distance()
      print(rDistance)
      #lDistance = lSensor.get_distance()

      # Converts readings from centimeters to inches
      fInchDistance = fDistance * 0.0394
      rInchDistance = rDistance * 0.0394
      # 0.394 is the conversion rate from millimeters to inches Determining error amount

      print("FRONT DISTANCE : ", fInchDistance)
      print("RIGHT DISTANCE : ", rInchDistance)

      # fError is the calculated respective error value aka the e(t) value
      fError = desiredDistance - fInchDistance
      rError = desiredDistance - rInchDistance

      print("FRONT ERROR : ", fError)
      print("RIGHT ERROR : ", rError)

      # Control Signal aka u(t)  = Kp * e(t)
      fControlSignal = kpValue * fError
      rControlSignal = kpValue * rError

      # Calculating new control signal value by running control signal through saturation function
      fNewSignal = saturationFunction(fControlSignal)
      rNewSignal = saturationFunction(rControlSignal)

      # Setting speed of the robot.
      setSpeedsvw(linearSpeed, -rNewSignal)

      # Checks for obstacle to the front if 5 consecutive reading are made robot makes a left turn
      if fInchDistance < 5.0:

            sensorCount += 1

            if sensorCount > 4:
                  leftTurn()

      else:
            sensorCount = 0