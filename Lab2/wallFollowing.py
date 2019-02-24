
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
LWSpeed = {
            0.00: 1.50, 0.01: 1.505, 0.02: 1.505, 0.03: 1.51, 0.04: 1.51,
            0.05: 1.51, 0.06: 1.51, 0.07: 1.51, 0.08: 1.515, 0.09: 1.515,
            0.10: 1.515, 0.11: 1.515, 0.12: 1.515, 0.13: 1.52, 0.14: 1.52,
            0.15: 1.52, 0.16: 1.52, 0.17: 1.52, 0.18: 1.525, 0.19: 1.525,
            0.20: 1.525, 0.21: 1.525, 0.22: 1.525, 0.23: 1.525, 0.24: 1.53,
            0.25: 1.53, 0.26: 1.53, 0.27: 1.53, 0.28: 1.53, 0.29: 1.535,
            0.30: 1.535, 0.31: 1.535, 0.32: 1.535, 0.33: 1.54, 0.34: 1.54,
            0.35: 1.54, 0.36: 1.54, 0.37: 1.54, 0.38: 1.545, 0.39: 1.545,
            0.40: 1.545, 0.41: 1.545, 0.42: 1.545, 0.43: 1.55, 0.44: 1.55,
            0.45: 1.55, 0.46: 1.55, 0.47: 1.56, 0.48: 1.555, 0.49: 1.555,
            0.50: 1.555, 0.51: 1.555, 0.52: 1.555, 0.53: 1.56, 0.54: 1.56,
            0.55: 1.56, 0.56: 1.56, 0.57: 1.56, 0.58: 1.565, 0.59: 1.565,
            0.60: 1.565, 0.61: 1.565, 0.62: 1.565, 0.63: 1.57, 0.64: 1.57,
            0.65: 1.57, 0.66: 1.57, 0.67: 1.57, 0.68: 1.575, 0.69: 1.575,
            0.70: 1.58, 0.71: 1.58, 0.72: 1.585, 0.73: 1.585, 0.74: 1.59,
            0.75: 1.59, 0.76: 1.59, 0.77: 1.60, 0.78: 1.60, 0.79: 1.61,
            0.80: 1.61, 0.81: 1.61, 0.82: 1.61, 0.83: 1.62, 0.84: 1.63,
            0.85: 1.64, 0.86: 1.65, 0.87: 1.70
            }
RWSpeed = {
            0.00: 1.50, 0.01: 1.50, 0.02: 1.50, 0.03: 1.505, 0.04: 1.505,
            0.05: 1.505, 0.06: 1.505, 0.07: 1.505, 0.08: 1.51, 0.09: 1.51,
            0.10: 1.51, 0.11: 1.51, 0.12: 1.51, 0.13: 1.515, 0.14: 1.515,
            0.15: 1.515, 0.16: 1.515, 0.17: 1.515, 0.18: 1.52, 0.19: 1.52,
            0.20: 1.52, 0.21: 1.52, 0.22: 1.52, 0.23: 1.525, 0.24: 1.525,
            0.25: 1.525, 0.26: 1.525, 0.27: 1.52, 0.28: 1.53, 0.29: 1.53,
            0.30: 1.53, 0.31: 1.53, 0.32: 1.53, 0.33: 1.535, 0.34: 1.535,
            0.35: 1.535, 0.36: 1.535, 0.37: 1.54, 0.38: 1.54, 0.39: 1.54,
            0.40: 1.54, 0.41: 1.54, 0.42: 1.54, 0.43: 1.5425, 0.44: 1.5425,
            0.45: 1.5425, 0.46: 1.545, 0.47: 1.545, 0.48: 1.5475, 0.49: 1.5475,
            0.50: 1.5475, 0.51: 1.5475, 0.52: 1.55, 0.53: 1.55, 0.54: 1.55,
            0.55: 1.55, 0.56: 1.555, 0.57: 1.555, 0.58: 1.555, 0.59: 1.56,
            0.60: 1.56, 0.61: 1.56, 0.62: 1.56, 0.63: 1.56, 0.64: 1.565,
            0.65: 1.565, 0.66: 1.565, 0.67: 1.565, 0.68: 1.57, 0.69: 1.57,
            0.70: 1.57, 0.71: 1.57, 0.72: 1.575, 0.73: 1.575, 0.74: 1.58,
            0.75: 1.58, 0.76: 1.58, 0.77: 1.59, 0.78: 1.59, 0.79: 1.60,
            0.80: 1.60, 0.81: 1.60, 0.82: 1.61, 0.83: 1.62, 0.84: 1.62,
            0.85: 1.62, 0.86: 1.65, 0.87: 1.70
            }

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

    if ipsLeft < 0 or ipsRight < 0:
        # Setting appropiate speeds to the servos when going forwards
        pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(setDifference(rPwmValue) / 20 * 4096))
    elif ipsLeft >= 0 or ipsRight >= 0:
        # Setting apporpiate speeds to the servos when going backwards
        pwm.set_pwm(LSERVO, 0, math.floor(setDifference(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))


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
      setSpeedsgiIPS(-1.3,1.3)
      time.sleep(3)
      setSpeedsIPS(0,0)

def setSpeedsvw(v, w):
      velocityLeft1 = float(( v + ( w * dAxis)))
      velocityRight1 = float(( v - ( w * dAxis)))

      setSpeedsIPS(velocityLeft1, velocityRight1)

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

while selectCommand != 's':
      selectCommand = input("Please enter \'s\' to begin robot movement: ")

startFlag =True

# Set Linear Speed to 5 inches per second
linearSpeed = 5

while startFlag == True:

      # Reads Distance From Sensors
      fDistance = fSensor.get_distance()
      print(fDistance)
      rDistance = rSensor.get_distance()
      print(rDistance)
      #lDistance = lSensor.get_distance()

      # Converts readings from centimeters to inches
      fInchDistance = fDistance * 0.3937
      rInchDistance = rDistance * 0.3937
      # 0.394 is the conversion rate from cm to inches Determining error amount

      print("FRONT DISTANCE : ", fInchDistance)
      print("RIGHT DISTANCE : ", rInchDistance)

      # fError is the calculated respective error value aka the e(t) value
      fError = desiredDistance - fInchDistance
      rError = desiredDistance - rInchDistance

      # Control Signal aka u(t)  = Kp * e(t)
      fControlSignal = kpValue * fError
      rControlSignal = kpValue * rError

      # Calculating new control signal value by running control signal through saturation function
      fNewSignal = saturationFunction(fControlSignal)
      rNewSignal = saturationFunction(rControlSignal)

      setSpeedsIPS(fNewSignal, fNewSignal)

      if rInchDistance > 5:
            print("RIGHT TURN")
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            time.sleep(0.5)
            rightTurn()
            time.sleep(0.5)

      if fInchDistance > 5.0:
            setSpeedsIPS(fNewSignal, rNewSignal)
            if rError > 0 and rError < 1:
                  print("STRAIGHT AHEAD")
                  setSpeedsvw(rNewSignalr,0)

            elif rError < 0:
                  print("PIVOT RIGHT")
                  arcpath = float(3.14) * float(40)
                  linearSpeed = float(arcpath) / 5        #might need modification
                  omega = linearSpeed / 40
                  setSpeedsvw(4, omega)

            elif rError > 1:
                  print("PIVOT LEFT")
                  arcpath = float(3.14) * float(40)
                  linearSpeed = float(arcpath) / 5        #might need modification
                  omega = linearSpeed / 40
                  setSpeedsvw(4, -omega)

      elif fInchDistance < 5.0:
            print("LEFT TURN")
            pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
            time.sleep(0.5)
            leftTurn()
            time.sleep(0.5)
