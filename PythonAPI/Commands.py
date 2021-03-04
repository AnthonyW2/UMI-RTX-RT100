#21 December 2020
#20-12-20

#Interactive program to easily control the robot

import math

import serial
from RTX import *

DEBUG = False

#Robot = RTX("/dev/ttyUSB0")
#Robot = RTX("/dev/USB0")
Robot = RTX("/dev/ttyRTX0")

try:
  Robot.setup()
except:
  print("\033[94m"+"\nFirst run since powered on, attempting setup again...\n"+"\033[0m")
  Robot.reset()
  Robot.setup()

def z(v):
  # Z-axis
  s = 1 if v > 0 else -1
  if v*s > 400:
    #for a in range(math.floor(v*s/400)):
    #  Robot.move(ZED,400*s)
    #  Robot.wait()
    #Robot.move(ZED,(v%400)*s)
    print("Values over 400 may not work as expected")
  Robot.move(ZED,v)

def s(v):
  # Shoulder
  Robot.move(SHOULDER,v)

def e(v):
  # Elbow
  Robot.move(ELBOW,v)

def y(v):
  # Yaw
  Robot.move(YAW,v)

def w(v):
  # Wrist (unimplemented)
  print("Wrist is not yet supported")

def g(v):
  # Gripper (unimplemented)
  print("Gripper is not yet supported")

def setup():
  Robot.setup()

def stop():
  Robot.manual()
  Robot.setup()
  Robot.manual()
  Robot.setup()

while True:
  cmd = input("\033[3m"+"\033[94m"+"[UMI RTX]$"+"\033[0m"+" ")
  eval(cmd)
