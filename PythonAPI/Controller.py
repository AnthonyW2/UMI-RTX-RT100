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

while True:
  JOINT = None
  VAL = 0
  
  joint = ""
  while JOINT is None:
    joint = input("Joint to control (Z/S/E/Y/W/G, Q to quit): ").lower()
    if joint == "z":
      JOINT = ZED;
    elif joint == "s":
      JOINT = SHOULDER
    elif joint == "e":
      JOINT = ELBOW
    elif joint == "y":
      JOINT = YAW
    elif joint == "w":
      print("Wrist not yet supported")
    elif joint == "g":
      print("Gripper is currently experimental, and not yet supported")
    elif joint == "q":
      exit(0)
    else:
      print("Input a valid joint to move")
  
  value = ""
  while VAL == 0:
    value = input("Amount to move (C to cancel): ")
    if value.lower() == "c":
      JOINT = None
      VAL = 1
    else:
      try:
        VAL = int(value)
      except:
        print("Input valid movement amount (integer)")
  
  if JOINT is not None:
    s = 1 if VAL > 0 else -1
    if JOINT == ZED and VAL*s > 400:
      for a in range(math.floor(VAL*s/400)):
        Robot.move(ZED,400*s)
        Robot.wait()
      Robot.move(ZED,(VAL%400)*s)
    else:
      Robot.move(JOINT,VAL)
  
