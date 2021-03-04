#8 April 2020
#20-4-8

#This is an example program to demonstrate how to use the RTX API

import serial
from RTX import *



WRIST1, WRIST2, ELBOW, SHOULDER, ZED, YAW, GRIPPER = range(7)

RobotObject = RTX("/dev/ttyUSB0")

#This will amke the robot move all of it's joints to their maximum positions, then back to minimum
#RobotObject.soakTest()

#Send the robot to its home position
RobotObject.home()

# print("Testing seperate joint movement")
# print()
# Move the arm down
# RobotObject.move(ZED,300)

#Testing the robot a different way
# RobotObject.joycontrol()

RobotObject.manual()

def movementShowcase():
    r = RTX()
    r.soakTest()
    r.move(ZED, 1000)
