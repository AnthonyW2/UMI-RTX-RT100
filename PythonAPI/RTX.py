#31 March 2020
#20-3-31

"""
API made by Anthony Wilson
Held under GNU software license (GPLv3) - Modification & distribution are freely allowed, as long as the code remains is open-source

Depending on your configuration, this script may have to be run as root to communicate over the serial port

This API is heavily derived from Barnaby Shearer's Github repo, which can be found here: https://github.com/BarnabyShearer/rt100

I have listed all sources I used in the README - which can be found in the github repository here: https://github.com/Anthony-Wilson-Programming/UMI-RTX-RT100
"""

#If using on operating system other than Linux: Make sure to pass the RTX class a port name, otherwise the default (/dev/ttyUSB0) will return an error

#Currently all controls are relative, because absolute mode is not working

# Note to self:
# To track I/O, use the following command and supply the device /dev/ttyRTX0 to the python RTX object:
# sudo interceptty /dev/ttyUSB0 /dev/ttyRTX0

DEBUG = True

import copy

import time
import struct
import serial
from serial.tools import list_ports
DELAY = 0.01

STOP_POWERED, FORWARDS, BACKWARDS, STOP_UNPOWERED = range(4)

DEAD_STOP, RAMP_STOP, FREE_STOP, FREE_OFF = range(4)

CP_ERROR, CURRENT_POSITION, ERROR_LIMIT, NEW_POSITION, \
    SPEED, KP, KI, KD, DEAD_BAND, OFFSET, MAX_FORCE, CURRENT_FORCE, \
    ACCELERATION_TIME, USER_RAM, USER_IO, ACTUAL_POSITION = range(16)

#UNPOWER_ALL = [STOP_UNPOWERED,
#    STOP_UNPOWERED,
#    STOP_UNPOWERED,
#    STOP_UNPOWERED,
#    STOP_UNPOWERED,
#    STOP_UNPOWERED,
#    STOP_UNPOWERED
#]

CTRLS = [#IP, CTRL, Min, Max
    [0, 2, -4000, 4000],  # WRIST1      (Wrist Motor 1)
    [0, 3, -4000, 4000],  # WRIST2      (Wrist Motor 2)
    [1, 0, -2630, 2206],  # ELBOW       (Middle joint)
    [1, 1, -2630, 2630],  # SHOULDER    (First & largest joint - connects directly to main vertical rail)
    [1, 2, -3554, 0],     # ZED         (Up/Down)
    [1, 3, -1080, 1080],  # YAW         (The last joint - this connects the gripper & wrist to the rest of the arm)
    [1, 4, -30, 1200],    # GRIPPER     (Gripper on the end of the robot)
]

WRIST1, WRIST2, ELBOW, SHOULDER, ZED, YAW, GRIPPER = range(7)

class RTX:
    #Initialise the RTX object & establish serial communication
    def __init__(self, destPort="/dev/ttyUSB0", type="RT100"):
        self.ser = serial.Serial(
            port=destPort,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False
        )
        
        #Prepare for serial communication
        self.ser.write(bytearray([0x0]))
        
        time.sleep(DELAY)
        
        while self.ser.in_waiting > 0:
            print(ord(self.ser.read()))
        
        self.GlobalIP = None
        
        self.totalCmds = 0;
        
        self.StartTime = time.time() #This variable is only for debugging purposes
        
        print("\033[92m"+"Robot communication initialised on serial port: "+str(destPort)+"\033[0m")
        print()
    
    def reset(self):
        self.ser.write(bytearray([0x0]))
        
        time.sleep(DELAY)
        
        while self.ser.in_waiting > 0:
            print(ord(self.ser.read()))
        
        self.GlobalIP = None
    
    #Allow power to motors & set params
    def setup(self):
        print("Performing motor activation & setting params...")
        
        #Allow power
        self.stop(FREE_OFF)
        
        #Set params
        for ctrl in CTRLS:
            self.changeParam(ctrl, SPEED, 150)
            self.changeParam(ctrl, MAX_FORCE, 30)
        
        print("\033[94m"+"Robot setup finished"+"\033[0m")
        print()
        
        return True
    
    #Move a joint by a given amount
    #This must be called after setup() in order do anything
    def move(self, joint, value):
        #Stop all
        self.manual()
        
        # Set position
        self.setPos(CTRLS[joint], 0)
        
        # Move to position by value
        self.numeric(CTRLS[joint], value)
        
        # Start the movement, and wait
        self.start()
        self.wait()
    def move1(self, joint, value):
        #Stop all
        # self.manual()
        
        #
        self.setPos(CTRLS[joint], 0)
        # self.setPos(CTRLS[ZED], 440)
        # self.setPos(CTRLS[ZED], -5000)
        
        # self.manual(0,0,0,0,1,0,0)
        
        self.numeric(CTRLS[joint], -420)
        # self.numeric(CTRLS[ZED], 0)
        # self.numeric(CTRLS[ZED], 500)
        self.start()
        self.wait()
    
    #Move all joints maximum then to center
    def home(self):
        print("Returning home...")
        
        #Allow power
        self.setup()
        
        #self.home_ctrl(CTRLS[GRIPPER])
        self.home_ctrl(CTRLS[ZED])
        self.home_ctrl(CTRLS[SHOULDER])
        self.home_ctrl(CTRLS[YAW])
        self.home_ctrl(CTRLS[ELBOW])
        # self.home_wrist()
        
        #Stop all motors if any are stuck
        self.unpower_all()
        self.manual()
        
        print("\033[94m"+"Successfully returned robot to home position"+"\033[0m")
        print()
    
    #Go to the home position of a given controller
    def home_ctrl(self, ctrl):
        moves = []
        for c in CTRLS:
            if c == ctrl:
                moves.append(FORWARDS)
            else:
                moves.append(STOP_POWERED)
        
        for retry in range(3):
            self.manual(*moves)
            self.wait()
        
        #Set to max
        self.setPos(ctrl, ctrl[3])
        
        #Stop all
        self.manual()
        
        #Zero
        self.numeric(ctrl, 0)
        self.start()
        self.wait()
    
    #Return both wrist motors to home positions
    def home_wrist(self):
        #PLACEHOLDER CODE UNTIL WRIST FUNCTIONALITY HAS BEEN WORKED OUT
        print("Homing wrists")
    
    #Move all joints to maximum then minimum
    def soakTest(self):
        print("Running full soak test...")
        
        #Allow power
        self.setup()
        
        self.soak_ctrl(CTRLS[GRIPPER])
        self.soak_ctrl(CTRLS[ZED])
        self.soak_ctrl(CTRLS[SHOULDER])
        self.soak_ctrl(CTRLS[YAW])
        self.soak_ctrl(CTRLS[ELBOW])
        # self.soak_wrist()
        
        #Stop all motors if any are stuck
        self.unpower_all()
        self.manual()
        
        print("\033[94m"+"Soak test finished"+"\033[0m")
        print()
    
    #Soak test a given controller
    def soak_ctrl(self, ctrl):
        moves = []
        for c in CTRLS:
            if c == ctrl:
                moves.append(FORWARDS)
            else:
                moves.append(STOP_POWERED)
        
        for retry in range(3):
            self.manual(*moves)
            self.wait()
        
        #Set to max
        self.setPos(ctrl, ctrl[3])
        
        #Stop all
        #self.manual()
        
        #Zero
        #self.numeric(ctrl, 0)
        #self.start()
        #self.wait()
        
        moves = []
        for c in CTRLS:
            if c == ctrl:
                moves.append(BACKWARDS)
            else:
                moves.append(STOP_POWERED)
        
        for retry in range(3):
            self.manual(*moves)
            self.wait()
        
        #Set to min
        self.setPos(ctrl, ctrl[2])
        #self.numeric(ctrl, ctrl[2])
        #self.start()
        #self.wait()

        #Stop all
        self.manual()

        #Zero
        self.numeric(ctrl, 0)
        self.start()
        self.wait()
    
    #Return both wrist motors to home positions
    def soak_wrist(self):
        #PLACEHOLDER CODE UNTIL WRIST FUNCTIONALITY HAS BEEN WORKED OUT
        print("Soak testing wrists")
    
    #Control manual movement
    def manual(
        self,
        wrist1=STOP_POWERED,
        wrist2=STOP_POWERED,
        elbow=STOP_POWERED,
        shoulder=STOP_POWERED,
        zed=STOP_POWERED,
        yaw=STOP_POWERED,
        gripper=STOP_POWERED,
    ):
        ctrls = [0, 0]
        ctrls[CTRLS[ZED][0]] += zed << (CTRLS[ZED][1] * 2)
        ctrls[CTRLS[SHOULDER][0]] += shoulder << (CTRLS[SHOULDER][1] * 2)
        ctrls[CTRLS[ELBOW][0]] += elbow << (CTRLS[ELBOW][1] * 2)
        ctrls[CTRLS[YAW][0]] += yaw << (CTRLS[YAW][1] * 2)
        ctrls[CTRLS[GRIPPER][0]] += gripper << (CTRLS[GRIPPER][1] * 2)
        ctrls[CTRLS[WRIST1][0]] += wrist1 << (CTRLS[WRIST1][1] * 2)
        ctrls[CTRLS[WRIST2][0]] += wrist2 << (CTRLS[WRIST2][1] * 2)

        if self.command(0,0x80,(struct.pack(b'h', ctrls[0])[0]), (struct.pack(b'h', ctrls[0])[1]))[0] != 0:
            raise Exception("Manual move failed (1)")

        if self.command(1,0x80,(struct.pack(b'h', ctrls[1])[0]), (struct.pack(b'h', ctrls[1])[1]))[0] != 0:
            raise Exception("Manual move failed (2)")
    
    #Wait until all joints have stopped moving
    def wait(self):
        print("Waiting... ("+str(round(time.time() - self.StartTime,4))+"s)")
        while self.command(0, 0x17)[1] & 1:
            time.sleep(DELAY * 5)
        while self.command(1, 0x17)[1] & 1:
            time.sleep(DELAY * 5)
    
    #Switch between relative & absolute position controls.
    #Currently NOT FUNCTIONAL
    def mode(self, force_pos=0, abs_rel=1):
        for ctrl in CTRLS:
            while self.command(ctrl[0], 0x10 + ctrl[1])[1] & 0x00001000 != (abs_rel << 4):
                print("Toggle ABS/REL")
                self.command(ctrl[0], 0x19, ctrl[1])
            while self.command(ctrl[0], 0x10 + ctrl[1])[1] & 0x00010000 != (force_pos << 5):
                print("Toggle Force/Pos")
                self.command(ctrl[0], 0x18, ctrl[1])
    
    #Set a new destination for numeric movement
    def numeric(self, ctrl, value):
        # if value < ctrl[2] or value > ctrl[3]:
        #     raise Exception("Move exceeds bounds")
        self.changeParam(ctrl, NEW_POSITION, value)
    
    #Start numeric movement
    def start(self):
        #Note that start doesn't always return success
        #So far start seems to always return success
        print(self.command(0, 0xAC))
        print(self.command(1, 0xBF))
    
    #Stop numeric movement
    def stop(self, value):
        if self.command(0, 0x24 + value)[0] != 0:
            raise Exception("Stop failed (1)")
        if self.command(1, 0x24 + value)[0] != 0:
            raise Exception("Stop failed (2)")
    
    #Disable all motor power
    def unpower_all(self):
        self.manual(STOP_UNPOWERED,
            STOP_UNPOWERED,
            STOP_UNPOWERED,
            STOP_UNPOWERED,
            STOP_UNPOWERED,
            STOP_UNPOWERED,
            STOP_UNPOWERED
        )
    
    #Set a parameter to a different value
    def changeParam(self, ctrl, param, value):
        #ctrl = IP and controller to write to
        #param = parameter to change
        #value = value to change to
        if self.command(ctrl[0], 0x70 + ctrl[1], param, 0xCC)[0] != 0xE0:
            print("Setting Parameter failed")
        buf = self.command(ctrl[0], 0x78 + ctrl[1], (struct.pack(b'h', value)[0]), (struct.pack(b'h', value)[1]))
        if buf[0] != (self._checksum(value) + 0b11110000):
            raise Exception("Checksum Fail - "+str(bin(buf[0])[2:].zfill(8))+" != "+str(bin(self._checksum(value) + 0b11110000)[2:].zfill(8)))
    
    #Set position of a joint
    def setPos(self, ctrl, value):
        #ctrl = IP and controller to write to
        #value = value to change to
        buf = self.command(ctrl[0], 0x58 + ctrl[1], (struct.pack(b'h', value)[0]), (struct.pack('h', value)[1]))
        if buf[0] != (self._checksum(value) + 0b10100000):
            raise Exception("Checksum Fail - "+str(bin(buf[0])[2:].zfill(8))+" != "+str(bin(self._checksum(value) + 0b10100000)[2:].zfill(8)))
    
    #Read & return the position of a joint
    def getPos(self, ctrl):
        buf = b"".join(chr(c) for c in self.command(ctrl[0], 0x48 + ctrl[1])[1:])
        return struct.unpack(b'h', buf)[0]
    
    #Switches the IP and sends a command to the Robot
    #This function handles all communication to and from the robot. If anything goes wrong, it probably happened here
    def command(self, ip, cmd, b1=None, b2=0):
        self.totalCmds += 1
        
        while ip != self.GlobalIP:
            if DEBUG:
                print("Switching IP")
            if self.ser.write(bytearray([0x29])) != 1:
                raise Exception("Write failed (1)")
            time.sleep(DELAY)
            
            # --------------------------------------------
            temp = ord(self.ser.read())
            if temp != 0:
                if temp == 15:
                    raise Exception("Expected error, Please re-run script")
                raise Exception("Switch failed - Data: "+str(temp))
                #If you get this error after the ip is reported "-17", run the program again
                #This is an unfortunate, and currently unsolvable issue with this API
            if self.ser.write(bytearray([0x01])) != 1:
                raise Exception("Write failed (2)")
            time.sleep(DELAY)
            self.GlobalIP = ord(self.ser.read()) - 0x20
            if DEBUG:
                print("New IP: " + str(self.GlobalIP))
                if self.GlobalIP == -17:
                    print("\033[93m"+"--- Error Expected: Please re-run script! ---"+"\033[0m")
        
        if b1 is not None:
            if DEBUG:
                print("Command params: "+str(ip)+", "+str(cmd)+", "+str(b1)+", "+str(b2))
            if self.ser.write(bytearray([cmd]) + bytearray([b1]) + bytearray([b2])) != 3:
                raise Exception("Write failed (3)")
        else:
            if DEBUG:
                print("Command params: "+str(ip)+", "+str(cmd))
            if self.ser.write(bytearray([cmd])) != 1:
                raise Exception("Write failed (4)")
        time.sleep(DELAY)
        buf = []
        if DEBUG:
            print("Time: "+str(round(time.time() - self.StartTime,4))+"s")
        while self.ser.in_waiting > 0:
            buf.append(ord(self.ser.read()))
            if DEBUG:
                print("Command Buffer:",bin(buf[-1])[2:].zfill(8))
        if DEBUG:
            print()
        
        return buf
    
    #Don't know what this does - it appears to handle some sort of error/encoding checking
    @staticmethod
    def _checksum(value):
        return (struct.pack(b'h', value)[0] & 0xF) ^ \
            (struct.pack(b'h', value)[0] >> 4) ^ \
            (struct.pack(b'h', value)[1] & 0xF) ^ \
            (struct.pack(b'h', value)[1] >> 4)
    
    #Uses similar code execution to the example joystick code in the original API
    def joycontrol(self):
        self.setup()
        
        AXIS = [
            [0, SHOULDER],
            [1, ZED],
            [2, ELBOW],
            [3, YAW]
        ]
        BUTTON = [
            [1, GRIPPER, 1],
            [2, GRIPPER, -1],
            [4, WRIST1, 1],
            [6, WRIST1, -1],
            [5, WRIST2, 1],
            [7, WRIST2, -1]
        ]

        CONTROL = [10,10,0,0]
        # CONTROL2 = [0,0,0,0,0,0]
        
        pos = []
        for ctrl in CTRLS:
            pos.append(0)
            
        while True:
            time.sleep(.1)
            old_pos = copy.copy(pos)
            for axis in AXIS:
                pos[axis[1]] += CONTROL[axis[0]]
            # for button in BUTTON:
            #     pos[button[1]] += 0
            i = 0
            for ctrl in CTRLS:
                if pos[i] < ctrl[2]:
                    pos[i] = ctrl[2]
                if pos[i] > ctrl[3]:
                    pos[i] = ctrl[3]
                if pos[i] != old_pos[i]:
                    self.numeric(ctrl, pos[i])
                i += 1
            self.start()
            print(pos)


if __name__ == "__main__":
    print("Running script independantly...")
    print()
    decision = input("Would you like to run a Soak Test? (y/n) - ")
    if decision == "y":
        r = RTX()
        r.soakTest()
        r.home()
    else:
        print("Exiting...")
    exit()
