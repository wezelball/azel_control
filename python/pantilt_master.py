#!/usr/bin/env python3

# pantilt_master.py
# Dave Cohen

#import sys
import smbus
import time
#import os
import datetime
#import ephem
import threading

# Set up the i2c bus
bus = smbus.SMBus(1)

# I2C address of Arduino Slaves
steppers_i2c_address = 0x04
encoders_i2c_address = 0x05
i2c_cmd = 0x01

# Start a message queue
messageQ = []

# *********************** BEGIN CLASSES ******************************

# This thread is for updating live parameters
class periodicThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        #self.delay = 0.5
        self.delay = 2.0
        self.lastAz, self.lastEl = getEncoders()

    def run(self):
        while(self.running):
            time.sleep(self.delay)
            variable.azPos, variable.elPos = getEncoders()
            variable.azVelocity = (variable.azPos - self.lastAz)/self.delay
            variable.elVelocity = (variable.elPos - self.lastEl)/self.delay
            self.lastAz,self.lastEl = getEncoders()
            #Update the logfile
            log.write(time.strftime('%H:%M:%S') + ',' +  str(variable.azPos) + ',' + str(variable.elPos) + \
                      ',' +  str(variable.azVelocity) + ',' + str(variable.elVelocity) + '\n')

            # watch for limits reached if homing
            if variable.azHoming == True:
                #print("azHoming")
                if isAzCCWLimit() == True:
                    variable.azHomed = True
                    #print("azHomed")
                    
            if variable.elHoming == True:
                #print("elHoming")
                if isElUpLimit() == True:
                    variable.elHomed = True
                    #print("elHomed")
                    
            if variable.azHoming and variable.azHomed and variable.elHoming and variable.elHomed:
                print("Homed")
                variable.azHoming = False
                variable.elHoming = False
                zeroEncoders()
                

    def print_time(self,threadName):
        print ("%s: %s") % (threadName, time.ctime(time.time()))

    def stop(self):
        self.running = False
        log.close()

class Variables():
    def __init__(self):
        # initialize limit switches
        self.azCWLimit = False
        self.azCCWLimit = False
        self.elUpLimit = False
        self.elDownLimit = False

        # Process variables
        self.azHomingSpeed = 500;
        self.elHomingSpeed = 500;
        self.azSlewSpeed = 500;
        self.elSlewSpeed = 500;
        self.azAccel = 500;
        self.elAccel = 500;

        # Real-time values
        self.azVelocity = 0
        self.elVelocity = 0
        self.azPos = 0
        self.elPos = 0

        # homing flags
        self.azHomed = False
        self.azHoming = False
        self.elHomed = False
        self.elHoming = False

# ************************* END CLASSES ******************************

# Sets inital values
def setInitialValues():
    setAzSpeed(1000)
    setAzMaxSpeed(1000)
    time.sleep(0.25)
    setAzAccel(500)
    time.sleep(0.25)
    setElSpeed(500)
    setElMaxSpeed(1000)
    time.sleep(0.25)
    setElAccel(500)

def sendMessage(priority, message, i2c_address):
    messageQ.append((priority, message))
    messageQ.sort(reverse = True)

    while messageQ:
        # this is a tuple (priority, message)
        # 1 is the highest priority
        # higher numbers are lower priority
        next_message = messageQ.pop()
        bytesToSend = ConvertStringToBytes(next_message[1])
        reply = ""
        replyBytes = ""
        try:
            bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)

            if i2c_address == steppers_i2c_address:
                replyBytes = bus.read_i2c_block_data(steppers_i2c_address, 0, 8)
            else:
                replyBytes = bus.read_i2c_block_data(encoders_i2c_address, 0, 16)

            reply = ConvertBytesToString(replyBytes)
        except IOError:
            reply = "Error"

        return reply

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

def ConvertBytesToString(src):
    result = ""
    for i in src:
        result += chr(i)

    return result

# Read the encoder values from the Nano as a tuple
# 1st element is azimuth
# 2nd element is elevation
# I'm getting bus errors periodically, which might
# be due to excessive cable length
def getEncoders():
    encPosList = []
    cmd = '0'	# send one byte
    reply = sendMessage(1, cmd, encoders_i2c_address)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = variable.azPos, variable.elPos

    return tuple(encPosList)

# Send a stepper motion command to the Uno
def sendStepperCommand(cmd):
    reply = sendMessage(1, cmd, steppers_i2c_address)
    return reply

# Process functions
def setAzSpeed(speed):
    cmd = "12" + ':' + str(speed)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

def setAzMaxSpeed(speed):
    cmd = "16" + ':' + str(speed)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

def setElSpeed(speed):
    cmd = "13" + ':' + str(speed)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

def setElMaxSpeed(speed):
    cmd = "17" + ':' + str(speed)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

def setAzAccel(accel):
    cmd = "14" + ':' + str(accel)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

def setElAccel(accel):
    cmd = "15" + ':' + str(accel)
    print("command: %s" % cmd)
    print (sendStepperCommand(cmd))

# Watches the axis while it's homing
# When home limit made, terminate homing mode 
# If the encoder velocity approaches zero while
#  homing, issue an estop and error
#
# axis = 0    azimuth
# axis = 1    elevation
#
# Currently assumes that home positions are
# az = west and el = down
def watchHomingAxis(axis):
    if axis == 0:
        if isAzCCWLimit == True:
            variable.azHoming = False
            variable.azHomed = True
    elif axis == 1:
        if isElDownLimit == True:
            variable.elHoming = False
            variable.elHomed = True
            
# Given stepper pulses, return degrees
# Assume the zeroth pulse is a 0 degrees (no offset)
# Negative values are allowed
# axis 0 = azimuth
# axis 1 = elevation
def stepperPulsesToDegrees(axis, pulses):
    # for azimuth axis
    # remember, reduce speed = multiply resolution
    # 400 pulses per rev (2x microstepping)
    # multiplied by 10:1 gearbox on output of stepper
    # multiplied by 13/18 for chain reduction
    # multiplied by 40 for # teeth in worm wheel (guess)
    if axis == 0:
        degrees = float(pulses)/401.250
    elif axis == 1:
        degrees = float(pulses)/769.166
            
    return degrees
        
# Given position in degrees, return pulses
# Assume the zeroth degree is at pulse 0 (no offset)
# Negative values are allowed
# axis 0 = azimuth
# axis 1 = elevation
def stepperDegreesToPulses(axis, degrees):
    if axis == 0:
        pulses = degrees * 401.250
    elif axis == 1:
        pulses = degrees * 769.166
        
    return int(pulses)

# Given encoder pulses, return degree equivalent
# The encoder gear relationships on both axes
# are identical
def encoderPulsesToDegrees(pulses):
    degreesPerPulse = 0.092308
    degrees = pulses * degreesPerPulse
    return degrees

def encoderDegreesToPulses(degrees):
    pulsesPerDegree = 10.83333333
    pulses = degrees * pulsesPerDegree
    return pulses

# ****************** Command functions ***********************
def slewNorth():
    print (sendStepperCommand("2:8000"))

def slewEast():
    print(sendStepperCommand("1:8000"))   

def slewWest():
    print(sendStepperCommand("1:-8000"))

def slewSouth():
    print(sendStepperCommand("2:-8000"))

def stopAllSlew():
    print(sendStepperCommand("3:0"))

def relMoveAz(distance):
    cmd = '1:' + str(distance)
    print (sendStepperCommand(cmd)) 

def relMoveEl(distance):
    cmd = '2:' + str(distance)
    print (sendStepperCommand(cmd))

def printEncoders():
    print(getEncoders())

def stopAz():
    print(sendStepperCommand("4:0"))

def stopEl():
    print(sendStepperCommand("5:0"))

# There is a bug when quickStop functions are called
# and a later move is performed, it starts
# at full speed with no accel
def quickStopAz():
    print(sendStepperCommand("6:0"))

def quickStopEl():
    print(sendStepperCommand("7:0"))

# returns 0 if limit made
def isAzCWLimit():
    variable.azCWLimit = sendStepperCommand("8:0")
    if variable.azCWLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.azCWLimit.find('1') != -1:
        #print ("False")
        return False

# returns 0 if limit made	
def isAzCCWLimit():
    variable.azCCWLimit = sendStepperCommand("9:0")
    if variable.azCCWLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.azCCWLimit.find('1') != -1:
        #print ("False")
        return False

# returns 0 if limit made
def isElUpLimit():
    variable.elUpLimit = sendStepperCommand("10:0")
    if variable.elUpLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.elUpLimit.find('1') != -1:
        #print ("False")
        return False

# returns 0 if limit made
def isElDownLimit():
    variable.elDownLimit = sendStepperCommand("11:0")
    if variable.elDownLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.elDownLimit.find('1') != -1:
        #print ("False")
        return False

# For now, let's home west
def homeAzimuth():
    variable.azHoming = True
    # slew west a long friggin way
    relMoveAz(-40000)

def homeElevation():
    variable.elHoming = True
    # slew south a long friggin way
    relMoveEl(40000)

# Sets both encoder axes to zero
# Use when both encoders are in home position and stopped
def zeroEncoders():
    encPosList = []
    cmd = '1'
    reply = sendMessage(1, cmd, encoders_i2c_address)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = variable.azPos, variable.elPos

    return tuple(encPosList)    

def processCmd(cmd):
    switcher = {
        ':Mn#':slewNorth,
        ':Me#':slewEast,
        ':Mw#':slewWest,
        ':Ms#':slewSouth,
        ':Q#':stopAllSlew,
        '0':printEncoders,
        '1':stopAz,
        '2':stopEl,
        '3':quickStopAz,
        '4':quickStopEl,
        '5':isAzCWLimit,
        '6':isAzCCWLimit,
        '7':isElUpLimit,
        '8':isElDownLimit,
        '9':homeAzimuth,
        '10':homeElevation,
        '11':zeroEncoders
    }
    func=switcher.get(cmd,lambda :'Invalid')
    return func()

# loop to send message
exit = False

# There needs to be a logfile
logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
log = open(logfile, 'w')
# Write the header
log.write("Time,azEncoder, elEncoder" + '\n')

# Instantiate global variables
variable = Variables()

# Start the comms thread after initialization
commThread = periodicThread(1, "Thread-1")
commThread.start()

# Set initial values - motor speeds, etc
setInitialValues()

while not exit:

    r = str(input('-> '))

    processCmd(r)

    if r=='q':
        exit=True

# Exit program here
commThread.stop()

