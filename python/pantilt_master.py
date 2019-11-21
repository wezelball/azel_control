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
from threading import Timer
import logging

# Set up the i2c bus
bus = smbus.SMBus(1)

# I2C address of Arduino Slaves
steppers_i2c_address = 0x04
encoders_i2c_address = 0x05
i2c_cmd = 0x01

# Start a message queue
messageQ = []

# *********************** BEGIN CLASSES ******************************

class RepeatingTimer(object):

    def __init__(self, interval, f, *args, **kwargs):
        self.interval = interval
        self.f = f
        self.args = args
        self.kwargs = kwargs

        self.timer = None

    def callback(self):
        self.f(*self.args, **self.kwargs)
        self.start()

    def cancel(self):
        self.timer.cancel()

    def start(self):
        self.timer = Timer(self.interval, self.callback)
        self.timer.start()

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
            # Update the process logfile
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
                
            # Monitor the run status of the steppers,
            # updating the process varaibles
            # This will used to invoke jam detection later
            if isRunning(0):    # azimuth
                variable.isAzRunning = True
                # Look to see if stopped
                if variable.azVelocity < 1.0:
                    if variable.azFailTiming == False:
                        az_fail_timer.start()
                        variable.azFailTiming = True
                        logging.debug("periodicThread.run() azFailTiming = True")
                else:   # running above min velocity
                    if variable.azFailTiming == True:
                        az_fail_timer.cancel()
                        variable.azFailTiming = False
                        logging.debug("periodicThread.run() azFailTiming = False")
                
            if isRunning(1):    # elevation
                variable.isElRunning = True
                # Look to see if stopped
                if variable.elVelocity < 1.0:
                    if variable.elFailTiming == False:
                        el_fail_timer.start()
                        variable.elFailTiming = True
                        logging.debug("periodicThread.run() elFailTiming = True")
                else:   # running above min velocity
                    if variable.elFailTiming == True:
                        el_fail_timer.cancel()
                        variable.elFailTiming = False
                        logging.debug("periodicThread.run() elFailTiming = False")                

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
        self.isAzRunning = False;
        self.isElRunning = False;

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
        
        # fail timer flags
        self.azFailTiming = False
        self.elFailTiming = False

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

# Sends the message over the i2c bus, using a 
# prioritized messageQ
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


# Gets he position of the stepper defined by axis
# axis 0 = azimuth
# axis 1 = elevation
def getStepperPosn(axis):
    cmd = "18" + ':' + str(axis)
    reply  = sendStepperCommand(cmd)
    
    # iterate through the string and pick out digits only
    result = ""
    for i in reply:
        if ord(i) >= 48 and ord(i) <= 57:
            result += i
    
    # for debugging
    print ("result: %s" %result)
    logging.debug("getStepperPosn() returned %s", result)
    
    return int(result)
    

# Send a stepper motion command to the Uno
def sendStepperCommand(cmd):
    reply = sendMessage(1, cmd, steppers_i2c_address)
    return reply

# Process functions
def setAzSpeed(speed):
    cmd = "12" + ':' + str(speed)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setAzSpeed() %s", speed)

def setAzMaxSpeed(speed):
    cmd = "16" + ':' + str(speed)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setAzMaxSpeed() %s", speed)

def setElSpeed(speed):
    cmd = "13" + ':' + str(speed)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setElSpeed() %s", speed)

def setElMaxSpeed(speed):
    cmd = "17" + ':' + str(speed)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setElMaxSpeed() %s", speed)

def setAzAccel(accel):
    cmd = "14" + ':' + str(accel)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setAzAccel() %s", accel)

def setElAccel(accel):
    cmd = "15" + ':' + str(accel)
    #print("command: %s" % cmd)
    sendStepperCommand(cmd)
    logging.debug("setAzAccel() %s", accel)

# Watches the axis while it's homing
# When home limit made, terminate homing mode 
# If the encoder velocity approaches zero while
# homing, issue an estop and error
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
    # should replace this with relMoveEl(distance)
    logging.debug("slewNorth()")
    relMoveEl(8000)

def slewEast():
    # should replace this with relMoveAz(distance)
    logging.debug("slewEast()")
    relMoveAz(8000)

def slewWest():
    # should replace this with relMoveAz(distance)
    logging.debug("slewWest()")
    relMoveAz(-8000)

def slewSouth():
    # should replace this with relMoveEl(distance)
    logging.debug("slewSouth()")
    relMoveEl(-8000)

def stopAllSlew():
    logging.debug("stopAllSlew()")
    print(sendStepperCommand("3:0"))

def relMoveAz(distance):
    cmd = '1:' + str(distance)
    logging.debug("relMoveAz(%s)", distance)
    print (sendStepperCommand(cmd)) 

def relMoveEl(distance):
    cmd = '2:' + str(distance)
    logging.debug("relMoveEl(%s)", distance)
    print (sendStepperCommand(cmd))

def printEncoders():
    print(getEncoders())

def stopAz():
    logging.debug("stopAz()")
    print(sendStepperCommand("4:0"))

def stopEl():
    logging.debug("stopEl()")
    print(sendStepperCommand("5:0"))

# There is a bug when quickStop functions are called
# and a later move is performed, it starts
# at full speed with no accel
def quickStopAz():
    logging.debug("quickStopAz()")
    print(sendStepperCommand("6:0"))

def quickStopEl():
    logging.debug("quickStopEl()")
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
    logging.debug("homeAzimuth()")
    # slew west a long friggin way
    relMoveAz(-40000)

def homeElevation():
    variable.elHoming = True
    logging.debug("homeElevation()")
    # slew south a long friggin way
    relMoveEl(40000)

# Sets both encoder axes to zero
# Use when both encoders are in home position and stopped
# I really need to pass an axis as argument and 
# have this zero each encoder separately.  Most of the
# work would be in the Nano
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

    logging.debug("zeroEncoders() returned %s", encPosList)
    
    return tuple(encPosList)    

# Is the stepper axis running?
def isRunning(axis):
    cmd = "19" + ':' + str(axis)
    reply = sendStepperCommand(cmd)

    # iterate through the string and pick out digits only
    result = ""
    for i in reply:
        if ord(i) >= 48 and ord(i) <= 57:
            result += i        
                
    logging.debug("isRunning() axis %s result %s", axis, result)    

    if result == "1":
        #print("True")
        return True
    else:
        #print("False")
        return False
    
def azJam():
    logging.debug("azJam() timeout")
    #print("azimuth jam detected")
    
def elJam():
    logging.debug("elJam() timeout")
    #print("elevation jam detected")

# Called if no case passed to switch function
def case_default():
    #print("No case")
    logging.debug("case_default() entered")

def switchCase(case):
    return {
        ":Mn#":slewNorth,
        ":Me#":slewEast,
        ":Mw#":slewWest,
        ":Ms#":slewSouth,
        ":Q#":stopAllSlew,
        "0":printEncoders,
        "1":stopAz,
        "2":stopEl,
        "3":quickStopAz,
        "4":quickStopEl,
        "5":isAzCWLimit,
        "6":isAzCCWLimit,
        "7":isElUpLimit,
        "8":isElDownLimit,
        "9":homeAzimuth,
        "10":homeElevation,
        "11":zeroEncoders,
        "12":relMoveAz,       # requires distance
        "13":relMoveEl,       # requires distance
        "14":getStepperPosn,  # requires axis, 0=az, 1=el
        "15":isRunning,       # requires axis, 0=az, 1=el
    }.get(case, case_default)

# loop to send message
exit = False

# Process logfile for live process parameters
logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
log = open(logfile, 'w')
# Write the header
log.write("Time,azEncoder, elEncoder" + '\n')

# This is the python logging module
logging.basicConfig(filename='debug.log', filemode='w', \
                    format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)

# Instantiate global variables
variable = Variables()

# Start the comms thread after initialization
commThread = periodicThread(1, "Thread-1")
commThread.start()

# Set timer thread
az_fail_timer = RepeatingTimer(1.0, azJam)
el_fail_timer = RepeatingTimer(1.0, elJam)

# Set initial values - motor speeds, etc
setInitialValues()

while not exit:

    # Get the user command
    r = input('-> ')
    cmdList = r.split()

    # Pass the command to the switch construct
    if len(cmdList) == 1:           # command with no parms
        switchCase(cmdList[0])()
    elif len(cmdList) == 2:         # command with one parm
        switchCase(cmdList[0])(cmdList[1])

    if cmdList[0] == 'q':
        exit=True

# Exit program here
commThread.stop()

