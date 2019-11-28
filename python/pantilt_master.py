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
import ctypes
from threading import Thread, Event, Timer
import logging

# Set up the i2c bus
bus = smbus.SMBus(1)

# I2C address of Arduino Slaves
steppers_i2c_address = 0x04
encoders_i2c_address = 0x05
i2c_cmd = 0x01

# Start a message queue
messageQ = []

# i2c comms error flags
encoderIOError = False
stepperIOError = False


# *********************** BEGIN CLASSES ******************************

# Moving average class
class MovingAverage():
    def __init__(self, windowsize):
        self.windowsize = windowsize
        self._setWindowSize(windowsize)
        self.accumValue = 0.0
        
    def _setWindowSize(self, size):
        self.window = [0.0 for i in range(self.windowsize)]
        
    def addValue(self,value):
        self.window.insert(0,value)
        self.window.pop()
        
    def computeAverage(self):
        self.accumValue = 0
        for i in self.window:
            self.accumValue += i
            
        return self.accumValue/len(self.window)

# Checks for failure of motion in either axis
class motionThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        self.delay = 1.5
        self.azFailTiming = False
        self.elFailTiming = False
        self.azAvgVel = 0.0
        self.elAvgVel = 0.0        
        
    
    def run(self):
        while self.running:
            time.sleep(self.delay)
            #logging.debug("motionThread() %s running", self.name)

            # Compute average velocities
            self.azAvgVel = variable.azAvgVelocity.computeAverage()
            self.elAvgVel = variable.elAvgVelocity.computeAverage()            

            #logging.debug("motionThread() azAvgVel %s", self.azAvgVel)
            #logging.debug("motionThread() elAvgVel %s", self.elAvgVel)
            
            # Monitor the run status of the steppers,
            # updating the process varaibles
            # This will used to invoke jam detection later
            if variable.isAzRunning == True:    # azimuth
                # Look to see if stopped
                if abs(variable.azAvgVelocity.computeAverage()) < 1.0 and isRunning(0) == True:
                    if self.azFailTiming == False:
                        self.azFailTiming = True
                        logging.debug("motionThread.run() azFailTiming = True")
                    elif self.azFailTiming == True:
                        self.azFailTiming == False
                        azJam()
                    
                elif abs(variable.azAvgVelocity.computeAverage()) > 1.0:   # running above min velocity
                    if self.azFailTiming == True:
                        self.azFailTiming = False
                        logging.debug("motionThread.run() azFailTiming = False")

                
            if variable.isElRunning == True:    # elevation
                # Look to see if stopped
                if abs(variable.elAvgVelocity.computeAverage()) < 1.0 and isRunning(1) == True:
                    if self.elFailTiming == False:
                        self.elFailTiming = True
                        logging.debug("motionThread.run() elFailTiming = True")
                    elif self.elFailTiming == True:
                        self.elFailTiming = False
                        elJam()
                        
                elif abs(variable.elAvgVelocity.computeAverage()) > 1.0:   # running above min velocity
                    if self.elFailTiming == True:
                        self.elFailTiming = False
                        logging.debug("motionThread.run() elFailTiming = False")    
        
    def stop(self):
        self.running = False
        
    def get_id(self): 
        # returns id of the respective thread 
        if hasattr(self, '_thread_id'): 
            return self._thread_id 
        for id, thread in threading._active.items(): 
            if thread is self: 
                return id
    
    def raise_exception(self): 
        thread_id = self.get_id()
        # i keep getting exceptions here when leaving the program
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, ctypes.py_object(SystemExit)) 
        if res > 1: 
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
            print('Exception raise failure')


# This thread is for updating live parameters
class periodicThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        self.delay = 0.2
        self.lastAz, self.lastEl = getEncoders()
        self.azAvgVel = 0.0
        self.elAvgVel = 0.0

    def run(self):
        global encoderIOError
        global stepperIOError
        
        while(self.running):
            time.sleep(self.delay)
            
            # Get position from encoders, but set flag if error
            try:
                variable.azPos, variable.elPos = getEncoders()
            except ValueError:
                encoderIOError = True
            
            # Get positions from steppers
            if stepperIOError == False:    # don't try to update if there's an IOError
                variable.azStepperPos = getStepperPosn(0)
                variable.azStepperPos = getStepperPosn(1)
            
            # Calculate velocities
            variable.azVelocity = (variable.azPos - self.lastAz)/self.delay
            variable.azAvgVelocity.addValue(variable.azVelocity)
            variable.elVelocity = (variable.elPos - self.lastEl)/self.delay
            variable.elAvgVelocity.addValue(variable.elVelocity)
            
            # Compute average velocities
            self.azAvgVel = variable.azAvgVelocity.computeAverage()
            self.elAvgVel = variable.elAvgVelocity.computeAverage()
            
            # Save encoder position to last position
            try:
                self.lastAz,self.lastEl = getEncoders()
            except ValueError:
                encoderIOError = True            
            
            # Update the process logfile
            log.write(time.strftime('%H:%M:%S') + ',' +  str(variable.azPos) + ',' + str(variable.elPos) + ',' + str(variable.azStepperPos) + ',' \
                      + str(variable.azStepperPos)+ ',' +  str(variable.azVelocity) + ',' + str(variable.elVelocity) + '\n')

            # Logging
            #logging.debug("periodicThread() azVelocity %s", variable.azVelocity)
            #logging.debug("periodicThread() elVelocity %s", variable.elVelocity)
            #logging.debug("periodicThread() azAvgVel %s", self.azAvgVel)
            #logging.debug("periodicThread() elAvgVel %s", self.elAvgVel)            

            # Watch homing axes if homing
            if variable.azHoming == True:
                watchHomingAxis(0)
            
            if variable.elHoming == True:
                watchHomingAxis(1)
                
                
    def print_time(self,threadName):
        print ("%s: %s") % (threadName, time.ctime(time.time()))

    def stop(self):
        self.running = False
        log.close()

    def get_id(self): 
        # returns id of the respective thread 
        if hasattr(self, 'threadID'): 
            return self.threadID 
        for id, thread in threading._active.items(): 
            if thread is self: 
                return id
    
    def raise_exception(self): 
        thread_id = self.get_id() 
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, ctypes.py_object(SystemExit)) 
        if res > 1: 
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
            print('Exception raise failure')



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
        self.azAvgVelocity = MovingAverage(3)
        self.elAvgVelocity = MovingAverage(3)
        self.azStepperPos = getStepperPosn(0)
        self.elStepperPos = getStepperPosn(1)

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
    setAzSpeed(500)
    setAzMaxSpeed(500)
    time.sleep(0.25)
    setAzAccel(500)
    time.sleep(0.25)
    setElSpeed(500)
    setElMaxSpeed(500)
    time.sleep(0.25)
    setElAccel(500)

# Sends the message over the i2c bus, using a 
# prioritized messageQ
def sendMessage(priority, message, i2c_address):
    global encoderIOError
    global stepperIOError
    
    messageQ.append((priority, message))
    messageQ.sort(reverse = True)

    while messageQ:
        # this is a tuple (priority, message)
        # 1 is the highest priority
        # higher numbers are lower priority
        next_message = messageQ.pop()
        #logging.debug("sendMessage() messageQ: %s", next_message)
        bytesToSend = ConvertStringToBytes(next_message[1])
        reply = ""
        replyBytes = ""
        #logging.debug("sendMessage() raw: %s", bytesToSend)
        try:
            bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)

            if i2c_address == steppers_i2c_address:
                replyBytes = bus.read_i2c_block_data(steppers_i2c_address, 0, 8)
            else:
                replyBytes = bus.read_i2c_block_data(encoders_i2c_address, 0, 16)

            reply = ConvertBytesToString(replyBytes)
        except IOError:
            if i2c_address == encoders_i2c_address:
                encoderIOError = True
                reply = ""
                logging.warn("sendMessage() - encoderIOError")
                
            elif i2c_address == steppers_i2c_address:
                stepperIOError = True
                reply = ""
                logging.warn("sendMessage() - stepperIOError") 
                
        else:   # Reset error flags if there was a good read
            if i2c_address == encoders_i2c_address:
                encoderIOError = False
            elif i2c_address == steppers_i2c_address:
                stepperIOError = False

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
            #encPosList = variable.azPos, variable.elPos
            logging.warn("getEncoders() - IOError")
            
    #logging.debug("getEncoders() returned %s", encPosList)
    
    return tuple(encPosList)


# Gets he position of the stepper defined by axis
# axis 0 = azimuth
# axis 1 = elevation
def getStepperPosn(axis):
    global stepperIOError
    
    cmd = "18" + ':' + str(axis)
    reply  = sendStepperCommand(cmd)
    
    # iterate through the string and pick out digits only
    result = ""
    for i in reply:
        if ord(i) >= 48 and ord(i) <= 57:
            result += i
    
    # for debugging
    #logging.debug("getStepperPosn() returned %s", result)
    
    if stepperIOError == False:
        try:
            return int(result)
        except ValueError:
            stepperIOError = True
            return 0
    

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
    logging.debug("watchHomingAxis() homing axis %s", axis)
    if axis == 0:
        if isAzCCWLimit() == True:
            variable.azHoming = False
            variable.azHomed = True
            variable.isAzRunning = False
            zeroAzEncoder()
            logging.debug("watchHomingAxis() azimuth homed")
    elif axis == 1:
        if isElDownLimit() == True:
            variable.elHoming = False
            variable.elHomed = True
            variable.isElRunning = False
            zeroElEncoder()
            logging.debug("watchHomingAxis() elevation homed")
            
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
    pulses = 0
    if axis == 0:
        pulses = float(degrees) * 401.250
    elif axis == 1:
        pulses = float(degrees) * 769.166
        
    return int(pulses)

# Given encoder pulses, return degree equivalent
# The encoder gear relationships on both axes
# are identical
def encoderCountsToDegrees(pulses):
    degreesPerPulse = 0.092308
    degrees = pulses * degreesPerPulse
    return degrees

def encoderDegreesToCounts(degrees):
    pulsesPerDegree = 10.83333333
    pulses = degrees * pulsesPerDegree
    return int(pulses)

# ****************** Command functions ***********************
def slewNorth():
    logging.debug("slewNorth()")
    relMoveEl(8000)

def slewEast():
    logging.debug("slewEast()")
    relMoveAz(8000)

def slewWest():
    logging.debug("slewWest()")
    relMoveAz(-8000)

def slewSouth():
    logging.debug("slewSouth()")
    relMoveEl(-8000)

def stopAllSlew():
    logging.debug("stopAllSlew()")
    print(sendStepperCommand("3:0"))
    variable.isAzRunning = False
    variable.isElRunning = False

def relMoveAz(distance):
    cmd = '1:' + str(distance)
    logging.debug("relMoveAz(%s)", distance)
    print (sendStepperCommand(cmd))
    time.sleep(0.5)
    variable.isAzRunning = True

def relMoveEl(distance):
    cmd = '2:' + str(distance)
    
    logging.debug("relMoveEl(%s)", distance)
    print (sendStepperCommand(cmd))
    time.sleep(0.5)
    variable.isElRunning = True

def printEncoders():
    print(getEncoders())

def stopAz():
    logging.debug("stopAz()")
    print(sendStepperCommand("4:0"))
    variable.isAzRunning = False

def stopEl():
    logging.debug("stopEl()")
    print(sendStepperCommand("5:0"))
    variable.isElRunning = False

# There is a bug when quickStop functions are called
# and a later move is performed, it starts
# at full speed with no accel
def quickStopAz():
    logging.debug("quickStopAz()")
    sendStepperCommand("6:0")
    variable.isAzRunning = False
    variable.azHoming = False

def quickStopEl():
    logging.debug("quickStopEl()")
    sendStepperCommand("7:0")
    variable.isElRunning = False
    variable.elHoming = False

# Move the axis the number of degrees specified
def moveAzStepperDegrees(degrees):
    logging.debug("moveAzStepperDegrees() %s", degrees)
    relMoveAz(stepperDegreesToPulses(0, degrees))

# Move the axis the number of degrees specified
def moveElStepperDegrees(degrees):
    logging.debug("moveElStepperDegrees() %s", degrees)
    relMoveEl(stepperDegreesToPulses(1, degrees))

# returns True if limit made
def isAzCWLimit():
    variable.azCWLimit = sendStepperCommand("8:0")
    if variable.azCWLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.azCWLimit.find('1') != -1:
        #print ("False")
        return False

def printIsAzCWLimit():
    print(isAzCWLimit())

# returns True if limit made	
def isAzCCWLimit():
    variable.azCCWLimit = sendStepperCommand("9:0")
    if variable.azCCWLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.azCCWLimit.find('1') != -1:
        #print ("False")
        return False

def printIsAzCCWLimit():
    print(isAzCCWLimit())

# returns True if limit made
def isElUpLimit():
    variable.elUpLimit = sendStepperCommand("10:0")
    if variable.elUpLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.elUpLimit.find('1') != -1:
        #print ("False")
        return False

def printIsElUpLimit():
    print(isElUpLimit())

# returns True if limit made
def isElDownLimit():
    variable.elDownLimit = sendStepperCommand("11:0")
    if variable.elDownLimit.find('0') != -1:
        #print ("True")
        return True
    elif variable.elDownLimit.find('1') != -1:
        #print ("False")
        return False

def printIsElDownLimit():
    print(isElDownLimit())


# 0 = azimuth
# 1 = elevation
def printStepperPosition(axis):
    print (getStepperPosn(axis))


# For now, let's home west
def homeAzimuth():
    variable.azHoming = True
    logging.debug("homeAzimuth() executed")
    # slew west a long friggin way
    relMoveAz(-40000)

def homeElevation():
    variable.elHoming = True
    logging.debug("homeElevation() executed")
    # slew south a long friggin way
    relMoveEl(-40000)

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

def zeroAzEncoder():
    encPosList = []
    cmd = '2'
    reply = sendMessage(1, cmd, encoders_i2c_address)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = variable.azPos, variable.elPos

    logging.debug("zeroAzEncoder() returned %s", encPosList)
    
    return tuple(encPosList)    

def zeroElEncoder():
    encPosList = []
    cmd = '3'
    reply = sendMessage(1, cmd, encoders_i2c_address)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = variable.azPos, variable.elPos

    logging.debug("zeroAzEncoder() returned %s", encPosList)
    
    return tuple(encPosList)

# Is the stepper axis running?
def isRunning(axis):
    cmd = "19" + ':' + str(axis)
    reply = sendStepperCommand(cmd)
    
    result = ""
    i = ""
    
    # iterate through the string and pick out digits only
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
    variable.azFailTiming = False
    quickStopAz()
    #print("azimuth jam detected")
    
def elJam():
    logging.debug("elJam() timeout")
    quickStopEl()
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
        "5":printIsAzCWLimit,
        "6":printIsAzCCWLimit,
        "7":printIsElUpLimit,
        "8":printIsElDownLimit,
        "9":homeAzimuth,
        "10":homeElevation,
        "11":zeroEncoders,
        "12":relMoveAz,             # requires distance
        "13":relMoveEl,             # requires distance
        "14":printStepperPosition,  # requires axis, 0=az, 1=el
        "15":isRunning,             # requires axis, 0=az, 1=el
        "16":zeroAzEncoder,
        "17":zeroElEncoder,
        "18":moveAzStepperDegrees,  # requires axis, 0=az, 1=el
        "19":moveElStepperDegrees,  # requires axis, 0=az, 1=el
    }.get(case, case_default)


if __name__ == "__main__":

    # loop to send message
    exit = False

    # Process logfile for live process parameters
    logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
    log = open(logfile, 'w')
    # Write the header
    log.write("Time,azEncoder, elEncoder, azStepper, elStepper, azVel, elVel" + '\n')

    # This is the python logging module
    logging.basicConfig(filename='debug.log', filemode='w', \
                    format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)


    # Instantiate global variables
    variable = Variables()    

    # Start the comms thread after initialization
    commThread = periodicThread(1, "Thread-1")
    commThread.start()

    motionCheckThread = motionThread(1.5, "m_thread")
    motionCheckThread.start()

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
            exit = True

            # Exit program here

            # Kill threads
            commThread.raise_exception()
            motionCheckThread.raise_exception()

            time.sleep(1.0)
            log.close()