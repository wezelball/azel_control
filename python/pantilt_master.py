#!/usr/bin/env python3

# pantilt_master.py
# Dave Cohen

import sys
import smbus
import time
#import os
import datetime
#import ephem
import threading
import ctypes
from threading import Thread, Event, Timer
# I love python logging
import logging

# Trying pysimpleGUI
import PySimpleGUI as sg

# This is where I will keep global data
import config

# Set up the i2c bus
bus = smbus.SMBus(1)

# I2C address of Arduino Slaves
#steppers_i2c_address = 0x04
#encoders_i2c_address = 0x05
#i2c_cmd = 0x01

# Start a message queue
messageQ = []

# i2c comms error flags
#encoderIOError = False
#stepperIOError = False


# *********************** BEGIN CLASSES ******************************

# Moving average class
class MovingAverage():
    def __init__(self, windowsize):
        self.windowsize = windowsize
        self._setWindowSize(windowsize)
        self.accumValue = 0.0
        1.0
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
            if config.isAzRunning == True:    # azimuth
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
                        
                elif abs(variable.azAvgVelocity.computeAverage()) < 1.0 and isRunning(0) == False:
                    config.isAzRunning = False
                
                
            if config.isElRunning == True:    # elevation
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
                        
                elif abs(variable.elAvgVelocity.computeAverage()) < 1.0 and isRunning(1) == False:
                    config.isElRunning = False                

            # Get stepper and limit switch positions if neither motor running
            # I have to do this when both motors are stopped or motors will "pulse" at the 
            # motion thread update rate
            if config.isAzRunning == False and config.isElRunning == False:
                # Update azimuth
                config.azStepperPosn = getStepperPosn(0)
                config.azCCWLimit = isAzCCWLimit()
                config.azCWLimit = isAzCWLimit()                
                # Update elevation
                config.elStepperPosn = getStepperPosn(1)
                config.elUpLimit = isElUpLimit()
                config.elDownLimit = isElDownLimit()                
                
        
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
        while(self.running):
            time.sleep(self.delay)
            
            # Get position from encoders, but set flag if error
            try:
                config.azMountPosn, config.elMountPosn = getEncoders()
                # Update the gui encoder label
                #gui.updateEncoders()                 
            except ValueError:
                config.encoderIOError = True
            
            
            # Calculate velocities
            # the variable.xxx values can be converted to locals
            variable.azVelocity = (config.azMountPosn - self.lastAz)/self.delay
            variable.azAvgVelocity.addValue(variable.azVelocity)
            variable.elVelocity = (config.elMountPosn - self.lastEl)/self.delay
            variable.elAvgVelocity.addValue(variable.elVelocity)
            
            # Compute average velocities
            # the variable.xxx values can be converted to locals
            self.azAvgVel = variable.azAvgVelocity.computeAverage()
            self.elAvgVel = variable.elAvgVelocity.computeAverage()
            
            # Save encoder position to last position
            try:
                self.lastAz,self.lastEl = getEncoders()
            except ValueError:
                config.encoderIOError = True            
            
            # Update the process logfile
            #log.write(time.strftime('%H:%M:%S') + ',' +  str(config.azMountPosn) + ',' + str(config.elMountPosn) + ',' + str(config.azStepperPos) + ',' \
            #          + str(config.elStepperPos)+ ',' +  str(variable.azVelocity) + ',' + str(variable.elVelocity) + '\n')
            log.write(time.strftime('%H:%M:%S') + ',' +  str(config.azMountPosn) + ',' + str(config.elMountPosn) + ','  \
                      +  str(variable.azVelocity) + ',' + str(variable.elVelocity) + '\n')            

            # Logging
            #logging.debug("periodicThread() azVelocity %s", variable.azVelocity)
            #logging.debug("periodicThread() elVelocity %s", variable.elVelocity)
            #logging.debug("periodicThread() azAvgVel %s", self.azAvgVel)
            #logging.debug("periodicThread() elAvgVel %s", self.elAvgVel)            

            # Watch homing axes if homing
            if config.azHoming == True:
                watchHomingAxis(0)
            
            if config.elHoming == True:
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


# This will be completely removed later on
class Variables():
    def __init__(self):
        # Process variables
        #self.azHomingSpeed = 500;
        #self.elHomingSpeed = 500;
        #self.azSlewSpeed = 500;
        #self.elSlewSpeed = 500;
        #self.azAccel = 500;
        #self.elAccel = 500;
        #self.isAzRunning = False;
        #self.isElRunning = False;

        # Real-time values
        self.azVelocity = 0
        self.elVelocity = 0
        self.azAvgVelocity = MovingAverage(3)
        self.elAvgVelocity = MovingAverage(3)
        #self.azStepperPos = getStepperPosn(0)
        #self.elStepperPos = getStepperPosn(1)

        #self.azPos = 0
        #self.elPos = 0

        # homing flags
        #self.azHomed = False
        #self.azHoming = False
        #self.elHomed = False
        #self.elHoming = False 
        
# ************************* END CLASSES ******************************

# Sets inital values
def setInitialValues():
    setAzSpeed(250)
    setAzMaxSpeed(500)
    time.sleep(01.25)
    setAzAccel(500)
    time.sleep(0.25)
    setElSpeed(250)
    setElMaxSpeed(500)
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
        #logging.debug("sendMessage() messageQ: %s", next_message)
        bytesToSend = ConvertStringToBytes(next_message[1])
        reply = ""
        replyBytes = ""
        #logging.debug("sendMessage() raw: %s", bytesToSend)
        try:
            bus.write_i2c_block_data(i2c_address, config.I2C_CMD, bytesToSend)

            if i2c_address == config.STEPPERS_I2C_ADDR:
                replyBytes = bus.read_i2c_block_data(config.STEPPERS_I2C_ADDR, 0, 8)
            else:
                replyBytes = bus.read_i2c_block_data(config.ENCODERS_I2C_ADDR, 0, 16)

            reply = ConvertBytesToString(replyBytes)
        except IOError:
            if i2c_address == config.ENCODERS_I2C_ADDR:
                config.encoderIOError = True
                reply = ""
                logging.warn("sendMessage() - encoderIOError")
                
            elif i2c_address == config.STEPPERS_I2C_ADDR:
                config.stepperIOError = True
                reply = ""
                logging.warn("sendMessage() - config.stepperIOError") 
                
        else:   # Reset error flags if there was a good read
            if i2c_address == config.ENCODERS_I2C_ADDR:
                config.encoderIOError = False
            elif i2c_address == config.STEPPERS_I2C_ADDR:
                config.stepperIOError = False

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
    reply = sendMessage(1, cmd, config.ENCODERS_I2C_ADDR)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            #encPosList = config.azMountPosn, config.elMountPosn
            logging.warn("getEncoders() - IOError")
            encPosList = [0,0]
            
    #logging.debug("getEncoders() returned %s", encPosList)
   
    return tuple(encPosList)

# Gets the position of the encoder (in degrees) defined by axis
# axis 0 = azimuth
# axis 1 = elevation
def getEncodersDegrees(axis):
    # Assign raw counts to tuple
    encodersCounts = getEncoders()
    if axis == 0:
        encoderDegrees = encoderCountsToDegrees(0, encodersCounts[0])
    elif axis == 1:
        encoderDegrees = encoderCountsToDegrees(0, encodersCounts[1])

    return encoderDegrees    

# Gets the position of the stepper defined by axis
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
    #logging.debug("getStepperPosn() returned %s", result)
    
    if config.stepperIOError == False:
        try:
            return int(result)
        except ValueError:
            config.stepperIOError = True
            return 0
    

# Send a stepper motion command to the Uno
def sendStepperCommand(cmd):
    reply = sendMessage(1, cmd, config.STEPPERS_I2C_ADDR)
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
            config.azHoming = False
            config.azHomed = True
            config.isAzRunning = False
            zeroAzEncoder()
            zeroSteppers(0)
            logging.debug("watchHomingAxis() azimuth homed")
    elif axis == 1:
        if isElDownLimit() == True:
            config.elHoming = False
            config.elHomed = True
            config.isElRunning = False
            zeroElEncoder()
            zeroSteppers(1)
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
# Values were determined experimentally
def encoderCountsToDegrees(axis, pulses):
    if axis == 0:
        degreesPerPulse = 0.025356
    elif axis == 1:
        degreesPerPulse = 0.024473
    
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
    sendStepperCommand("3:0")
    config.isAzRunning = False
    config.isElRunning = False

def relMoveAz(distance):
    cmd = '1:' + str(distance)
    logging.debug("relMoveAz(%s)", distance)
    sendStepperCommand(cmd)
    time.sleep(0.5)
    config.isAzRunning = True

def relMoveEl(distance):
    cmd = '2:' + str(distance)
    logging.debug("relMoveEl(%s)", distance)
    sendStepperCommand(cmd)
    time.sleep(0.5)
    config.isElRunning = True

# Run at constant speed, based on last setSpeed()
# axis 0 = azimuth
# axis 1 = elevation
def runSpeed(axis):
    cmd = '20:' + str(axis)
    logging.debug("runSpeed() axis: %s", axis)
    sendStepperCommand(cmd)
    time.sleep(0.5)
    if axis == 0:
        config.isAzRunning = True
    elif axis == 1:
        config.isElRunning = True

def printEncodersCounts():
    print(getEncoders())

def printEncodersDegrees():
    print(getEncodersDegrees())

def stopAz():
    logging.debug("stopAz()")
    sendStepperCommand("4:0")
    config.isAzRunning = False

def stopEl():
    logging.debug("stopEl()")
    sendStepperCommand("5:0")
    config.isElRunning = False

# quickStop functions stop both axes at the same time
# because the both stepper enable pins are tied to the
# same output
def quickStopAz():
    logging.debug("quickStopAz()")
    sendStepperCommand("6:0")
    config.isAzRunning = False
    config.isElRunning = False
    config.azHoming = False

# quickStop functions stop both axes at the same time
# because the both stepper enable pins are tied to the
# same output
def quickStopEl():
    logging.debug("quickStopEl()")
    sendStepperCommand("7:0")
    config.isAzRunning = False
    config.isElRunning = False
    config.elHoming = False

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
    azCWLimit = sendStepperCommand("8:0")
    if azCWLimit.find('0') != -1:
        return True
    elif azCWLimit.find('1') != -1:
        return False

def printIsAzCWLimit():
    print(isAzCWLimit())

# returns True if limit made	
def isAzCCWLimit():
    azCCWLimit = sendStepperCommand("9:0")
    if azCCWLimit.find('0') != -1:
        return True
    elif azCCWLimit.find('1') != -1:
        return False

def printIsAzCCWLimit():
    print(isAzCCWLimit())

# returns True if limit made
def isElUpLimit():
    elUpLimit = sendStepperCommand("10:0")
    if elUpLimit.find('0') != -1:
        return True
    elif elUpLimit.find('1') != -1:
        return False

def printIsElUpLimit():
    print(isElUpLimit())

# returns True if limit made
def isElDownLimit():
    elDownLimit = sendStepperCommand("11:0")
    if elDownLimit.find('0') != -1:
        return True
    elif elDownLimit.find('1') != -1:
        return False

def printIsElDownLimit():
    print(isElDownLimit())


# 0 = azimuth
# 1 = elevation
def printStepPosnSteps(axis):
    print (getStepperPosn(axis))


# For now, let's home west
def homeAzimuth():
    config.azHoming = True
    logging.debug("homeAzimuth() executed")
    # slew west a long friggin way
    relMoveAz(-40000)

def homeElevation():
    config.elHoming = True
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
    reply = sendMessage(1, cmd, config.ENCODERS_I2C_ADDR)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = config.azMountPosn, config.elMountPosn

    logging.debug("zeroEncoders() returned %s", encPosList)
    
    return tuple(encPosList)    

def zeroAzEncoder():
    encPosList = []
    cmd = '2'
    reply = sendMessage(1, cmd, config.ENCODERS_I2C_ADDR)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = config.azMountPosn, config.elMountPosn

    logging.debug("zeroAzEncoder() returned %s", encPosList)
    
    return tuple(encPosList)    

def zeroElEncoder():
    encPosList = []
    cmd = '3'
    reply = sendMessage(1, cmd, config.ENCODERS_I2C_ADDR)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encPosList.append(int(i.rstrip('\x00')))
        except ValueError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = config.azMountPosn, config.elMountPosn

    logging.debug("zeroAzEncoder() returned %s", encPosList)
    
    return tuple(encPosList)

def zeroSteppers(axis):
    cmd = '21:' + str(axis)
    logging.debug("zeroSteppers() axis: %s", axis)
    sendStepperCommand(cmd)    

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

def shutdown():
    logging.debug("shutdown() entered")
    commThread.raise_exception()
    motionCheckThread.raise_exception()
    time.sleep(1.0)
    log.close()    
    
    # Close the damn window
    del window
    sys.exit()
    

if __name__ == "__main__":

    # loop to send message
    exit = False

    # Process logfile for live process parameters
    logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
    log = open(logfile, 'w')
    # Write the header
    #log.write("Time,azEncoder, elEncoder, azStepper, elStepper, azVel, elVel" + '\n')
    log.write("Time,azEncoder, elEncoder, azVel, elVel" + '\n')

    # This is the python logging module
    logging.basicConfig(filename='debug.log', filemode='w', \
                    format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)


    # Instantiate global variables
    # This will be deprecated soon
    variable = Variables()    

    # PySimpleGUI
    motion_layout =  [
                        [sg.Text('Slew')],
                        [sg.Button('SLEW_AZ'),sg.InputText('',size=(10,1),key='slewAz'),sg.Button('SLEW_EL'),sg.InputText('', size=(10,1),key='slewEl')],
                        [sg.Text('Jog Az')],             # assign a key to this and use it
                        [sg.Button('JOG_AZ_CCW'),sg.Button('JOG_AZ_CW')],       # assign a key to this and use it
                        [sg.Text('Jog El')],
                        [sg.Button('JOG_EL_DOWN'),sg.Button('JOG_EL_UP')],      # assign a key to this and use it
                        [sg.Text('Relative Move in Steps')],
                        [sg.Button('REL_AZ'),sg.InputText('',size=(10,1),key='relAz'),sg.Button('REL_EL'),sg.InputText('', size=(10,1),key='relEl')],
                        [sg.Text('Relative Move in Degrees')],
                        [sg.Button('REL_AZ_DEG'),sg.InputText('',size=(10,1),key='relAzDeg'),sg.Button('REL_EL_DEG'),sg.InputText('', size=(10,1),key='relElDeg')],
                        [sg.Text('Homing')],             # assign a key to this and use it
                        [sg.Button('HOME_AZ'),sg.Button('HOME_EL')],       # assign a key to this and use it                        
                        [sg.Text('Stop Motion')],
                        [sg.Button('STOP_AZ'),sg.Button('STOP_EL'),sg.Button('FSTOP_AZ'),sg.Button('FSTOP_EL')],
                        ]    

    position_layout =   [
                        #[sg.Text('Azimuth', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'azimuth')],
                        #[sg.Text('Elevation', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'elevation')],
                        [sg.Text('Az Encoder', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azEncoder'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azEncoderDeg')], 
                        [sg.Text('El Encoder', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elEncoder'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elEncoderDeg')],
                        [sg.Text('StepsAz', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsAz')],
                        [sg.Text('StepsEl', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsEl')],
                        [sg.Button('ZERO_AZ_ENC'),sg.Button('ZERO_EL_ENC')],
                        [sg.Button('ZERO_AZ_STEP'),sg.Button('ZERO_EL_STEP')],
                        ]    


    state_layout =      [
                        [sg.Text('State')],
                        [sg.Checkbox('AZ CCW Limit', key = 'azCCWLimit'), sg.Checkbox('AZ CW Limit', key = 'azCWLimit')],
                        [sg.Checkbox('EL UP Limit', key = 'elUPLimit'), sg.Checkbox('EL Down Limit', key = 'elDownLimit')],
                        ]


    layout =            [
                        [sg.Frame('Position', position_layout),sg.Frame('State', state_layout)],
                        [sg.Frame('Motion', motion_layout)],
                        ]    
    
    # Open the GUI
    sg.change_look_and_feel('GreenMono')
    window = sg.Window('Radio Telescope Control', layout)    


    # Start the comms thread after initialization
    commThread = periodicThread(1, "Thread-1")
    commThread.start()

    motionCheckThread = motionThread(1.5, "m_thread")
    motionCheckThread.start()

    # Set initial values - motor speeds, etc
    setInitialValues()    

    while True:
        event, values = window.Read(500) # Please try and use as high of a timeout value as you can
    
        # EVENTS
        if event is None or event == 'Quit':    # if user closed the window using X or clicked Quit button
            shutdown()
        if event == 'SLEW_AZ':
            setAzSpeed(values['slewAz'])
            runSpeed(0)
        if event == 'SLEW_EL':
            setElSpeed(values['slewEl'])
            runSpeed(1)
        if event == 'JOG_AZ_CW':
            slewEast()
        if event == 'JOG_AZ_CCW':
            slewWest()
        if event == 'JOG_EL_UP':
            slewNorth()
        if event == 'JOG_EL_DOWN':
            slewSouth()
        if event == 'ZERO_AZ_ENC':
            zeroAzEncoder()
        if event == 'ZERO_EL_ENC':
            zeroElEncoder()        
        if event == 'ZERO_AZ_STEP':
            zeroSteppers(0)
        if event == 'ZERO_EL_STEP':
            zeroSteppers(1)        
        if event == 'REL_AZ':
            relMoveAz(values['relAz'])
        if event == 'REL_EL':
            relMoveEl(values['relEl'])
        if event == 'REL_AZ_DEG':
            moveAzStepperDegrees(values['relAzDeg'])
        if event == 'REL_EL_DEG':
            moveElStepperDegrees(values['relElDeg'])
        if event == 'HOME_AZ':
            homeAzimuth()
        if event == 'HOME_EL':
            homeElevation()
        if event == 'STOP_AZ':
            stopAz()
        if event == 'STOP_EL':
            stopEl()
        if event == 'FSTOP_AZ':
            quickStopAz()
        if event == 'FSTOP_EL':
            quickStopEl()
            
        # UPDATES
        # Updates the information in the window
        # These values can be updated only on change
        window.Element('azEncoder').Update(config.azMountPosn)
        window.Element('elEncoder').Update(config.elMountPosn)
        window.Element('azEncoderDeg').Update(getEncodersDegrees(0))
        window.Element('elEncoderDeg').Update(getEncodersDegrees(1))        
        window.Element('stepsAz').Update(config.azStepperPosn)
        window.Element('stepsEl').Update(config.elStepperPosn)
        window.Element('azCCWLimit').Update(config.azCCWLimit)
        window.Element('azCWLimit').Update(config.azCWLimit)
        window.Element('elUPLimit').Update(config.elUpLimit)
        window.Element('elDownLimit').Update(config.elDownLimit)