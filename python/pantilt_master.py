#!/usr/bin/env python3

# pantilt_master.py
# Dave Cohen

import sys
import smbus
import time
#import os
import datetime
# For the tracking velocity calculations
import math
import ephem
import threading
import ctypes

# I love python logging
import logging

# Trying pysimpleGUI
import PySimpleGUI as sg

# This is where I will keep global data
import config

# Set up the i2c bus
bus = smbus.SMBus(1)

# Start a message queue
messageQ = []


# *********************** BEGIN CLASSES ******************************

# Moving average class
class MovingAverage():
    """Moving average class - keeps track of encoder velocities
    
    Methods
    -------
    preload()
    loads the window with a constant value
    
    addValue()
    adds a value to the window, discarding the first value added
    
    computeAverage()
    return the computed average of the window elements
    """
    def __init__(self, windowsize):
        self.windowsize = windowsize
        self._setWindowSize(windowsize)
        self.accumValue = 0.0
    
    def _setWindowSize(self, size):
        self.window = [0.0 for i in range(size)]
    
    # Preload window with a certain value
    def preload(self, value):
        """loads the window with a constant value """
        self.window = [value for i in range(self.windowsize)]
        
    def addValue(self,value):
        """adds a value to the window, discarding the first value added"""
        self.window.insert(0,value)
        self.window.pop()
        
    def computeAverage(self):
        """return the computed average of the window elements """
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
    
    def run(self):
        while self.running:
            time.sleep(self.delay)
            #logging.debug("motionThread() %s running", self.name)            
            
            #logging.debug("motionThread() azAvgVel %s", config.azAvgVelocity)
            #logging.debug("motionThread() elAvgVel %s", config.elAvgVelocity)
            #logging.debug("motionThread() config.azGeoPosn %s", config.azGeoPosn)
            #logging.debug("motionThread() config.azMountPosn %s", config.azMountPosn)
            #logging.debug("motionThread() config.elGeoPosn %s", config.elGeoPosn)
            #logging.debug("motionThread() config.elMountPosn %s", config.elMountPosn)
            #logging.debug("motionThread() getEncodersDegrees(1) %s", getEncodersDegrees(1))
            
            # Monitor the run status of the steppers,
            # updating the process varaibles
            # This will used to invoke jam detection later
            if config.isAzRunning == True:    # azimuth
                # Look to see if stopped
                if abs(config.azAvgVelocity) < 1.0 and isRunning(0) == True:
                    if self.azFailTiming == False:
                        self.azFailTiming = True
                        #logging.debug("motionThread.run() azFailTiming = True")
                        logging.debug("motionThread() azAvgVel too low: %s", config.azAvgVelocity)
                    elif self.azFailTiming == True:
                        self.azFailTiming == False
                        azJam()
                    
                elif abs(config.azAvgVelocity) > 1.0:   # running above min velocity
                    if self.azFailTiming == True:
                        self.azFailTiming = False
                        logging.debug("motionThread.run() azFailTiming = False")
                        
                elif abs(config.azAvgVelocity) < 1.0 and isRunning(0) == False:
                    config.isAzRunning = False
                
                
            if config.isElRunning == True:    # elevation
                # Look to see if stopped
                if abs(config.elAvgVelocity) < 1.0 and isRunning(1) == True:
                    if self.elFailTiming == False:
                        self.elFailTiming = True
                        #logging.debug("motionThread.run() elFailTiming = True")
                        logging.debug("motionThread() elAvgVel too low: %s", config.elAvgVelocity)
                    elif self.elFailTiming == True:
                        self.elFailTiming = False
                        elJam()
                        
                elif abs(config.elAvgVelocity) > 1.0:   # running above min velocity
                    if self.elFailTiming == True:
                        self.elFailTiming = False
                        logging.debug("motionThread.run() elFailTiming = False")
                        
                elif abs(config.elAvgVelocity) < 1.0 and isRunning(1) == False:
                    config.isElRunning = False                

            # Get stepper and limit switch positions if neither motor running
            # I have to do this when both motors are stopped or motors will "pulse" at the 
            # motion thread update rate
            # This is here to keep the GUI updated
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


# This thread is for updating live parameters - it is faster than the motion thread
# The logifile is updated here
class periodicThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        self.delay = 0.2
        self.lastAz, self.lastEl = getEncoders()

    def run(self):      
        while(self.running):
            time.sleep(self.delay)
            
            # Get position from encoders, but set flag if error
            try:
                config.azMountPosn, config.elMountPosn = getEncoders()
            except ValueError:
                config.encoderIOError = True
           
            # Calculate velocities
            azVel = 10 * (config.azMountPosn - self.lastAz)/self.delay
            elVel = 10 * (config.elMountPosn - self.lastEl)/self.delay
            
            # Add the calculated velocities to the 
            azMovingAverage.addValue(azVel)
            elMovingAverage.addValue(elVel)
            
            # Compute average velocities
            # the variable.xxx values can be converted to locals
            config.azAvgVelocity = azMovingAverage.computeAverage()
            config.elAvgVelocity = elMovingAverage.computeAverage()
            
            # Update the last encoders positions
            self.lastAz,self.lastEl = config.azMountPosn, config.elMountPosn

            # Update geographical positions
            updateGeoPosition(0)
            updateGeoPosition(1)
            
            # Update the process logfile
            log.write(time.strftime('%H:%M:%S') + ',' +  str(config.azMountPosn) + ',' + str(config.elMountPosn) + ','  \
                      +  str(config.azAvgVelocity) + ',' + str(config.elAvgVelocity) + '\n')            

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
                
            # Control closed-loop move
            if config.azMovingClosedLoop == True:
                watchEncoderMove(0)
            
            if config.elMovingClosedLoop == True:
                watchEncoderMove(1)            
                
                
    def print_time(self,threadName):
        print ("%s: %s") % (threadName, time.ctime(time.time()))

    # This does not seem to work
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
#class Variables():
#    def __init__(self):

        # Real-time values
#       self.azVelocity = 0
#       self.elVelocity = 0
        #self.azAvgVelocity = MovingAverage(5)
        #self.elAvgVelocity = MovingAverage(5)
        
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
            encPosList = config.azMountPosn, config.elMountPosn
            logging.debug("getEncoders() Value error")
        except AttributeError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = config.azMountPosn, config.elMountPosn
            logging.debug("getEncoders() Attribute error")
            
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
    
# Get the local sidereal time
def getCurrentLST():
    observer.date = ephem.now()
    return (observer.sidereal_time())

# Send a stepper motion command to the Uno
def sendStepperCommand(cmd):
    reply = sendMessage(1, cmd, config.STEPPERS_I2C_ADDR)
    return reply

# Process functions
def setAzSpeed(speed):
    cmd = "12" + ':' + str(speed)
    sendStepperCommand(cmd)
    logging.debug("setAzSpeed() %s", speed)

def setAzMaxSpeed(speed):
    cmd = "16" + ':' + str(speed)
    sendStepperCommand(cmd)
    logging.debug("setAzMaxSpeed() %s", speed)

def setElSpeed(speed):
    cmd = "13" + ':' + str(speed)
    sendStepperCommand(cmd)
    logging.debug("setElSpeed() %s", speed)

def setElMaxSpeed(speed):
    cmd = "17" + ':' + str(speed)
    sendStepperCommand(cmd)
    logging.debug("setElMaxSpeed() %s", speed)

def setAzAccel(accel):
    cmd = "14" + ':' + str(accel)
    sendStepperCommand(cmd)
    logging.debug("setAzAccel() %s", accel)

def setElAccel(accel):
    cmd = "15" + ':' + str(accel)
    sendStepperCommand(cmd)
    logging.debug("setAzAccel() %s", accel)

# Manages homing flags during homing process,
# and terminates when homw limit switch made
# When home limit made, terminate homing mode 
# If the encoder velocity approaches zero while
# homing, issue an estop and error
# 
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

# Given encoder degrees, return pulse equivalent
def encoderDegreesToCounts(axis, degrees):
    if axis == 0:
        pulsesPerDegree = 39.438397
    elif axis == 1:
        pulsesPerDegree = 40.861357
        
    pulses = float(degrees) * pulsesPerDegree
    return int(pulses)


# Set the geographical posiion between of the axis based
# on operator pointing to a known position
# This is a "one-star" calibration
# mount must be level for this to work
# position is in degrees
# axis 0 = azimuth
# axis 1 = elevation
def setGeoOffset(axis, position):
    if axis == 0:
        config.azGeoOffset = float(position) - encoderCountsToDegrees(0, config.azMountPosn)
        logging.debug("setGeoOffset(0) set offset to %s", config.azGeoOffset)
    elif axis == 1:
        config.elGeoOffset = float(position) - encoderCountsToDegrees(1, config.elMountPosn)
        logging.debug("setGeoOffset(1) set offset to %s", config.elGeoOffset)

# Get the geographical posiion between of the axis
# returns position in degrees
# axis 0 = azimuth
# axis 1 = elevation
def updateGeoPosition(axis):
    if axis == 0:
        config.azGeoPosn = encoderCountsToDegrees(0, config.azMountPosn) + config.azGeoOffset
    elif axis == 1:
        config.elGeoPosn = encoderCountsToDegrees(1, config.elMountPosn) + config.elGeoOffset        

# Returns tracking velocity for an axis
# axis 0 = azimuth
# axis 1 = elevation
# az = azimuth in degrees
# el = elevation (altitude) in degrees
# angle is either azimuth or elevation angle in degrees
# latitude is observer's latitude
# Returns velocity is steps per second
def getTrackVelocity(axis, az, el, latitude):
    # convert angles to radians
    latR = math.radians(latitude)
    azR = math.radians(az)
    elR = math.radians(el)
        
    # trigonometric functions
    SA = math.sin(azR)
    CA = math.cos(azR)
    SE = math.sin(elR)
    CE = math.cos(elR)
    SP = math.sin(latR)
    CP = math.cos(latR)
    
    # First, solve for HA and DEC (h2e.f)
    # HA, Dec as X, Y, Z
    X = -CA * CE * SP + SE * CP
    Y = -SA * CE
    Z = CA * CE * CP + SE * SP
    
    # To HA, Dec
    R = math.sqrt(X*X+Y*Y)
    
    if R == 0:
        HA = 0.0
    else:
        HA = math.atan2(Y,X)
        
    DEC = math.atan2(Z,R)
    
    #ha = math.degrees(HA)/15    # convert angle to time
    #haF = ephem.hours(HA)       # convert HA to time
    #dec = math.degrees(DEC)     # declination in degrees
    
    # Solve for velocities
    # trigonometric functions
    SH = math.sin(HA)       # already in radians
    CH = math.cos(HA)
    SD = math.sin(DEC)
    CD = math.cos(DEC)
    CHCD = CH * CD
    SDCP = SD * CP
    X = -CHCD * SP + SDCP
    Y = -SH * CD
    Z = CHCD * CP + SD * SP
    RSQ = X*X+Y*Y
    R = math.sqrt(RSQ)
       
    # Parallactic angle
    C = CD * SP - CH * SDCP
    S = SH * CP
    
    if (C*C+S*S) > 0:
        Q = math.atan2(S,C)
    else:
        Q = math.pi - HA
    
    PA = Q  # parallactic angle
    
    # Velocities (and accelerations, if I want them)
    TINY =  1e-30   # a very small number
    
    if RSQ < TINY:
        RSQ = TINY
        R = math.sqrt(RSQ)
    
    QD = -X*CP/RSQ
    AD = SP + Z * QD
    ED = CP*Y/R
    EDR = ED/R
    
    AZD = AD    # azimuth velocity
    ELD = ED    # elevation velocity
    
    # convert to degrees/sec
    azV = AZD * ((2*math.pi)/86400) * (360/(2*math.pi))
    elV = ELD * ((2*math.pi)/86400) * (360/(2*math.pi))
    
    # Convert to pulses per second, then return
    if axis == 0:
        return int(azV * 401.250)
    elif axis == 1:
        return int(elV * 769.166)
    else:
        return 0.0
    
    
    

# ****************** Command functions ***********************

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

# Triggers a closed-loop relative move by encoder couns
# Called by GUI pushbutton and text entry
def startEncoderMove(axis, distance):
    if axis == 0:
        config.azMovingClosedLoop = True
        config.azDistance = int(distance)
    elif axis == 1:
        config.elMovingClosedLoop = True
        config.elDistance = int(distance)
        
    logging.debug("startEncoderMove(%s, %s) invoked", axis, distance)
    

# Maintains a closed-loop relative move by encoder counts
# Called by the periodic thread if xxMovingClosedLoop flag is set
def watchEncoderMove(axis):
    if axis == 0:
        config.azDistanceTogo = config.azDistance - config.azMountPosn
        
        # Need to determine direction based on sign
        if config.azDistance - config.azMountPosn > 0:
            polarity = 1
        else:
            polarity = -1
    
        if abs(config.azDistanceTogo) > config.endpointFarDistance:
            azSpeed = config.azEndpointFarSpeed * polarity
            # Starts the motion at this speed
            if config.azInFarApproach == False:
                logging.debug("watchEncoderMove(%s) set far approach run", axis)
                # Start slew at this speed
                setAzSpeed(azSpeed)
                runSpeed(0)
            config.azInFarApproach = True
        elif abs(config.azDistanceTogo) > config.endpointNearDistance:
            azSpeed = config.azEndpointNearSpeed * polarity
            if config.azInNearApproach == False:
                logging.debug("watchEncoderMove(%s) set near approach run", axis)
                # Start slew at this speed
                setAzSpeed(azSpeed)
                runSpeed(0)                        
            config.azInNearApproach = True
        elif abs(config.azDistanceTogo) > config.endpointVeryNearDistance:
            azSpeed = config.azEndpointVeryNearSpeed * polarity
            if config.azInVeryNearApproach == False:
                logging.debug("watchEncoderMove(%s) set very near approach run", axis)
                setAzSpeed(azSpeed)
                runSpeed(0)                        
            config.azInVeryNearApproach = True
        elif abs(config.azDistanceTogo) < config.endpointDeadband:
            logging.debug("watchEncoderMove(%s) stopped in deadband", axis)
            stopAz()
            # Reset the flags
            config.azInFarApproach = False
            config.azInNearApproach = False
            config.azInVeryNearApproach = False
            config.azMovingClosedLoop = False    

    if axis == 1:
        config.elDistanceTogo = config.elDistance - config.elMountPosn
        
        # Need to determine direction based on sign
        if config.elDistance - config.elMountPosn > 0:
            polarity = 1
        else:
            polarity = -1
    
        if abs(config.elDistanceTogo) > config.endpointFarDistance:
            elSpeed = config.elEndpointFarSpeed * polarity
            # Starts the motion at this speed
            if config.elInFarApproach == False:
                logging.debug("watchEncoderMove(%s) set far approach run", axis)
                # Start slew at this speed
                setElSpeed(elSpeed)
                runSpeed(1)
            config.elInFarApproach = True
        elif abs(config.elDistanceTogo) > config.endpointNearDistance:
            elSpeed = config.elEndpointNearSpeed * polarity
            if config.elInNearApproach == False:
                logging.debug("watchEncoderMove(%s) set near approach run", axis)
                # Start slew at this speed
                setElSpeed(elSpeed)
                runSpeed(1)                        
            config.elInNearApproach = True
        elif abs(config.elDistanceTogo) > config.endpointVeryNearDistance:
            elSpeed = config.elEndpointVeryNearSpeed * polarity
            if config.elInVeryNearApproach == False:
                logging.debug("watchEncoderMove(%s) set very near approach run", axis)
                setElSpeed(elSpeed)
                runSpeed(1)                        
            config.elInVeryNearApproach = True
        elif abs(config.elDistanceTogo) < config.endpointDeadband:
            logging.debug("watchEncoderMove(%s) stopped in deadband", axis)
            stopEl()
            # Reset the flags
            config.elInFarApproach = False
            config.elInNearApproach = False
            config.elInVeryNearApproach = False
            config.elMovingClosedLoop = False




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

def stopAz():
    logging.debug("stopAz()")
    sendStepperCommand("4:0")
    config.isAzRunning = False

def stopEl():
    logging.debug("stopEl()")
    sendStepperCommand("5:0")
    config.isElRunning = False

def quickStopAz():
    logging.debug("quickStopAz()")
    sendStepperCommand("6:0")
    config.isAzRunning = False
    #config.isElRunning = False
    config.azHoming = False

def quickStopEl():
    logging.debug("quickStopEl()")
    sendStepperCommand("7:0")
    #config.isAzRunning = False
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

# returns True if limit made	
def isAzCCWLimit():
    azCCWLimit = sendStepperCommand("9:0")
    if azCCWLimit.find('0') != -1:
        return True
    elif azCCWLimit.find('1') != -1:
        return False

# returns True if limit made
def isElUpLimit():
    elUpLimit = sendStepperCommand("10:0")
    if elUpLimit.find('0') != -1:
        return True
    elif elUpLimit.find('1') != -1:
        return False


# returns True if limit made
def isElDownLimit():
    elDownLimit = sendStepperCommand("11:0")
    if elDownLimit.find('0') != -1:
        return True
    elif elDownLimit.find('1') != -1:
        return False

# For now, let's home west
def homeAzimuth():
    config.azHoming = True
    logging.debug("homeAzimuth() executed")
    setAzMaxSpeed(250)
    # slew west a long friggin way
    relMoveAz(-40000)

def homeElevation():
    config.elHoming = True
    logging.debug("homeElevation() executed")
    setElMaxSpeed(250)
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
    quickStopAz()
    
    # Reset flags so encoder move will work again
    config.azInFarApproach = False
    config.azInNearApproach = False
    config.azInVeryNearApproach = False
    config.azMovingClosedLoop = False    
    
    
def elJam():
    logging.debug("elJam() timeout")
    quickStopEl()
    
    # Reset flags so encoder move will work again
    config.elInFarApproach = False
    config.elInNearApproach = False
    config.elInVeryNearApproach = False
    config.elMovingClosedLoop = False    

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
    window.Close()
    #del window
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

    # Pyephem
    observer = ephem.Observer()
    observer.lon = '-77:55:27'
    observer.lat = '37:47:26'
    observer.elevation = 116   # i'm calling this heightASL from now on    


    # Instantiate moving averages for jam detection velocity calculations
    azMovingAverage = MovingAverage(10)
    elMovingAverage = MovingAverage(10)

    # PySimpleGUI
    motion_layout =  [
                        [sg.Text('Slew')],
                        [sg.Button('SLEW_AZ'),sg.InputText('',size=(10,1),key='slewAz'),sg.Button('SLEW_EL'),sg.InputText('', size=(10,1),key='slewEl')],
                        [sg.Text('Relative Open Loop Move in Steps')],
                        [sg.Button('REL_AZ'),sg.InputText('',size=(10,1),key='relAz'),sg.Button('REL_EL'),sg.InputText('', size=(10,1),key='relEl')],
                        [sg.Text('Relative Open Loop Move in Stepper Degrees')],
                        [sg.Button('REL_AZ_DEG'),sg.InputText('',size=(10,1),key='relAzDeg'),sg.Button('REL_EL_DEG'),sg.InputText('', size=(10,1),key='relElDeg')],
                        [sg.Text('Absolute Closed Loop Move in Encoder Counts')],
                        [sg.Button('ABS_AZ_ENC'),sg.InputText('',size=(10,1),key='absAzEnc'),sg.Button('ABS_EL_ENC'),sg.InputText('', size=(10,1),key='absElEnc')],
                        [sg.Text('Absolute Closed Loop Move in Encoder Degrees')],
                        [sg.Button('DEG_AZ_ENC'),sg.InputText('',size=(10,1),key='degAzEnc'),sg.Button('DEG_EL_ENC'),sg.InputText('', size=(10,1),key='degElEnc')],
                        [sg.Text('Homing')],
                        [sg.Button('HOME_AZ'),sg.Button('HOME_EL')],                        
                        [sg.Text('Stop Motion')],
                        [sg.Button('STOP_AZ'),sg.Button('STOP_EL'),sg.Button('FSTOP_AZ'),sg.Button('FSTOP_EL')],
                        ]    

    position_layout =   [
                        [sg.Text('Az Encoder', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azEncoder'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azEncoderDeg')], 
                        [sg.Text('El Encoder', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elEncoder'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elEncoderDeg')],
                        [sg.Text('StepsAz', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsAz')],
                        [sg.Text('StepsEl', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsEl')],
                        [sg.Text('Az Vel', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'azVel')],
                        [sg.Text('El Vel', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'elVel')],
                        [sg.Text('Az Geo', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'azGeoPosn')],
                        [sg.Text('El Geo', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'elGeoPosn')],
                        [sg.Button('ZERO_AZ_ENC'),sg.Button('ZERO_EL_ENC')],
                        [sg.Button('ZERO_AZ_STEP'),sg.Button('ZERO_EL_STEP')],
                        ]    


    state_layout =      [
                        [sg.Checkbox('AZ CCW Limit', key = 'azCCWLimit'), sg.Checkbox('AZ CW Limit', key = 'azCWLimit')],
                        [sg.Checkbox('EL UP Limit', key = 'elUPLimit'), sg.Checkbox('EL Down Limit', key = 'elDownLimit')],
                        [sg.Checkbox('AZ Running', key = 'azRunning'), sg.Checkbox('EL Running', key = 'elRunning')],
                        [sg.Button('AZ_SPD'),sg.InputText('',size=(5,1),key='azSpeed'),sg.Button('AZ_MAX'),sg.InputText('',size=(5,1),key='azMax'),sg.Button('AZ_ACCEL'),sg.InputText('',size=(5,1),key='azAccel')],
                        [sg.Button('EL_SPD'),sg.InputText('',size=(5,1),key='elSpeed'),sg.Button('EL_MAX'),sg.InputText('',size=(5,1),key='elMax'),sg.Button('EL_ACCEL'),sg.InputText('',size=(5,1),key='elAccel')],
                        ]

    celestial_layout =  [
                        [sg.Button('SET_AZ_GEO'),sg.InputText('',size=(10,1),key='azGeoInput'),sg.Button('SET_EL_GEO'),sg.InputText('', size=(10,1),key='elGeoInput')],
                        [sg.Text('LST', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue', key = 'localSiderealTime')],
                        [sg.Text('TrackVel:', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azTrackVel'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elTrackVel')],
                        ]


    layout =            [
                        [sg.Frame('Position', position_layout),sg.Frame('State', state_layout)],
                        [sg.Frame('Motion', motion_layout),sg.Frame('Celestial', celestial_layout)],
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

        if event == 'ZERO_AZ_ENC':
            zeroAzEncoder()

        if event == 'ZERO_EL_ENC':
            zeroElEncoder()        

        if event == 'ZERO_AZ_STEP':
            zeroSteppers(0)

        if event == 'ZERO_EL_STEP':
            zeroSteppers(1)        

        if event == 'REL_AZ':
            setAzMaxSpeed(500)
            relMoveAz(values['relAz'])

        if event == 'REL_EL':
            setElMaxSpeed(500)
            relMoveEl(values['relEl'])

        if event == 'REL_AZ_DEG':
            setAzMaxSpeed(500)
            moveAzStepperDegrees(values['relAzDeg'])

        if event == 'REL_EL_DEG':
            setElMaxSpeed(500)
            moveElStepperDegrees(values['relElDeg'])

        if event == 'ABS_AZ_ENC':
            setAzMaxSpeed(500)
            startEncoderMove(0, values['absAzEnc'])

        if event == 'ABS_EL_ENC':
            setElMaxSpeed(500)
            startEncoderMove(1, values['absElEnc'])

        if event == 'DEG_AZ_ENC':
            setAzMaxSpeed(500)
            startEncoderMove(0, encoderDegreesToCounts(0,values['degAzEnc']))

        if event == 'DEG_EL_ENC':
            setElMaxSpeed(500)
            startEncoderMove(1, encoderDegreesToCounts(0,values['degElEnc']))        

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

        # Speeds and accelerations
        if event == 'AZ_SPD':
            setAzSpeed(values['azSpeed'])

        if event == 'AZ_MAX':
            setAzMaxSpeed(values['azMax'])

        if event == 'AZ_ACCEL':
            setAzAccel(values['azAccel'])            

        if event == 'EL_SPD':
            setAzSpeed(values['elSpeed'])

        if event == 'EL_MAX':
            setAzMaxSpeed(values['elMax'])

        if event == 'EL_ACCEL':
            setAzAccel(values['elAccel'])        

        if event == 'SET_AZ_GEO':
            setGeoOffset(0, values['azGeoInput'])

        if event == 'SET_EL_GEO':
            setGeoOffset(1, values['elGeoInput'])        
           
            
        # UPDATES
        # Updates the information in the text boxes
        # These values can be updated only on change
        window.Element('azEncoder').Update(config.azMountPosn)
        window.Element('elEncoder').Update(config.elMountPosn)
        window.Element('azEncoderDeg').Update(getEncodersDegrees(0))
        window.Element('elEncoderDeg').Update(getEncodersDegrees(1))   # does not agree with elGeoPosn below, looks more accurate     
        window.Element('stepsAz').Update(config.azStepperPosn)
        window.Element('stepsEl').Update(config.elStepperPosn)
        window.Element('azCCWLimit').Update(config.azCCWLimit)
        window.Element('azCWLimit').Update(config.azCWLimit)
        window.Element('elUPLimit').Update(config.elUpLimit)
        window.Element('elDownLimit').Update(config.elDownLimit)
        window.Element('azRunning').Update(config.isAzRunning)
        window.Element('elRunning').Update(config.isElRunning)
        window.Element('azVel').Update(config.azAvgVelocity)
        window.Element('elVel').Update(config.elAvgVelocity)
        window.Element('azGeoPosn').Update(config.azGeoPosn)
        window.Element('elGeoPosn').Update(config.elGeoPosn)            # see above elEncoderDeg
        window.Element('localSiderealTime').Update(str(getCurrentLST()))
        # Hard-coding the latitude for now
        window.Element('azTrackVel').Update(getTrackVelocity(0, config.azGeoPosn, config.elGeoPosn, 37.79))
        window.Element('elTrackVel').Update(getTrackVelocity(1, config.azGeoPosn, config.elGeoPosn, 37.79))