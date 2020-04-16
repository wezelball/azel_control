#!/usr/bin/env python3

# pantilt_master.py
# Dave Cohen

import sys
import smbus
import time
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

# Acceleration ramp for closed-loop moves
class AccelRamp():
    # Constructor
    def __init__(self, axis, dT):
        self.axis = axis    # 0 or 1
        self.deltaT = dT    # periodic thread update time
        self.isRampComplete = False
        self.newSpeed = 0.0
        self.currentSpeed = 0.0
        self.finalSpeed = 0.0
        self.deltaV = 0.0
        self.cmd = '20:' + str(self.axis)
        self.enabled = False
        
    # Set the ramp time for a specific move
    def setRampTime(self, time):
        self.rampTime = time
        logging.debug("AccelRamp() setRampTime(): %f", self.rampTime)
    
    # Set the final speed for a specific move    
    def setFinalSpeed(self, speed):
        self.finalSpeed = speed
        logging.debug("AccelRamp() setFinalSpeed(): %f", self.finalSpeed)
    
    # We must know the current speed
    def setCurrentSpeed(self, speed):
        self.currentSpeed = speed
        logging.debug("AccelRamp() setCurrentSpeed(): %f", self.currentSpeed)
        
    # Is the ramp generator enabled?
    def isEnabled(self):
        # TODO - there's a cooler way to do this
        if self.enabled == True:
            #logging.debug("AccelRamp() isEnabled = True")
            return True
        else:
            #logging.debug("AccelRamp() isEnabled = False")
            return False
    
    # Is the ramp complete?
    def isComplete(self):
        # TODO - there's a cooler way to do this
        if self.isRampComplete == True:
            #logging.debug("AccelRamp() isComplete = True")
            return True
        else:
            #logging.debug("AccelRamp() isComplete = False")
            return False
    
    def enable(self):
        logging.debug("AccelRamp() enabled")
        self.enabled = True
        
    def disable(self):
        self.enabled = False
        self.isRampComplete = False
        logging.debug("AccelRamp() disabled")

    # Calculate deltaV, the speed increment
    def getDeltaV(self):
        return (self.finalSpeed - self.currentSpeed)*self.deltaT/self.rampTime

    # perform a speed increment 
    def update(self):
        if self.enabled == True:
            # Get the calculated speed increment
            self.deltaV = self.getDeltaV()
            # Update the speed
            self.newSpeed += self.deltaV
            logging.debug("AccelRamp.update() newSpeed : %f", self.newSpeed)
            
            # Make sure we don't exceed the speed limit
            if abs(self.newSpeed) >= abs(self.finalSpeed):
                self.newSpeed = self.finalSpeed
                self.isRampComplete = True
            else:
                # Update the stepper speed
                if self.axis == 0:
                    setAzSpeed(self.newSpeed)
                elif self.axis == 1:
                    setElSpeed(self.newSpeed)
                    
                # Tell the stepper to run at speed here
                sendStepperCommand(self.cmd)
                logging.debug("AccelRamp() sendStepperCommand(): %s", self.cmd)
        


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
        self.accumValue = 0.0
        for i in self.window:
            self.accumValue += i
            
        return self.accumValue/len(self.window)

# Checks for failure of motion in either axis
# It's slower than the periodic thread
class motionThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        self.delay = 1.5
        self.azFailTiming = False
        self.elFailTiming = False
        self.azStartCount = 0
        self.elStartCount = 0
        self.azPulseAge = 0.0
        self.elPulseAge = 0.0
        self.azLastPosn = config.azMountPosn
        self.elLastPosn = config.elMountPosn
    
    def run(self):
        while self.running:
            time.sleep(self.delay)
            
            # Update the state of the startup flags
            # If an axis has been told to run, we will give it several
            # iterations of the motion thread before setting the
            # startup complete flag
            # Then zero speed will be checked

            # Azimuth
            if config.isAzRunning == True:
                self.azStartCount += 1
                if self.azStartCount > 3:
                    config.azStartupComplete = True
                    #logging.debug("motionThread() azStartup complete")
                
            else:
                self.azStartCount = 0

            # Elevation
            if config.isElRunning == True:
                self.elStartCount += 1
                if self.elStartCount > 3:
                    config.elStartupComplete = True
                    #logging.debug("motionThread() elStartup complete")
                
            else:
                self.azStartCount = 0            

            # Jam detection has been the causes of many issues, due to false triggering
            # I'm rethinkng the way to accomplish this.
            #
            # When an axis is running, encoder counts will change periodically. The rate
            # of change is based on the velocity of the axis, which may also have a rate
            # of change, due to accel/decel. It's easy enough to detect a change, but the
            # main problem will be in tracking, when there is a long time between pulses.
            # In any case, there is an expected time between counts.  The startup complete
            # can handle the problem of accel.
            #
            # So there needs to be a pulse age variable, which is based on the speed that
            # we are running. After startupComplete (should really be rampComplete), we
            # increment the pulse age age variable by the value of the motionThread interval.
            # If that time exceeds the expected time by some value, we generate a jam signal
            
            if config.isAzRunning == True and config.azStartupComplete == True:
                if self.azLastPosn == config.azMountPosn:  
                    self.azPulseAge += self.delay   # this is the interval of this thread
                    logging.debug("motionThread() azPulseAge extended: %d", self.azPulseAge)
                else:
                    self.azLastPosn = config.azMountPosn
                    self.azPulseAge = 0.0
                    logging.debug("motionThread() azPulseAge reset")
                    
            # TODO - This code is not stable, working to replace.  Jam detection is now disabled
            
            """        
            if config.isElRunning == True and config.elStartupComplete == True:
                if self.elLastPosn == config.elMountPosn:  
                    self.elPulseAge += self.delay   # this is the interval of this thread
                    logging.debug("motionThread() elPulseAge extended: %d", self.elPulseAge)
                else:
                    self.elLastPosn = config.elMountPosn
                    self.elPulseAge = 0.0
                    logging.debug("motionThread() elPulseAge reset")


            # Monitor the run status of the steppers,
            # updating the process varaibles
            # This is used to invoke jam detection
            # Does not work for tracking, speed too slow  
            if config.isAzRunning == True and config.isTracking == False:    # azimuth
                # Look to see if stopped
                if abs(config.azAvgVelocity) < 1.0 and config.azStartupComplete == True:
                    if self.azFailTiming == False:
                        self.azFailTiming = True
                        #logging.debug("motionThread.run() azFailTiming = True")
                        logging.debug("motionThread() azAvgVel too low: %s", config.azAvgVelocity)
                    elif self.azFailTiming == True:
                        self.azFailTiming == False
                        azJam()
                
                # Reset the fail timeout if speed above minimum    
                elif abs(config.azAvgVelocity) > 1.0:   # running above min velocity
                    if self.azFailTiming == True:
                        self.azFailTiming = False
                        logging.debug("motionThread.run() azFailTiming = False")
                        
                # If below min speed and stepper controller reports not running,
                # reset the motor run state
                elif abs(config.azAvgVelocity) < 1.0 and isRunning(0) == False:
                    config.isAzRunning = False
                
                
            if config.isElRunning == True and config.isTracking == False:    # elevation
                # Look to see if stopped, after 
                if abs(config.elAvgVelocity) < 1.0 and config.elStartupComplete == True:
                    if self.elFailTiming == False:
                        self.elFailTiming = True
                        #logging.debug("motionThread.run() elFailTiming = True")
                        logging.debug("motionThread() elAvgVel too low: %s", config.elAvgVelocity)
                    elif self.elFailTiming == True:
                        self.elFailTiming = False
                        elJam()
                        
                # Reset the fail timeout if speed above minimum    
                elif abs(config.elAvgVelocity) > 1.0:   # running above min velocity
                    if self.elFailTiming == True:
                        self.elFailTiming = False
                        logging.debug("motionThread.run() elFailTiming = False")

                # If below min speed and stepper controller reports not running,
                # reset the motor run state                        
                elif abs(config.elAvgVelocity) < 1.0 and isRunning(1) == False:
                    config.isElRunning = False                
            """
            
            # Get stepper and limit switch positions if neither motor running
            # I have to do this when both motors are stopped or motors will "pulse" at the 
            # motion thread update rate
            # This is here to keep the GUI updated
            #if config.isAzRunning == False and config.isElRunning == False:
            if config.isAzRunning == False and config.isElRunning == False or config.isTracking == True:
                # Update azimuth
                config.azStepperPosn = getStepperPosn(0)
                config.azCCWLimit = isAzCCWLimit()
                config.azCWLimit = isAzCWLimit()                
                # Update elevation
                config.elStepperPosn = getStepperPosn(1)
                config.elUpLimit = isElUpLimit()
                config.elDownLimit = isElDownLimit()
                
            
            # Let's also update tracking velocities here, the periodic thread is so busy
            if config.isTracking == True:
                # Add the thread time here
                config.trackingAge += self.delay
                
                # Now check to see if an update is required, the tracking velocity
                # is too old - 90 sec for now
                if config.trackingAge >= 90:
                    config.trackingAge = 0
                    setAzSpeed(getTrackVelocity(0, config.azGeoPosn, config.elGeoPosn, 37.79))
                    setElSpeed(getTrackVelocity(1, config.azGeoPosn, config.elGeoPosn, 37.79))
                    logging.debug("motionThread.run() Tracking velocities updated")
                    
        
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
# The logfile is updated here
class periodicThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.running = True
        self.delay = 0.2
        self.lastAz, self.lastEl = getEncoders()
        self.azTrackingTicks = 0   # number of times the periodic thread iterated w/o encoder change
        self.elTrackingTicks = 0
        self.azDeltaT = 0.0         # time elapsed between encoder value changes
        self.elDeltaT = 0.0         # time elapsed between encoder value changes

    def run(self):      
        while(self.running):
            time.sleep(self.delay)
            
            # Get position from encoders, but set flag if error
            try:
                config.azMountPosn, config.elMountPosn = getEncoders()
            except ValueError:
                config.encoderIOError = True
           
            # Calculate velocities - current units are (seconds of arc)/sec
            # TODO - The velocities during tracking are not accurate because there
            # is an unknown number of interations of the periodic thread that occur
            # between encoder changes.  To fix this, I need to count the number
            # of iterations and multiply by self.delay, and divide the difference in
            # encoder counts by that value.  Need a separate value for az and el
            
            # No change in encoder counts during thread, increment the tracking ticks
            if config.isTracking == True and config.azMountPosn == self.lastAz:
                self.azTrackingTicks += 1
            if config.isTracking == True and config.elMountPosn == self.lastEl:
                self.elTrackingTicks += 1            
                        
            # Tracking, only calculate velocities if encoder value changes
            if config.isTracking == True and config.azMountPosn != self.lastAz:
                self.azTrackingTicks += 1
                # Total time elaped between last encoder count
                self.azDeltaT = self.azTrackingTicks * self.delay   
                azVel = (encoderCountsToDegrees(0, (config.azMountPosn - self.lastAz))/self.azDeltaT) * 3600.0
                # Add the calculated velocities to the moving average
                azMovingAverage.addValue(azVel)
                self.lastAz = config.azMountPosn                
            # Tracking, only calculate velocities if encoder value changes
            if config.isTracking == True and config.elMountPosn != self.lastEl:
                self.elTrackingTicks += 1
                # Total time elaped between last encoder count
                self.elDeltaT = self.elTrackingTicks * self.delay                 
                elVel = (encoderCountsToDegrees(1, (config.elMountPosn - self.lastEl))/self.elDeltaT) * 3600.0
                # Add the calculated velocities to the moving average
                elMovingAverage.addValue(elVel)
                self.lastEl = config.elMountPosn
            # Not tracking, calculate velocities and update averages
            if config.isTracking == False:
                azVel = (encoderCountsToDegrees(0, (config.azMountPosn - self.lastAz))/self.delay) * 3600.0
                # Add the calculated velocities to the moving average
                azMovingAverage.addValue(azVel)
                self.lastAz = config.azMountPosn   
                elVel = (encoderCountsToDegrees(1, (config.elMountPosn - self.lastEl))/self.delay) * 3600.0
                # Add the calculated velocities to the moving average
                elMovingAverage.addValue(elVel)
                self.lastEl = config.elMountPosn                

            
            # Compute average velocities
            config.azAvgVelocity = azMovingAverage.computeAverage()
            config.elAvgVelocity = elMovingAverage.computeAverage()
            
            # Update the last encoders positions
            #self.lastAz,self.lastEl = config.azMountPosn, config.elMountPosn

            # Update geographical positions
            # Do I need to maintain these variables?
            config.azGeoPosn = getGeoPosition(0)
            config.elGeoPosn = getGeoPosition(1)
            
            # Update RA, dec
            config.rightAscension, config.declination = observer.radec_of(math.radians(config.azGeoPosn), math.radians(config.elGeoPosn))
            #logging.debug("periodicThread() config.rightAscension %s", config.rightAscension)
            
            # Update the process logfile
            log.write(time.strftime('%H:%M:%S') + ',' +  str(config.azMountPosn) + ',' + str(config.elMountPosn) + ','  \
                      +  str(config.azAvgVelocity) + ',' + str(config.elAvgVelocity) + '\n')            

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
                
            # Maintain accel ramps for closed-loop moves
            if azAccelRamp.isEnabled():
                azAccelRamp.update()
            if azAccelRamp.isComplete():
                azAccelRamp.disable()

            if elAccelRamp.isEnabled():
                elAccelRamp.update()
            if elAccelRamp.isComplete():
                elAccelRamp.disable()
                
            
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

        
# ************************* END CLASSES ******************************

# Sets inital motion values
def setInitialValues():
    setAzSpeed(config.azSpeed)
    setAzMaxSpeed(config.azMaxSpeed)
    time.sleep(0.25)
    setAzAccel(config.azAccel)
    time.sleep(0.25)
    setElSpeed(config.elSpeed)
    setElMaxSpeed(config.elMaxSpeed)
    time.sleep(0.25)
    setElAccel(config.elAccel)

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
                config.encoderi2CErrorCount += 1
                reply = ""
                logging.warn("sendMessage() - encoderI2CError %d", config.encoderi2CErrorCount)
                #logging.debug("setAzSpeed() %s", speed)
                
            elif i2c_address == config.STEPPERS_I2C_ADDR:
                config.stepperIOError = True
                config.stepperi2CErrorCount += 1
                reply = ""
                logging.warn("sendMessage() - stepperI2CError %d", config.stepperi2CErrorCount) 
                
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
            #logging.debug("getEncoders() Value error")
        except AttributeError:
            # Bus error, assign last known values of encoder
            # positions - this is ugly and might cause issues
            encPosList = config.azMountPosn, config.elMountPosn
            #logging.debug("getEncoders() Attribute error")
            
    #logging.debug("getEncoders() returned %s", encPosList)
   
    return tuple(encPosList)

# Gets the position of the encoder (in degrees) defined by axis
# axis 0 = azimuth
# axis 1 = elevation

# TODO - I could probably eliminate this by calling 
# encoderCountsToDegrees directly instead
# This seems redundant
def getEncodersDegrees(axis):
    if axis == 0:
        encoderDegrees = encoderCountsToDegrees(0, config.azMountPosn)
    elif axis == 1:
        encoderDegrees = encoderCountsToDegrees(1, config.elMountPosn)

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
    #config.azSpeed = speed
    logging.debug("setAzSpeed() %s", speed)

def setAzMaxSpeed(speed):
    cmd = "16" + ':' + str(speed)
    sendStepperCommand(cmd)
    #config.azMaxSpeed = speed
    logging.debug("setAzMaxSpeed() %s", speed)

def setElSpeed(speed):
    cmd = "13" + ':' + str(speed)
    sendStepperCommand(cmd)
    #config.elSpeed = speed
    logging.debug("setElSpeed() %s", speed)

def setElMaxSpeed(speed):
    cmd = "17" + ':' + str(speed)
    sendStepperCommand(cmd)
    #config.elMaxSpeed = speed
    logging.debug("setElMaxSpeed() %s", speed)

def setAzAccel(accel):
    cmd = "14" + ':' + str(accel)
    sendStepperCommand(cmd)
    config.azAccel = accel
    logging.debug("setAzAccel() %s", accel)

def setElAccel(accel):
    cmd = "15" + ':' + str(accel)
    sendStepperCommand(cmd)
    config.elAccel = accel
    logging.debug("setAzAccel() %s", accel)

# Manages homing flags during homing process,
# and terminates when home limit switch made
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

# Updates the geographical posiions of the axes
# does not return a value
# axis 0 = azimuth
# axis 1 = elevation
#
# TODO - get rid of this and use getGeoPosition()
def updateGeoPosition(axis):
    if axis == 0:
        config.azGeoPosn = encoderCountsToDegrees(0, config.azMountPosn) + config.azGeoOffset
    elif axis == 1:
        config.elGeoPosn = encoderCountsToDegrees(1, config.elMountPosn) + config.elGeoOffset        

# Gets the geographical posiions of an axis
# Returns position in degrees
# axis 0 = azimuth
# axis 1 = elevation
def getGeoPosition(axis):
    if axis == 0:
        return encoderCountsToDegrees(0, config.azMountPosn) + config.azGeoOffset
    elif axis == 1:
        return encoderCountsToDegrees(1, config.elMountPosn) + config.elGeoOffset


# Returns tracking velocity for an axis
# axis 0 = azimuth
# axis 1 = elevation
# az = azimuth in degrees
# el = elevation (altitude) in degrees
# angle is either azimuth or elevation angle in degrees
# latitude is observer's latitude
# Returns velocity is steps per second
# I have tested this algorithm several times
# against Mel Bartels Scope To Sky Calculator
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
    #EDR = ED/R
    
    AZD = AD    # azimuth velocity
    ELD = ED    # elevation velocity
    
    # convert to degrees/sec
    azV = AZD * ((2*math.pi)/86400) * (360/(2*math.pi))
    elV = ELD * ((2*math.pi)/86400) * (360/(2*math.pi))
    
    # convert to seconds of arc per second, so 
    
    # Convert to pulses per second, then return
    if axis == 0:
        return (azV * 401.250)
        # Debug - show arcsec/second
        #return azV * 3600
    elif axis == 1:
        return (elV * 769.166)
        # Debug - show arcsec/second
        #return elV * 3600

# ****************** Command functions ***********************

def stopAllSlew():
    logging.debug("stopAllSlew()")
    sendStepperCommand("3:0")
    config.isAzRunning = False
    config.isElRunning = False
    config.azCurrentSpeed = 0.0

def relMoveAz(distance):
    cmd = '1:' + str(distance)
    logging.debug("relMoveAz(%s)", distance)
    sendStepperCommand(cmd)
    time.sleep(0.5)
    config.isAzRunning = True
    config.azCurrentSpeed = config.azMaxSpeed

def relMoveEl(distance):
    cmd = '2:' + str(distance)
    logging.debug("relMoveEl(%s)", distance)
    sendStepperCommand(cmd)
    time.sleep(0.5)
    config.isElRunning = True
    config.elCurrentSpeed = config.elMaxSpeed

# Triggers a closed-loop relative move by encoder counts
# Called by GUI pushbutton and text entry
# This is controlled in the periodic thread?
# TODO - program crashes if I accidentally enter a floating-point
# value for encoder counts
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
# TODO - Sometimes starting the encoder move does not work, even
# though the running flag is true.  It can be forced to start by
# making an open-loop move.  When the OL move is done, the 
# closed-loop move will execute.
#
# TODO - sometimes the axis moves past the encoder target and
# keeps going. I've seen this twice with az.  Subsequent
# closed-loop moves will fail after that. I was able to force
# it to work after making an open-loop degree move.
# 
# TODO - runSpeed() starts the motor with max acceleration
# should there be an initial starting ramp?  I would have
# to generate it myself
#
# TODO - I am calling runSpeed() every iteration of the loop, 
# which is poor programming.  I need to call only once for
# every speed change
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
                #setAzSpeed(azSpeed)
                runSpeed(0, azSpeed, 2)
            config.azInFarApproach = True
        elif abs(config.azDistanceTogo) > config.endpointNearDistance:
            azSpeed = config.azEndpointNearSpeed * polarity
            if config.azInNearApproach == False:
                logging.debug("watchEncoderMove(%s) set near approach run", axis)
                # Start slew at this speed
                #setAzSpeed(azSpeed)
                runSpeed(0, azSpeed, 2)                        
            config.azInNearApproach = True
        elif abs(config.azDistanceTogo) > config.endpointVeryNearDistance:
            azSpeed = config.azEndpointVeryNearSpeed * polarity
            if config.azInVeryNearApproach == False:
                logging.debug("watchEncoderMove(%s) set very near approach run", axis)
                #setAzSpeed(azSpeed)
                runSpeed(0, azSpeed, 2)                        
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
                #setElSpeed(elSpeed)
                runSpeed(1, elSpeed, 2)
            config.elInFarApproach = True
        elif abs(config.elDistanceTogo) > config.endpointNearDistance:
            elSpeed = config.elEndpointNearSpeed * polarity
            if config.elInNearApproach == False:
                logging.debug("watchEncoderMove(%s) set near approach run", axis)
                # Start slew at this speed
                #setElSpeed(elSpeed)
                runSpeed(1, elSpeed, 2)                        
            config.elInNearApproach = True
        elif abs(config.elDistanceTogo) > config.endpointVeryNearDistance:
            elSpeed = config.elEndpointVeryNearSpeed * polarity
            if config.elInVeryNearApproach == False:
                logging.debug("watchEncoderMove(%s) set very near approach run", axis)
                #setElSpeed(elSpeed)
                runSpeed(1, elSpeed, 2)                        
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
# This is part of the experimental ramping branch
# runSpeed will be changed to implement an 
# accel ramp based on passed arguments
def runSpeed(axis, speed, acceltime):
    #cmd = '20:' + str(axis)
    logging.debug("runSpeed() axis: %s", axis)
    #sendStepperCommand(cmd)
    #time.sleep(0.5)
    if axis == 0:
        config.isAzRunning = True
        config.azStartupComplete = False
        #config.isAzRamping = True
        # Startup the ramp generator
        azAccelRamp.setCurrentSpeed(config.azCurrentSpeed)
        azAccelRamp.setFinalSpeed(speed)
        azAccelRamp.setRampTime(acceltime)
        azAccelRamp.enable()
    elif axis == 1:
        config.isElRunning = True
        config.elStartupComplete = False
        #config.isElRamping = True
       # Startup the ramp generator
        elAccelRamp.setCurrentSpeed(config.azCurrentSpeed)
        elAccelRamp.setFinalSpeed(speed)
        elAccelRamp.setRampTime(acceltime)
        elAccelRamp.enable()

def stopAz():
    logging.debug("stopAz()")
    sendStepperCommand("4:0")
    config.isAzRunning = False
    config.azStartupComplete = False
    config.isTracking = False
    #config.azSpeed = 0.0
    azAccelRamp.disable()

def stopEl():
    logging.debug("stopEl()")
    sendStepperCommand("5:0")
    config.isElRunning = False
    config.elStartupComplete = False
    config.isTracking = False
    #config.elSpeed = 0.0
    elAccelRamp.disable()
    
def quickStopAz():
    logging.debug("quickStopAz()")
    sendStepperCommand("6:0")
    config.isAzRunning = False
    config.azHoming = False
    config.azStartupComplete = False
    config.isTracking = False
    #config.azSpeed = 0.0
    azAccelRamp.disable()

def quickStopEl():
    logging.debug("quickStopEl()")
    sendStepperCommand("7:0")
    config.isElRunning = False
    config.elHoming = False
    config.elStartupComplete = False
    config.isTracking = False
    #config.elSpeed = 0.0
    elAccelRamp.disable()

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

# Moves west in Azimuth until CCW limit switch made
# This results in setting the azHoming flag which is watched in
# the periodic thread by calling watchHomingAxis()

# TODO - it was observed after first starting program that
# system went into homing but motors didn't move.  Debug log
# showed that watchHoming was active. azRunning flag observed 
# to stick on in this case

# TODO - observed that after home limit was made and motor stopped,
# several very large values of velocity would display in the 
# position window before zeroing out
def homeAzimuth():
    config.azHoming = True
    logging.debug("homeAzimuth() executed")
    setAzMaxSpeed(config.azHomingSpeed)
    # slew west a long friggin way
    #relMoveAz(-40000)
    #setAzSpeed(-config.azHomingSpeed)
    runSpeed(0, -config.azHomingSpeed, 2)

# Moves south in elvation until down limit switch made
# This results in setting the elHoming flag which is watched in
# the periodic thread by calling watchHomingAxis()

# TODO - it was observed after first starting program that
# system went into homing but motors didn't move.  Debug log
# showed that watchHoming was activez. elRunning flag observed 
# to stick on in this case

# TODO - observed that after home limit was made and motor stopped,
# several very large values of velocity would display in the 
# position window before zeroing out
def homeElevation():
    config.elHoming = True
    logging.debug("homeElevation() executed")
    setElMaxSpeed(config.elHomingSpeed)
    # slew south a long friggin way
    #relMoveEl(-40000)
    #setElSpeed(-config.elHomingSpeed)
    runSpeed(1, -config.elHomingSpeed, 2)

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
# Let's ask the Arduino stepper controller
# Note - this doesn't always return correct value for elevation axis
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

# Star sidereal rate tracking
# TODO - stepper steps not updated while tracking
def startTracking():
    config.isTracking = True
    # TODO - latitude is hardcoded, fix it
    azSpeed = getTrackVelocity(0, config.azGeoPosn, config.elGeoPosn, 37.79)
    elSpeed = getTrackVelocity(1, config.azGeoPosn, config.elGeoPosn, 37.79)

    logging.debug("startTracking() azSpeed: %f", azSpeed)
    logging.debug("startTracking() elSpeed: %f", elSpeed)

    # Set the stepper speeds
    #setAzSpeed(azSpeed)
    #setElSpeed(elSpeed)
    
    # Start the motors
    runSpeed(0, azSpeed, 2)
    runSpeed(1, elSpeed, 2)
    

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
    azMovingAverage = MovingAverage(5)
    elMovingAverage = MovingAverage(5)

    # Instantiate accel ramps for both axes
    # TODO - delta T's are hard-coded, fix
    azAccelRamp = AccelRamp(0, 0.2)
    elAccelRamp = AccelRamp(1, 0.2)
    
    
    # PySimpleGUI
    # TODO - found a failure mode where I could not make absolute moves with the azimuth stepper, but could do open-loop 
    # moves, positive or negative.
    motion_layout =     [
                        [sg.Text('Relative Open Loop Move in Steps')],
                        [sg.Button('REL_AZ', pad=config.padSize, size=(10,1)),sg.InputText('',size=(10,1),key='relAz'),sg.Button('REL_EL', pad=config.padSize, size=(10,1)),sg.InputText('', size=(10,1),key='relEl')],
                        [sg.Text('Relative Open Loop Move in Stepper Degrees')],
                        [sg.Button('REL_AZ_DEG', pad=config.padSize),sg.InputText('',size=(10,1),key='relAzDeg'),sg.Button('REL_EL_DEG'),sg.InputText('', size=(10,1),key='relElDeg')],
                        [sg.Text('Absolute Closed Loop Move in Encoder Counts')],
                        [sg.Button('ABS_AZ_ENC', pad=config.padSize),sg.InputText('',size=(10,1),key='absAzEnc'),sg.Button('ABS_EL_ENC'),sg.InputText('', size=(10,1),key='absElEnc')],
                        [sg.Text('Absolute Closed Loop Move in Encoder Degrees')],
                        [sg.Button('DEG_AZ_ENC', pad=config.padSize),sg.InputText('',size=(10,1),key='degAzEnc'),sg.Button('DEG_EL_ENC'),sg.InputText('', size=(10,1),key='degElEnc')],
                        [sg.Text('Homing')],
                        [sg.Button('HOME_AZ', pad=config.padSize),sg.Button('HOME_EL', pad=config.padSize)],                        
                        [sg.Text('Stop Motion')],
                        [sg.Button('STOP_AZ', pad=config.padSize),sg.Button('STOP_EL', pad=config.padSize),sg.Button('FSTOP_AZ', pad=config.padSize),sg.Button('FSTOP_EL', pad=config.padSize)],
                        ]    

    position_layout =   [
                        [sg.Text('Az Encoder', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'azEncoder'),sg.Text('', size=(6,1), background_color = 'lightblue',key = 'azEncoderDeg')], 
                        [sg.Text('El Encoder', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'elEncoder'),sg.Text('', size=(6,1), background_color = 'lightblue',key = 'elEncoderDeg')],
                        [sg.Text('StepsAz', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'stepsAz')],
                        [sg.Text('StepsEl', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'stepsEl')],
                        [sg.Text('Az Vel', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'azVel')],
                        [sg.Text('El Vel', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'elVel')],
                        [sg.Text('Az Geo', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'azGeoPosn')],
                        [sg.Text('El Geo', size=(10,1)), sg.Text('', size=(7,1), background_color = 'lightblue',key = 'elGeoPosn')],
                        [sg.Button('ZERO_AZ_ENC', size=(13,1), pad=config.padSize),sg.Button('ZERO_EL_ENC',size=(13,1), pad=config.padSize)],
                        [sg.Button('ZERO_AZ_STEP',size=(13,1), pad=config.padSize),sg.Button('ZERO_EL_STEP',size=(13,1), pad=config.padSize)],
                        ]    


    state_layout =      [
                        [sg.Checkbox('AZ CCW Limit', key = 'azCCWLimit', size = (15,1)), sg.Checkbox('AZ CW Limit', key = 'azCWLimit', size = (15,1))],
                        [sg.Checkbox('EL UP Limit', key = 'elUPLimit', size = (15,1)), sg.Checkbox('EL Down Limit', key = 'elDownLimit', size = (15,1))],
                        [sg.Checkbox('AZ Running', key = 'azRunning', size = (15,1)), sg.Checkbox('EL Running', key = 'elRunning', size = (15,1)), sg.Checkbox('Tracking', key = 'tracking', size = (15,1))],
                        [sg.Button('AZ_SPD', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='azSpeed'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configAzSpeed')],
                        [sg.Button('AZ_MAX', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='azMax'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configAzMax')],
                        [sg.Button('AZ_ACCEL', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='azAccel'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configAzAccel')],
                        [sg.Button('EL_SPD', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='elSpeed'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configElSpeed')],
                        [sg.Button('EL_MAX', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='elMax'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configElMax')],
                        [sg.Button('EL_ACCEL', size=(10,1), pad=config.padSize),sg.InputText('',size=(5,1),key='elAccel'),sg.Text('', size=(6,1)), sg.Text('', size=(6,1), background_color = 'lightblue',key = 'configElAccel')],                        
                        ]

    celestial_layout =  [
                        [sg.Button('SET_AZ_GEO', pad=config.padSize),sg.InputText('',size=(10,1),key='azGeoInput'),sg.Button('SET_EL_GEO'),sg.InputText('', size=(10,1),key='elGeoInput')],
                        [sg.Button('TRACK', size=(10,1), pad=config.padSize), sg.Button('STOP_TRACK', size=(10,1))],
                        [sg.Text('LST', size=(10,1)), sg.Text('', size=(12,1), background_color = 'lightblue', key = 'localSiderealTime')],
                        [sg.Text('RA', size=(10,1)), sg.Text('', size=(12,1), background_color = 'lightblue',key = 'configRA')],
                        [sg.Text('Dec', size=(10,1)), sg.Text('', size=(12,1), background_color = 'lightblue',key = 'configDec')],
                        [sg.Text('TrackVel:', size=(10,1)), sg.Text('', size=(9,1), background_color = 'lightblue',key = 'azTrackVel'),sg.Text('', size=(9,1), background_color = 'lightblue',key = 'elTrackVel')],
                        ]

    other_layout =      [
                        [sg.Button('EXIT', size=(13,1))],
                        ]


    # I've got tab groups now!
    layout =            [[sg.TabGroup([[sg.Tab('Position', position_layout,font='Courier 24'), sg.Tab('Motion', motion_layout,font='Courier 24'), sg.Tab('State', state_layout,font='Courier 24'), sg.Tab('Celestial', celestial_layout,font='Courier 24'), sg.Tab('Other', other_layout,font='Courier 24'),]])]]    
                         
    
    
    # Open the GUI
    #sg.change_look_and_feel('GreenMono')
    sg.change_look_and_feel('DarkAmber')
    # This starts the window maximized, but I had to add an exit button 
    # cause the top bar dissapears
    window = sg.Window('Radio Telescope Control', layout, font='Courier 30', resizable = True).Finalize()
    window.maximize()

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

        if event == 'ZERO_AZ_ENC':
            zeroAzEncoder()

        if event == 'ZERO_EL_ENC':
            zeroElEncoder()        

        if event == 'ZERO_AZ_STEP':
            zeroSteppers(0)

        if event == 'ZERO_EL_STEP':
            zeroSteppers(1)        

        if event == 'REL_AZ':                   # TODO - breaks if a floating-point entered
            setAzMaxSpeed(config.azMaxSpeed)    # TODO - relative moves dont turn off isRunning state of motor
            relMoveAz(values['relAz'])

        if event == 'REL_EL':                   # TODO - breaks if a floating-point entered
            setElMaxSpeed(config.elMaxSpeed)    # TODO - relative moves dont turn off isRunning state of motor
            relMoveEl(values['relEl'])

        if event == 'REL_AZ_DEG':               # TODO - relative moves dont turn off isRunning state of motor
            setAzMaxSpeed(config.azMaxSpeed)
            moveAzStepperDegrees(values['relAzDeg'])

        if event == 'REL_EL_DEG':               # TODO - relative moves dont turn off isRunning state of motor
            setElMaxSpeed(config.elMaxSpeed)
            moveElStepperDegrees(values['relElDeg'])

        if event == 'ABS_AZ_ENC':               # TODO - breaks if a floating-point entered
            setAzMaxSpeed(config.azMaxSpeed)
            startEncoderMove(0, values['absAzEnc'])

        if event == 'ABS_EL_ENC':               # TODO - breaks if a floating-point entered
            setElMaxSpeed(config.elMaxSpeed)   
            startEncoderMove(1, values['absElEnc'])

        if event == 'DEG_AZ_ENC':
            setAzMaxSpeed(config.azMaxSpeed)
            startEncoderMove(0, encoderDegreesToCounts(0,values['degAzEnc']))

        if event == 'DEG_EL_ENC':
            setElMaxSpeed(config.elMaxSpeed)
            startEncoderMove(1, encoderDegreesToCounts(1,values['degElEnc']))        

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

        if event == 'EXIT':
            shutdown()

        # Speeds and accelerations
        # TODO - handle case where button is pressed but there 
        # is invalid or no speed in text entry
        if event == 'AZ_SPD':
            setAzSpeed(values['azSpeed'])

        if event == 'AZ_MAX':
            setAzMaxSpeed(values['azMax'])
            config.azMaxSpeed = values['azMax']

        if event == 'AZ_ACCEL':
            setAzAccel(values['azAccel'])            

        if event == 'EL_SPD':
            setElSpeed(values['elSpeed'])

        if event == 'EL_MAX':
            setElMaxSpeed(values['elMax'])
            config.elMaxSpeed = values['elMax']

        if event == 'EL_ACCEL':
            setElAccel(values['elAccel'])        

        if event == 'SET_AZ_GEO':
            setGeoOffset(0, values['azGeoInput'])

        if event == 'SET_EL_GEO':
            setGeoOffset(1, values['elGeoInput'])
            
        if event == 'TRACK':
            startTracking()
            
        if event == 'STOP_TRACK':
            stopAz()
            stopEl()
           
            
        # UPDATES
        # Updates the information in the text boxes
        # These values can be updated only on change
        # Note the formatting - example '{:0.3f}'.format() which rounds to 3 decimal places,
        # no padding to the left of the decimal point
        window.Element('azEncoder').Update(config.azMountPosn)
        window.Element('elEncoder').Update(config.elMountPosn)
        window.Element('azEncoderDeg').Update('{:0.3f}'.format(getEncodersDegrees(0)))
        window.Element('elEncoderDeg').Update('{:0.3f}'.format(getEncodersDegrees(1)))    
        window.Element('stepsAz').Update(config.azStepperPosn)
        window.Element('stepsEl').Update(config.elStepperPosn)
        window.Element('azCCWLimit').Update(config.azCCWLimit)
        window.Element('azCWLimit').Update(config.azCWLimit)
        window.Element('elUPLimit').Update(config.elUpLimit)
        window.Element('elDownLimit').Update(config.elDownLimit)
        window.Element('azRunning').Update(config.isAzRunning)
        window.Element('elRunning').Update(config.isElRunning)
        window.Element('tracking').Update(config.isTracking)
        window.Element('azVel').Update('{:0.3f}'.format(config.azAvgVelocity))
        window.Element('elVel').Update('{:0.3f}'.format(config.elAvgVelocity))
        window.Element('azGeoPosn').Update('{:0.3f}'.format(getGeoPosition(0)))
        window.Element('elGeoPosn').Update('{:0.3f}'.format(getGeoPosition(1)))
        window.Element('localSiderealTime').Update(str(getCurrentLST()))
        window.Element('configRA').Update(str(config.rightAscension))
        window.Element('configDec').Update(str(config.declination))
        
        # TODO - latitude is hard-coded, fix it    
        window.Element('azTrackVel').Update('{:0.4f}'.format(getTrackVelocity(0, config.azGeoPosn, config.elGeoPosn, 37.79)))   # pulses/second
        window.Element('elTrackVel').Update('{:0.4f}'.format(getTrackVelocity(1, config.azGeoPosn, config.elGeoPosn, 37.79)))   # pulses/second
        
        # Default motion parms
        window.Element('configAzSpeed').Update(config.azSpeed)
        window.Element('configAzMax').Update(config.azMaxSpeed)
        window.Element('configAzAccel').Update(config.azAccel)
        window.Element('configElSpeed').Update(config.elSpeed)
        window.Element('configElMax').Update(config.elMaxSpeed)
        window.Element('configElAccel').Update(config.elAccel)
        