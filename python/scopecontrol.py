#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 17 10:19:35 2019

@author: dcohen
"""
import sys
import time
import serial
import ephem
import threading


# Start a message queue
messageQ = []


# need serial port autodetection
# to monitor serial port, run the follwoing command
# sudo interceptty -s 'ispeed 9600 ospeed 9600' 
#   -l /dev/ttyACM0 /dev/ttyDUMMY | interceptty-nicedump
# 
# connect to /dev/ttyACM0 from Arduino IDE, and to
# /dev/ttyDUMMY from this program

#self.port = serial.Serial('/dev/ttyDUMMY', 57600, timeout = 5)
#self.port = serial.Serial('/dev/ttyACM0', 57600, timeout = 5)
port = serial.Serial('/dev/ttyDUMMY', 57600, timeout = 5)


def sendMessage(priority, message):
    messageQ.append((priority, message))
    messageQ.sort(reverse = True)
    
    while messageQ:
        # this is a tuple (priority, message)
        next_message = messageQ.pop()
        port.write(next_message[1])
        time.sleep(0.5)
        response = ghettoReadline()
        #print("Response: %s" % response)
        return response
    
# This works beautifully
def ghettoReadline():   # a "private" member function
    eol = b'\r\n'   # [CR][LF] is what arduino sends
    leneol = len(eol)
    line = bytearray()
    while True:
        c = port.read(1)
        # Print in hex form
        #print("c: %s" % hex(ord(c)))
        if c:
            line += c
            if line[-leneol:] == eol:   # slice of last 2 items in array
                break
        else:
            break
        
    return bytes(line)


# This thread is where communications will eventually 
# take place
class periodicThread (threading.Thread):
   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.running = True
      self.delay = 5
      
   def run(self):
       #print "Starting " + self.name
       while(self.running):
           time.sleep(self.delay)
           #self.print_time(self.name)
           sendMessage(2, "png\n")


   def print_time(self,threadName):
       print ("%s: %s") % (threadName, time.ctime(time.time()))
   
   def stop(self):
       self.running = False

class RadioObject():
    # Need to incorporate flux in Janskys in the catalog entry
    def __init__(self):
        """Returns a radio object based on a string match of the search string.
        
        The radio objects are in XEphem format, or at least a simplified
        subset of it.
        
        The field used here are defined as follows:
        "Name|Alt Name,f|J,rHH rMM rSS,dD, dM, dS,epoch"
        
        Note that subfields are separated by the | symbol
        
        Field 1 =       Name|Alt Name
        Field 2 =       "f" = fixed object
        Subfield 2A =   "J" = radio object
        Field 3     =   RA
        Field 4     =   Dec
        Field 5     =   Magnitude
        Field 6     =   Epoch (default 2000)
        """
        self.objects = [
            "Cygnus A|3C 405,f|J,19 57 44.0,+40 35 46.0,1.0,2000,0",
            "Cassiopeia A|3C 461,f|J,23 21 7.0,+58 32 47.0,1.0,2000,0",
            "Taurus A|3C 144,f|J,5 31 30.0,+21 59 0.0,1.0,2000,0"
            ]
        
    def getObject(self, searchstr):
        for item in self.objects:
            if searchstr in item:
                return item

        
class Stepper():
    def __init__(self, axis):

        # Which axis is this
        # 0 = azimuth, 1 = elevation
        if "az" in axis:
            self.axis = "az"
        elif "el" in axis:
            self.axis = "el"
        else:
            print("you must specify an axis")
            self.shutdown()
            sys.exit()
            
        # The actual position of the stepper
        self.stepperPosition = 0
        
        # Axis hasn't been homed yet
        axisHomed = False
        
    # Relative move
    def moveRelative(self, steps):
        command = "mvr " + str(steps) + '\n'
        # returns 1
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("mvr result: %s" % result)
        
    def setMaxSpeed(self, speed):
        command = "sms " + str(speed) + '\n'
        # returns 2
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("sms result: %s" % result)
        time.sleep(0.5)
        
    # Sets speed for constant speed running
    def setSpeed(self, speed):
        command = "spd " + str(speed) + '\n'
        # returns 3
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("spd result: %s" % result)
        time.sleep(0.5)
        
    # Runs at speed defined by setSpeed method
    def runAtSpeed(self):
        command = "rsp\n" # this command times out
        # returns 4
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("rsp result: %s" % result)

    # Stops running at constant speed
    def stop(self):
        command = "stp\n" 
        # returns 5
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("stp result: %s" % result)
        
    def setAccel(self, accel):
        command = "acl " + str(accel) + '\n'
        # returns 6
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("acl result: %s" % result)
    
    # This will be made "private" later
    def __getCurrentStepperPosition(self):
        command = "gcp\n"
        print("Sendmessage: %s" % command)
        result = long(sendMessage(1, command))
        print("gcp result: %s" % result)
        return result
    
    # This will be made "private" later
    def __setCurrentStepperPosition(self, position):
        command = "scp " + str(position) + '\n'
        # returns 7
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("scp result: %s" % result)
    
    # Absolute move
    def moveAbsolute(self, absolutePosition):
        command = "mva " + str(absolutePosition) + '\n'
        # returns 8
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("mva result: %s" % result)
        
    # Absolute move to a horizontal coordinate
    def moveAbsoluteDegrees(self, degrees):
        pass

    # Set the horizontal coordinate, in degrees (altazimuth)
    # works for azimuth or elevation, except that ranges will differ
    def setCoordinate(self, d, m=0, s=0): # degrees, minutes, seconds
        # I will need conditional range checking here
        dd = d + float(m)/60 + float(s)/3600
        if (dd > 0)  and  (dd <= 360):
            self.coordinate = dd
            return True
        else:
            #print ("Improper coordinate")
            return False
        
    # Get the horizontal coordinate, in degrees
    #def getCoordinate(self):
    #    return self.coordinate
    
    # Given stepper pulses, return degrees
    # Assume the zeroth pulse is a 0 degrees (no offset)
    # Negative values are allowed
    def pulsesToDegrees(self, pulses):
        # for azimuth axis
        # remember, reduce speed = multiply resolution
        # 400 pulses per rev (2x microstepping)
        # multiplied by 10:1 gearbox on output of stepper
        # multiplied by 13/18 for chain reduction
        # multiplied by 40 for # teeth in worm wheel (guess)
        if self.axis == 'az':
            degrees = pulses/321.0
        elif self.axis == "el":
            degrees = pulses/615.3
            
        return degrees
        
    # Given position in degrees, return degrees
    # Assume the zeroth degree is at pulse 0 (no offset)
    # Negative values are allowed
    def degreesToPulses(self, degrees):
        if self.axis == 'az':
            degrees = degrees * 321.0
        elif self.axis == "el":
            degrees = degrees * 615.3
    
    def getImuPosition(self):
        if self.axis == 'az':
            command = "gia\n"
            
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("gia result: %s" % result)
        return float(result)

if __name__ == "__main__":    
     
    # Create threads
    # Main comm thread
    commThread = periodicThread(1, "Thread-1")
    #thread2 = myThread(2, "Thread-2", 2)
    #thread2.start()
    
    # Instantiate the radio object class
    # This is my own custom radio object catalog class
    rObject = RadioObject()
    
    
    # All this defines an observer
    # Don't include a date, ephem will assume "now"
    gumSpring = ephem.Observer()
    gumSpring.lon = '-77:57'
    gumSpring.lat = '37:47'
    gumSpring.elevation = 116
    
    # Define a particular radio object. There should be a better
    # way to do this, like define all objects at once
    # Ephem needs instances though
    cygA = ephem.readdb(rObject.getObject("3C 405"))
    cygA.compute(gumSpring)
    
    cassA = ephem.readdb(rObject.getObject("3C 461"))
    cassA.compute(gumSpring)
    
    taurA =  ephem.readdb(rObject.getObject("Taurus"))
    taurA.compute(gumSpring)
    
    print('Cygnus A az/alt coords: %s %s' % (cygA.az, cygA.alt))
    print ('Cygnus A flux: %s' % cygA.mag)
    print('Cassiopeia A az/alt coords: %s %s' % (cassA.az, cassA.alt))
    print ('Cassiopeia A flux: %s' % cassA.mag)
    print('Taurus A az/alt coords: %s %s' % (taurA.az, taurA.alt))
    
 
    # Motion stuff here
    
    # Start the comms thread
    commThread.start()

    # Instantiate a stepper class
    stepperAz = Stepper(axis = "azimuth")
    # Set the azimuth coordinate

    # what is the IMU saying?
    #print("IMU azimuth: %d" % stepperAz.getImuPosition())
    
    # Setup
    stepperAz.setMaxSpeed(1000)
    stepperAz.setAccel(2000)
    stepperAz.setSpeed(1000)
    
    # Moves should be given as number of degrees relative, or degrees 
    # absolute. Then convert to steps and perform move.
    # When move complete, get current position in steps and convert
    # to degrees, storing both as current position values
    
    # Do some moves
    stepperAz.moveAbsolute(16000);
    
    #for i in range(3):
    #    j = stepperAz.moveRelative(4000)
        # the code works without this 
    #    time.sleep(1.0)
        # this return value from thee moveRelative command is 
        # getting captured by the getCurentPosition command
        # I need somw way of fixing that
    #    print("Position: %d" % stepperAz.getCurrentPosition())
    
    #time.sleep(5)
    #stepperAz.setCurrentPosition(0)
    #stepperAz.runAtSpeed()
    #time.sleep(10)
    #stepperAz.stop()
    #time.sleep(1)
    #print("Position: %d" % stepperAz.getCurrentPosition())
    #stepperAz.shutdown()
    
    for i in range(20):
        print("IMU azimuth: %d" % stepperAz.getImuPosition())
        time.sleep(1)
    
    # Must kill the comm thread before exiting the program
    print("Exiting, stopping comm thread")
    commThread.stop()
