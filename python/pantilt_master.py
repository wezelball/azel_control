#!/usr/bin/env python3

# pantilt_master.py
# Dave Cohen

import sys
import smbus
import time
import os
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
        self.delay = 0.5
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
                pass



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
    #setAzSpeed(250)
    setAzMaxSpeed(2000)
    time.sleep(0.25)
    setAzAccel(1500)
    time.sleep(0.25)
    #setElSpeed(250)
    setElMaxSpeed(2000)
    time.sleep(0.25)
    setElAccel(1500)

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
def getEncoders():
    encStringList = []
    cmd = '0'	# send one byte
    reply = sendMessage(1, cmd, encoders_i2c_address)
    encByteList = reply.split(':')
    for i in encByteList:
        try:
            encStringList.append(int(i.rstrip('\x00')))
        except ValueError:
            # This will happen if there is a bus error,
            # which does happen
            # This is not a real fix, now the position
            # is wrong - but what do I do?
            encStringList.append(0)

    return tuple(encStringList)

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


# Command functions
def slewNorth():
    print (sendStepperCommand("2:4000"))

def slewEast():
    print(sendStepperCommand("1:4000"))

def slewWest():
    print(sendStepperCommand("1:-4000"))

def slewSouth():
    print(sendStepperCommand("2:-4000"))

def stopAllSlew():
    print(sendStepperCommand("3:0"))

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
        print ("True")
        return True
    elif variable.azCWLimit.find('1') != -1:
        print ("False")
        return False


# returns 0 if limit made	
def isAzCCWLimit():
    variable.azCCWLimit = sendStepperCommand("9:0")
    if variable.azCCWLimit.find('0') != -1:
        print ("True")
        return True
    elif variable.azCCWLimit.find('1') != -1:
        print ("False")
        return False

# returns 0 if limit made
def isElUpLimit():
    variable.elUpLimit = sendStepperCommand("10:0")
    if variable.elUpLimit.find('0') != -1:
        print ("True")
        return True
    elif variable.elUpLimit.find('1') != -1:
        print ("False")
        return False

# returns 0 if limit made
def isElDownLimit():
    variable.elDownLimit = sendStepperCommand("11:0")
    if variable.elDownLimit.find('0') != -1:
        print ("True")
        return True
    elif variable.elDownLimit.find('1') != -1:
        print ("False")
        return False

# For now, let's home west
def homeAzimuth():
    variable.azHoming = True
    # slew west a long friggin way
    #sendStepperCommand("1:-20000"))
    pass

def homeElevation():
    variable.elHoming = True
    # slew west a long friggin way
    #sendStepperCommand("1:-20000"))
    pass

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
                '10':homeElevation
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

