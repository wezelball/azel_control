#have to run 'sudo apt-get install python-smbus'
#in Terminal to install smbus
import smbus
import time
import os
import datetime
import ephem
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
		
      self.azVelocity = 0
      self.elVelocity = 0
      self.azPos = 0
      self.elPos = 0
				
      # homing flags
      self.homed = False
      self.homing = False

# ************************* END CLASSES ******************************

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


# Command functions
def slewNorth():
   print (sendStepperCommand("2:3000"))
	
def slewEast():
   print(sendStepperCommand("1:3000"))
	
def slewWest():
   print(sendStepperCommand("1:-3000"))
	
def slewSouth():
   print(sendStepperCommand("2:-3000"))

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
def getAzCWLimit():
   variable.azCWLimit = sendStepperCommand("8:0")
   print(variable.azCWLimit)

# returns 0 if limit made	
def getAzCCWLimit():
   variable.azCCWLimit = sendStepperCommand("9:0")
   print(variable.azCCWLimit)

# returns 0 if limit made
def getElUpLimit():
   variable.azUpLimit = sendStepperCommand("10:0")
   print(variable.azUpLimit)
	
# returns 0 if limit made
def getElDownLimit():
   variable.azDownLimit = sendStepperCommand("11:0")
   print(variable.azDownLimit)

# Process menu command
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
		'5':getAzCWLimit,
		'6':getAzCCWLimit,
		'7':getElUpLimit,
		'8':getElDownLimit
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

while not exit:
    r = raw_input('Enter something, "q" to quit: ')
    
    processCmd(r)
    
    if r=='q':
        exit=True

# Exit program here
commThread.stop()
#log.close()
