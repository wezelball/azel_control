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

# This thread is where communications will eventually 
# take place
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
            self.az,self.el = getEncoders()
            self.azVel = (self.az - self.lastAz)/self.delay
            self.elVel = (self.el - self.lastEl)/self.delay
            self.lastAz,self.lastEl = getEncoders()
            #Update the logfile
	    log.write(time.strftime('%H:%M:%S') + ',' +  str(self.az) + ',' + str(self.el) + \
			',' +  str(self.azVel) + ',' + str(self.elVel) + '\n')

   def print_time(self,threadName):
       print ("%s: %s") % (threadName, time.ctime(time.time()))
   
   def stop(self):
       self.running = False

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
    data = '0'	# send one byte
    bytesToSend = ConvertStringToBytes(data)
    bus.write_i2c_block_data(encoders_i2c_address, i2c_cmd, bytesToSend)
	
    replyBytes = bus.read_i2c_block_data(encoders_i2c_address, 0, 16)
    reply = ConvertBytesToString(replyBytes)
	
    encByteList = reply.split(':')
    for i in encByteList:
		encStringList.append(int(i.rstrip('\x00')))
    
    return tuple(encStringList)

# Send a stepper motion command to the Uno
def sendStepperCommand(cmd):
    bytesToSend = ConvertStringToBytes(cmd)
    bus.write_i2c_block_data(steppers_i2c_address, i2c_cmd, bytesToSend)
	
	# TODO - fix this - the Uno is not returning values
    replyBytes = bus.read_i2c_block_data(steppers_i2c_address, 0, 16)
    reply = ConvertBytesToString(replyBytes)
    return reply

def processCmd(cmd):
	switcher = {
		':Mn#':slewNorth,
		':Me#':slewEast,
		':Mw#':slewWest,
		':Ms#':slewSouth,
		':Q#':stopAllSlew,
		'0':printEncoders
	}
	func=switcher.get(cmd,lambda :'Invalid')
	return func()
	
def slewNorth():
	print (sendStepperCommand("2:6000"))
	
def slewEast():
	print(sendStepperCommand("1:6000"))
	
def slewWest():
	print(sendStepperCommand("1:-6000"))
	
def slewSouth():
	print(sendStepperCommand("2:-6000"))

def stopAllSlew():
	print(sendStepperCommand("3:0"))

def printEncoders():
	print(getEncoders())

# loop to send message
exit = False

# There needs to be a logfile
logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
log = open(logfile, 'w')
# Write the header
log.write("Time,azEncoder, elEncoder" + '\n')


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
log.close()
