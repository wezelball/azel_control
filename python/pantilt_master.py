#have to run 'sudo apt-get install python-smbus'
#in Terminal to install smbus
import smbus
import time
import os

# display system info
#print os.uname()

bus = smbus.SMBus(1)

# I2C address of Arduino Slave
i2c_address = 0x05
i2c_cmd = 0x01

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
		
def getEncoder(data):
	bytesToSend = ConvertStringToBytes(data)
	bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)
	
	replyBytes = bus.read_i2c_block_data(i2c_address, 0, 16)
	reply = ConvertBytesToString(replyBytes)
	
	return reply


# send welcome message at start-up
#bytesToSend = ConvertStringToBytes("Hello Uno")
#bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)

# loop to send message
exit = False
while not exit:
    r = raw_input('Enter something, "q" to quit"')
    print(r)
    
    #bytesToSend = ConvertStringToBytes(r)
    #bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)
    
    print(getEncoder(r))
    
    #reply = bus.read_i2c_block_data(i2c_address, 0, 16)
    #print (ConvertBytesToString(reply))
    
    
    if r=='q':
        exit=True
