#have to run 'sudo apt-get install python-smbus'
#in Terminal to install smbus
import smbus
import time
import os

# display system info
#print os.uname()

bus = smbus.SMBus(1)

# I2C address of Arduino Slaves
steppers_i2c_address = 0x04
encoders_i2c_address = 0x05
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
		
def getEncoders():
    data = '0'	# send one byte
    bytesToSend = ConvertStringToBytes(data)
    bus.write_i2c_block_data(encoders_i2c_address, i2c_cmd, bytesToSend)
	
    replyBytes = bus.read_i2c_block_data(encoders_i2c_address, 0, 16)
    reply = ConvertBytesToString(replyBytes)
	
    return reply

def sendStepperCommand(cmd):
    bytesToSend = ConvertStringToBytes(cmd)
    bus.write_i2c_block_data(steppers_i2c_address, i2c_cmd, bytesToSend)
	
    #replyBytes = bus.read_i2c_block_data(steppers_i2c_address, 0, 16)
    #reply = ConvertBytesToString(replyBytes)
    
    #return reply


# loop to send message
exit = False
while not exit:
    r = raw_input('Enter something, "q" to quit: ')
    print(r)
    
    if r == '0':
		print(getEncoders())
		
    if r == '1':
		#print(sendStepperCommand("1:1000"))
		sendStepperCommand("1:1000")
		
    if r == '2':
		#print(sendStepperCommand("2:2000"))
		sendStepperCommand("2:2000")
    
    if r == '3':
		#print(sendStepperCommand("2:2000"))
		sendStepperCommand("1:-1000")

    if r == '4':
		#print(sendStepperCommand("2:2000"))
		sendStepperCommand("2:-2000")

    if r=='q':
        exit=True
