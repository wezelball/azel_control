# Global variables
# Yeah, that's right, global frickin' variables
# Sorry if you don't like it

# Constants
STEPPERS_I2C_ADDR = 0x04
ENCODERS_I2C_ADDR = 0x05
I2C_CMD = 0x01

# i2c comms I/O flags
encoderIOError = False
stepperIOError = False

# Machine state
isAzRunning = False;
isElRunning = False;

# homing flags
azHomed = False
azHoming = False
elHomed = False
elHoming = False