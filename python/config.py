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

# State of limit switches
azCWLimit = False
azCCWLimit = False
elUpLimit = False
elDownLimit = False

# Axis positions
# The physical position of the mount in encoder counts, referenced to home limits
azMountPosn = 0
elMountPosn = 0

# Stepper positions
azStepperPosn = 0
elStepperPosn = 0

# The geographical position of the mount, referenced to true North and horizon
#azGeoPosn = 0
#elGeoPosn = 0