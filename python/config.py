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

# Moving closed loop by encoder counts
# Need some additional flags to control
azMovingClosedLoop = False
elMovingClosedLoop = False

azDistance = 0
elDistance = 0

endpointFarDistance = 100
endpointNearDistance = 50
endpointVeryNearDistance = 10
endpointDeadband = 2
azDistanceTogo = 0
elDistanceTogo = 0

azEndpointFarSpeed = 1000     # replace with axis max speed when available
elEndpointFarSpeed = 1000     # replace with axis max speed when available
azEndpointNearSpeed = 250
elEndpointNearSpeed = 250
azEndpointVeryNearSpeed = 100
elEndpointVeryNearSpeed = 100

# We need to know what region the axis is in
azInFarApproach = False
elInFarApproach = False
azInNearApproach = False
elInNearApproach = False
azInVeryNearApproach = False
elInVeryNearApproach = False

# Homing flags
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
azGeoPosn = 0.0
elGeoPosn = 0.0
# Offset values are added to the mount position values to get the geo values
azGeoOffset = 0.0
elGeoOffset = 0.0

# Average velocities
azAvgVelocity = 0
elAvgVelocity = 0

# Track the number of i2c errors per session
encoderi2CErrorCount = 0
stepperi2CErrorCount = 0

# I need to know if az or el are finished starting to make this jam detection work
# If ramping, ignore timeout in motion thread
azStartupComplete = False
elStartupComplete = False
