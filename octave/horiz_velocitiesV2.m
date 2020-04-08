# Altaz velocities calculator
# Dave Cohen
# Same math that is driving the Python telescope 
# pointing application

# Entry values for azimuth, elevation, and longitude
# hard-coded for now
# These values are in degrees, of course
az = 40.0;
el = 30.0;
lat = 37.79;

# Convert angles to radians
azR = az * pi/180;
elR = el * pi/180;
latR = lat * pi/180;

# Trigonometric functions of the above angles
SA = sin(azR);
CA = cos(azR);
SE = sin(elR);
CE = cos(elR);
SP = sin(latR);
CP = cos(latR);

# Solve for hour angle and declination
# in terms of vector components x, y, z
X1 = -CA * CE * SP + SE * CP;
Y1 = -SA * CE;
Z1 = CA * CE * CP + SE * SP;

# To HA, Dec
R1 = sqrt(X1*X1+Y1*Y1);

# HA
if (R1 == 0)
    HA = 0.0
else
    HA = atan2(Y1,X1);
endif

ha = (HA * 180/pi)/15;

# DEC
DEC = atan2(Z1,R1)

# Trigonometric functions of HA, DEC
SH = sin(HA)       # already in radians
CH = cos(HA)
SD = sin(DEC)
CD = cos(DEC)
CHCD = CH * CD
SDCP = SD * CP

X2 = -CHCD * SP + SDCP
Y2 = -SH * CD
Z2 = CHCD * CP + SD * SP
RSQ = X2*X2+Y2*Y2
R2 = sqrt(RSQ)

# Parallactic angle
C = CD * SP - CH * SDCP
S = SH * CP

if (C*C+S*S) > 0
    Q = atan2(S,C)
else
    Q = pi - HA
endif

# Parallactic angle, in degrees
PA = 180/pi * Q

# Velocities (and accelerations, if I want them)
TINY =  1e-30   # a very small number

if (RSQ < TINY)
    RSQ = TINY
    R2 = math.sqrt(RSQ)
endif

QD = -X*CP/RSQ
AD = SP + Z * QD
ED = CP*Y/R2
EDR = ED/R2

AZD = AD    # azimuth velocity
ELD = ED    # elevation velocity

# convert to arcsec/sec
azV = (AZD * ((2*pi)/86400) * (360/(2*pi))) * 3600
elV = (ELD * ((2*pi)/86400) * (360/(2*pi))) * 3600



