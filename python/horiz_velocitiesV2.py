#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 09:42:27 2019

@author: dcohen

I checked this against Mel Bartels Scope To Sky Calculator
and it is in close agreement
"""
import sys
import ephem
import math 

# Convert a string value of angle to decimal degrees
# Accepts the following formats:
# 1. A decimal degree value, like 39.50
# 2. A space separate value, like dd mm ss.ss or dd mm.m
# 3. A colon separated value, like dd:mm:ss.s
# Returns
# Decimal value of angle in degrees
def dms2dec(str_angle):
    try:
        value = float(str_angle)
    except ValueError as error:
        if len(str_angle.split(' ')) == 3:
            vlist =  str_angle.split(' ')
            value = float(vlist[0]) + (float(vlist[1])/60.0) + (float(vlist[1])/3600.0)
        elif len(str_angle.split(':')) == 3:
            print("Colon separated value")
            vlist =  str_angle.split(':')
            value = float(vlist[0]) + (float(vlist[1])/60.0) + (float(vlist[1])/3600.0)
    finally:
        return value



# This is a translation from Starlink fortran library
# altaz.f and h2e.f

# inputs are hour angle, declination, and latitude

gumSpring = ephem.Observer()
gumSpring.lon = '-77:55:27'
gumSpring.lat = '37:47:26'
gumSpring.elevation = 116   # i'm calling this heightASL from now on
gumSpring.date = ephem.now()
print("Now: %s" % gumSpring.date)
print("LST: %s" % gumSpring.sidereal_time())
print("")

# input parameters
latitude = 40.0        # degrees
azimuth = 186.141      # degrees
elevation = 49.7795    # degrees

azString = input("Azimuth: ")
elString = input("Elevation: ")

azimuth = dms2dec(azString)
elevation = dms2dec(elString)

# convert to radians
azR = math.radians(azimuth)
elR = math.radians(elevation)
latR = math.radians(latitude)

#print("")
#print("azR: %f" % azR)
#print("elR: %f" % elR)
#print("latR: %f" % latR)

# trigonometric functions
SA = math.sin(azR)
CA = math.cos(azR)
SE = math.sin(elR)
CE = math.cos(elR)
SP = math.sin(latR)
CP = math.cos(latR)

#print("")
#print("SA: %f" % SA)
#print("CA: %f" % CA)
#print("SE: %f" % SE)
#print("CE: %f" % CE)
#print("SP: %f" % SP)
#print("CP: %f" % CP)

# First, solve for HA and DEC (h2e.f)

# HA, Dec as X, Y, Z
X = -CA * CE * SP + SE * CP
Y = -SA * CE
Z = CA * CE * CP + SE * SP

#print("")
#print("X: %f" % X)
#print("Y: %f" % Y)
#print("Z: %f" % Z)

# To HA, Dec
R = math.sqrt(X*X+Y*Y)



if R == 0:
    HA = 0.0
else:
    HA = math.atan2(Y,X)
    
#print("")
#print("HA: %f" % HA)
    
DEC = math.atan2(Z,R)
#print("DEC: %f" % DEC)


ha = math.degrees(HA)/15    # convert angle to time
haF = ephem.hours(HA)       # convert HA to time
dec = math.degrees(DEC)

#print("Declination: %f" % dec)
#print("Hour angle: %s" % haF)
#print("")

# Next, solve for velocities

# trigonometric functions
SH = math.sin(HA)       # already in radians
CH = math.cos(HA)
SD = math.sin(DEC)
CD = math.cos(DEC)
CHCD = CH * CD
SDCP = SD * CP
X = -CHCD * SP + SDCP
Y = -SH * CD
Z = CHCD * CP + SD * SP
RSQ = X*X+Y*Y
R = math.sqrt(RSQ)

#print("")
#print("SH: %f" % SH)
#print("CH: %f" % CH)
#print("SD: %f" % SD)
#print("CD: %f" % CD)
#print("CHCD: %f" % CHCD)
#print("SDCP: %f" % SDCP)
#print("X: %f" % X)
#print("Y: %f" % Y)
#print("Z: %f" % Z)
#print("RSQ: %f" % RSQ)
#print("R: %f" % R)

# Azimuth and elvation (just for checking)
if RSQ == 0.0:  
    A = 0.0
else:
    A = math.atan2(Y,X)
    
if A < 0:
    A  += 2*math.pi

E = math.atan2(Z,R)

print("Check azimuth: %f" %  math.degrees(A))
print("Check elevation: %f" % math.degrees(E))
print("")

# Parallactic angle
C = CD * SP - CH * SDCP
S = SH * CP

if (C*C+S*S) > 0:
    Q = math.atan2(S,C)
else:
    Q = math.pi - HA

PA = Q

print("Parallactic angle: %f" %  math.degrees(PA))

# Velocities (and accelerations, if I want them)
TINY =  1e-30   # a very small number

if RSQ < TINY:
    RSQ = TINY
    R = math.sqrt(RSQ)

#print("")
#print("R: %f" % R)

QD = -X*CP/RSQ
AD = SP + Z * QD
ED = CP*Y/R
EDR = ED/R

AZD = AD    # azimuth velocity
ELD = ED    # elevation velocity

# convert to degrees/sec
azV = AZD * ((2*math.pi)/86400) * (360/(2*math.pi))
elV = ELD * ((2*math.pi)/86400) * (360/(2*math.pi))

print("Azimuth velocity: %f degrees/sec" %  azV)
print("Elevation velocity: %f degrees/sec" % elV)
print("Azimuth velocity: %f arcsec/sec" %  (3600.0 * azV))
print("Elevation velocity: %f arcsec/sec" % (3600.0 * elV))

# Calculate the stepping rate
# There are 400 steps per motor revolution, and 10 motor revolutions
# per gearbox revolution, or 4000 steps per gearbox revolution
# The chain gear ratios are as follows
#
# For azimuth, the 18t/13t gear reduction reduces the PPR to 
# 4000 * (13/18) = 2889 PPR at the worm gear
# 
# For elevation, the 13t/18t gear reduction increases the PPR to 
# 4000 * (18/13) = 5538 PPR at the worm gear
#
# The worm ratio is simply the number of teeth in the worm wheel,
# for a single-start worm.  The worm wheel is 50-teeth, so that
# multiplies the PPR as follows:
#
# Azimuth: 2889 PPR at worm gear * 50 = 144444.4444 PPR
# Elevation: 5538 PPR at worm gear * 50 = 276923.076923 PPR
#
# Divide both sides by 360 to get the pulses per degree
#
# Azimuth = 144450/360 = 401.234567901 pulses per degree
# Elevation = 276900/360 = 769.230769231 pulses per degree
# 
# Multiply the velocities in degrees/second * pulses/degree = pulses/second
azVelPPS = 400 * 10 * (13.0/18.0) * 50 / 360 * azV
elVelPPS = 400 * 10 * (18.0/13.0) * 50 / 360 * elV

print("Azimuth velocity: %f pulses/sec" %  azVelPPS)
print("Elevation velocity: %f pulses/sec" % elVelPPS)
