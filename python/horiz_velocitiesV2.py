#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 09:42:27 2019

@author: dcohen
"""

import ephem
import math 


#def getCurrentLST():
#    gumSpring.date = ephem.now()
#    print("Now: %s: " % gumSpring.date)
#    return (gumSpring.sidereal_time())


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
latitude = 37.79        # degrees
azimuth = 186.0         # degrees
elevation = 28.0        # degrees

# convert to radians
azR = math.radians(azimuth)
elR = math.radians(elevation)
latR = math.radians(latitude)

# trigonometric functions
SA = math.sin(azR)
CA = math.cos(azR)
SE = math.sin(elR)
CE = math.cos(elR)
SP = math.sin(latR)
CP = math.cos(latR)

# First, solve for HA and DEC (h2e.f)

# HA, Dec as X, Y, Z
X = -CA * CE * SP + SE * CP
Y = -SA * CE
Z = CA * CE * CP + SE * SP

# To HA, Dec
R = math.sqrt(X*X+Y*Y)

if R == 0:
    HA = 0.0
else:
    HA = math.atan2(Y,X)
    
DEC = math.atan2(Z,R)

ha = math.degrees(HA)/15    # convert angle to time
haF = ephem.hours(HA)       # convert HA to time
dec = math.degrees(DEC)

print("Declination: %f" % dec)
print("Hour angle: %s" % haF)
print("")

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
