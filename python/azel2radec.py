#!/usr/bin/python3

import math

# azel2radec.py
# Dave Cohen
#
# Given horizon coordiantes (azimuth/altitude) of a point in the sky, return the
# equatorial coordinates (RA/Dec)

# The equations:
#
# sin(declination) = sin(altitude) * sin(latitude) + cos(altitude) * cos(latitude) * cos(Azimuth)
#
# cos(hourAngle) = sin(altitude) - sin(latitude) * sin(declination)
#                  ------------------------------------------------
#                         cos(latitude) * cos(declination)
#
# RA = LST - hourAngle
#
# Source is "Practical Astronomy With Your Calculator", Peter Duffet-Smitth, p. 38


# Given hours as a float, return a string hh:mm:ss
def dec2hms(timeval):
    hours = int(timeval)
    minutes = int((timeval - hours) * 60)
    seconds = int(((timeval - hours) * 60 - int((timeval - hours) * 60)) * 60)
    timeStr = str(hours) + ":" + str(minutes) + ":" + str(seconds)
    return timeStr

# converts horizontal (azimuth, altitude) to equatorial (RA, Dec) coordinates
# Inputs:
# az, alt, lat are in radians (float)
# lst is in hours (float)
# Returns:
# RA in hours, Dec in radians as tuple
def horiz2equ(az, alt, lat, lst):
    # Find sin(declination)
    sinDec = math.sin(alt) * math.sin(lat) + math.cos(alt) * math.cos(lat) * math.cos(az)
    dec = math.asin(sinDec)

    # Find cos(HA) (hour angle)
    cosHA = (math.sin(alt) - math.sin(lat) * math.sin(dec))/(math.cos(lat) * math.cos(dec))  
    # take inverse cos to find HA, but need to account for "wrap"
    HA1 = math.acos(cosHA)
    #print('HA1 = %f' % math.degrees(HA1))

    if math.sin(azimuth) < 0.0:
        HA2 = HA1
    else:
        HA2 = math.radians(360) - HA1

    # Convert hour angle into hours by converting to degrees, 
    # then dividing by 15
    HA = math.degrees(HA2)/15

    # Subtract HA from LST. If result is negative, add 24
    # This is RA in hours
    if (LST - HA) < 0:
        RA = LST - HA + 24
    else:
        RA = LST - HA

    # Return as a tuple
    return (RA, dec)



# Example:
# A star is seen by observer at latitude 52 deg. N to have an altitude of 19d 20' 04"
# and an azimuth of 283d 16' 16". What are its HA and declination? If the observer is
# on the Greenwich meridian and the GST is 0h 24m 05s, what is the RA?

# Convert azimuth and altitude to decimal degrees, then radians
print('Azimuth, degrees: %f' % (283 + 16/60 + 16/3600))
azimuth = math.radians(283 + 16/60 + 16/3600) 
print('Azimuth, radians: %f' % azimuth)
print('')
print('Altitude, degrees: %f' % (19 + 20/60 + 4/3600))
altitude = math.radians(19 + 20/60 + 4/3600)
print('Altitude, radians: %f' % altitude)
print('')

# Convert latitude to radians
latitude = math.radians(52.0)

# Find sin(declination)
sinDec = math.sin(altitude) * math.sin(latitude) + math.cos(altitude) * math.cos(latitude) * math.cos(azimuth)
print('sin(decliination) = %f' % sinDec)

# take inverse sin to find declination
declination = math.asin(sinDec)
print('declination = %f' % math.degrees(declination))

# Find cos(HA) (hour angle)
cosHA = (math.sin(altitude) - math.sin(latitude) * math.sin(declination))/(math.cos(latitude) * math.cos(declination))
print('cos(HA) = %f' % cosHA)

# take inverse cos to find HA, but need to account for "wrap"
HA1 = math.acos(cosHA)
print('HA1 = %f' % math.degrees(HA1))

if math.sin(azimuth) < 0.0:
    HA2 = HA1
else:
    HA2 = math.radians(360) - HA1

# Convert hour angle into hours by converting to degrees, 
# then dividing by 15
HA = math.degrees(HA2)/15

print('Hour angle = %f' % HA)

# Need to find the right ascension
# RA = LST - hourAngle

# Convert LST (0h 24m 5s) into decimal hours
LST = 24/60 + 5/3600
print('LST: %f' % LST)

# Subtract HA from LST. If result is negative, add 24
# This is RA in hours
if (LST - HA) < 0:
    RA = LST - HA + 24
else:
    RA = LST - HA

print('RA, hours =  %s' % dec2hms(RA))
