#!/usr/bin/python3

import time
import ephem
import math
            

if __name__ == "__main__":
    # Define the observer
    
    # Don't include a date, ephem will assume "now"
    gumSpring = ephem.Observer()
    gumSpring.lon = '-77:55:27'
    gumSpring.lat = '37:47:26'
    gumSpring.elevation = 116           # elevation, in meters 
    
    # This defines a fixed body in the sky, like a Messier object or star
    # In this case, the beautiful planetary nebula in Lyra
    M57 = ephem.FixedBody()
    M57._ra = '18:53:36'                # mean J2000 coordinates         
    M57._dec = '33:02:00'
    
    # Get RA and Dec as normailzed radian angles
    RA = (ephem.hours(M57._ra)/ (2 * math.pi)) * 24.0
    Dec = M57._dec    
    
    # Sidereal tracking rate
    mu = 7.272e-5
     
    while True:
        gumSpring.date = ephem.now()
        # Positions agree withing  5 minutes of arc with Cartes DuCiel
        M57.compute(gumSpring)          # uses J2000 epoch by default
        
        # Initialize variables
        altitude, azimuth = M57.alt.norm, M57.az.norm   # normalized from 0 to 2*pi radians
        # zenith distance
        z_dist = math.pi/2 - altitude
        # local seidereal time
        LST = gumSpring.sidereal_time()
        
        # Calculate hour angle
        #hour_angle = LST - RA
        hour_angle_float = LST - M57._ra
        # Convert to angle type
        hour_angle = ephem.hours(hour_angle_float)
        
        # latitude
        latitude = gumSpring.lat
        
        # I used the equations from "A Mathematical Description of the Control System for the William Hershel Telescope"
        # R.A. Laing, Royal Greenwich Observatory
        # pp. 2-3
        #
        # The values were verified with Mel Bartel's "Scope To Sky" calculator, 
        # https://www.bbastrodesigns.com/scopeToSky.html
        
        # First derivative of zenith distace w.r.t. sideral time
        # 
        azVelocity = ((math.sin(latitude) * math.sin(z_dist) - math.cos(latitude) * math.cos(z_dist) * math.cos(azimuth)) / math.sin(z_dist)) * mu
        
        # Elevation velocity (in terms of zenith distance)
        # The math appears to be working, only the sign is backwards!
        z_velocity = (-math.sin(azimuth) * math.cos(latitude)) * mu
        elVelocity = -z_velocity
        
        while hour_angle < 0:
            hour_angle += 2 * math.pi
            
        while hour_angle > 2 * math.pi:
            hour_angle -= 2 * math.pi
        
        # debugging
        #print(type(hour_angle))
        #print('Latitude: %s' % gumSpring.lat)
        print('Latitude: %f' % (gumSpring.lat * 180.0/math.pi))
        print('Longitude: %f' % (gumSpring.lon * 180.0/math.pi))
        print("")
        print('RA: %f Dec: %f' % (RA, Dec))
        print('RA: %s Dec: %s' % (M57._ra, M57._dec))
        print("")
        print('LST: %f' % LST)
        print('LST: %s' % LST)
        print("")    
        print('HA: %f' % hour_angle)
        print('HA: %s' % ephem.hours(hour_angle))
        print("")
        
        # prints in dd:mm:ss.s
        #print('%s %s' % (M57.alt, M57.az))
        print('Alt: %s Az: %s' % (altitude, azimuth))
        # prints in radians
        #print('%f %f' % (M57.alt, M57.az))
        print('Alt: %f Az: %f' % (altitude, azimuth))
        #print(ephem.degrees(p.alt))
        print("")
        print('Az velocity: %f' % azVelocity)
        print('Az velocity: %s' % ephem.degrees(azVelocity))
        print("")
        print('El velocity: %f' % elVelocity)
        print('El velocity: %s' % ephem.degrees(elVelocity))
        print("")        
        
        # Time is here to keep everything from happening at once
        time.sleep(1.0)