#!/usr/bin/python3

import sys
import time
import serial
import ephem
import threading
import datetime
import statistics

import PySimpleGUI as sg

# ********************************************* GLOBAL *********************************************
# Start a message queue
messageQ = []

# Create lists for az and el resolvers
azResolverArray = []
elResolverArray = []

# need serial port autodetection
# to monitor serial port, run the follwoing command
# sudo interceptty -s 'ispeed 57600 ospeed 57600' 
#   -l /dev/ttyACM0 /dev/ttyDUMMY | interceptty-nicedump
# 
# connect to /dev/ttyACM0 from Arduino IDE, and to
# /dev/ttyDUMMY from this program

port = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)
#port = serial.Serial('/dev/ttyDUMMY', 57600, timeout = 5)

# ********************************************* FUNCTIONS *********************************************

def sendMessage(priority, message):
    messageQ.append((priority, message))
    messageQ.sort(reverse = True)
    
    while messageQ:
        # this is a tuple (priority, message)
        # 1 is the highest priority
        # higher numbers are lower priority
        next_message = messageQ.pop()
        port.write(next_message[1].encode())
        time.sleep(0.1)
        response = ghettoReadline()

        return response
    

# This python3 encoding crap beat the shit outta me for awhile
def ghettoReadline():   # a "private" member function
    eol = b'\r\n'   # [CR][LF] is what arduino sends
    leneol = len(eol)
    line = bytearray()
    while True:
        c = port.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:   # slice of last 2 items in array
                #print ("Found eol...")
                break
        else:
            break

    lineStr = line.decode('UTF-8')    
    return(lineStr)


def shutdown():
    print ("Shutting down...")
    commThread.stop()
    log.close()
    window.Close()
    time.sleep(3)
    port.close()
    sys.exit()

# return nicely formatted date/time
def getCurrentTime():
    currentDT = datetime.datetime.now()
    return currentDT.strftime("%Y-%m-%d %H:%M:%S")

def getCurrentLST():
    gumSpring.date = ephem.now()
    return (gumSpring.sidereal_time())

def addResolverValue(r_list, r_size, r_value):
    r_list.append(r_value)

    if len(r_list) > r_size:
        r_list.pop(0)

    return r_list

def getMovingAverage(r_list):
    sum = 0
    
    for i in r_list:
        sum += i

    return (float) (sum / len(r_list))

def getMovingMedian(r_list):
    # A StatisticsError will be thrown if the lst is empty
    try:
        return (float) (statistics.median(r_list))
    except statistics.StatisticsError as error:
        return 0



# ********************************************* CLASSES *********************************************

# This thread is where communications will eventually 
# take place
class periodicThread (threading.Thread):
   def __init__(self, threadID, name):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.running = True
      self.delay = 5
      self.message = ""
      self.response = ""
      
   def run(self):
       while(self.running):
           time.sleep(self.delay)
           #self.message = 'png\n'
           self.message = "gap 0\n"
           self.response = sendMessage(5, self.message) # very low queue priority
           addResolverValue(azResolverArray, 20, int(self.response[:-2]))
           time.sleep(0.5)
           self.message = "gap 1\n"
           self.response = sendMessage(5, self.message) # very low queue priority
           addResolverValue(elResolverArray, 20, int(self.response[:-2]))



   def print_time(self,threadName):
       print ("%s: %s") % (threadName, time.ctime(time.time()))
   
   def stop(self):
       self.running = False


class RadioObject():
    # Need to incorporate flux in Janskys in the catalog entry
    def __init__(self):
        """Returns a radio object based on a string match of the search string.
        
        The radio objects are in XEphem format, or at least a simplified
        subset of it.
        
        The field used here are defined as follows:
        "Name|Alt Name,f|J,rHH rMM rSS,dD, dM, dS,epoch"
        
        Note that subfields are separated by the | symbol
        
        Field 1 =       Name|Alt Name
        Field 2 =       "f" = fixed object
        Subfield 2A =   "J" = radio object
        Field 3     =   RA
        Field 4     =   Dec
        Field 5     =   Magnitude
        Field 6     =   Epoch (default 2000)
        """
        self.objects = [
            "Cygnus A|3C 405,f|J,19 57 44.0,+40 35 46.0,1.0,2000,0",
            "Cassiopeia A|3C 461,f|J,23 21 7.0,+58 32 47.0,1.0,2000,0",
            "Taurus A|3C 144,f|J,5 31 30.0,+21 59 0.0,1.0,2000,0"
            ]
        
    def getObject(self, searchstr):
        for item in self.objects:
            if searchstr in item:
                return item


class Stepper():
    def __init__(self, axis):

        # Which axis is this
        # 0 = azimuth, 1 = elevation
        if "az" in axis:
            self.axis = 'a'
        elif "el" in axis:
            self.axis = 'e'
        else:
            print("you must specify an axis")
            self.shutdown()
            sys.exit()
            
        # The actual position of the stepper
        self.stepperPosition = self.getCurrentStepperPosition()   # not axis-specific
        
        # Axis hasn't been homed yet
        azAxisHomed = False
        elAxisHomed = False

    # Relative move
    def moveRelative(self, steps):
        if self.axis == 'a':
            command = "mvr " + "0 " + str(steps) + '\n'
        elif self.axis == 'e':
            command = "mvr " + "1 " + str(steps) + '\n'
        # returns 1
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("mvr result: %s" % result)
        
    def setMaxSpeed(self, speed):
        if self.axis == 'a':
            command = "sms " + "0 " + str(speed) + '\n'
        elif self.axis == 'e':
            command = "sms " + "1 " + str(speed) + '\n'
        # returns 2
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("sms result: %s" % result)
        time.sleep(0.5)
        
    # Sets speed for constant speed running
    def setSpeed(self, speed):
        if self.axis == 'a':
            command = "spd " + "0 " + str(speed) + '\n'
        elif self.axis == 'e':
            command = "spd " + "1 " + str(speed) + '\n'
        # returns 3
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("spd result: %s" % result)
        time.sleep(0.5)
        
    # Runs at speed defined by setSpeed method
    def runAtSpeed(self):
        if self.axis == 'a':
            command = "rsp 0\n"
        elif self.axis == 'e':
            command = "rsp 1\n"
        # returns 4
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("rsp result: %s" % result)

    # Stops running at constant speed
    def stop(self):
        if self.axis == 'a':
            command = "stp 0\n"
        elif self.axis == 'e':
            command = "stp 1\n"
        # returns 5
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("stp result: %s" % result)
        
    def setAccel(self, accel):
        if self.axis == 'a':
            command = "acl " + "0 " + str(accel) + '\n'
        elif self.axis == 'e':
            command = "acl " + "1 " + str(accel) + '\n'
        # returns 6
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("acl result: %s" % result)
    
    # This is "private"
    def getCurrentStepperPosition(self):
        if self.axis == 'a':
            command = "gcp 0\n"
        elif self.axis == 'e':
            command = "gcp 1\n"
        print("Sendmessage: %s" % command)
        result = (sendMessage(3, command))
        print("gcp result: %s" % result)
        return (result)         # FIXME
    
    # This is "private"
    def __setCurrentStepperPosition(self, position):
        if self.axis == 'a':
            command = "scp " + "0 " + str(position) + '\n'
        elif self.axis == 'e':
            command = "scp " + "1 " + str(position) + '\n'
        # returns 7
        print("Sendmessage: %s" % command)
        result = sendMessage(2, command)
        print("scp result: %s" % result)
    
    # Absolute move
    def moveAbsolute(self, absolutePosition):
        if self.axis == 'a':
            command = "mva " + "0 " + str(absolutePosition) + '\n'
        elif self.axis == 'e':
            command = "mva " + "1 " + str(absolutePosition) + '\n'
        # returns 8
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("mva result: %s" % result)
        
    # Absolute move to a horizontal coordinate
    def moveAbsoluteDegrees(self, degrees):             # need to define axes
        pass

    # Set the horizontal coordinate, in degrees (altazimuth)
    # works for azimuth or elevation, except that ranges will differ
    def setCoordinate(self, d, m=0, s=0): # degrees, minutes, seconds
        # I will need conditional range checking here
        dd = d + float(m)/60 + float(s)/3600
        if (dd > 0)  and  (dd <= 360):
            self.coordinate = dd
            return True
        else:
            #print ("Improper coordinate")
            return False
        
    # Get the horizontal coordinate, in degrees
    #def getCoordinate(self):
    #    return self.coordinate
    
    # Given stepper pulses, return degrees
    # Assume the zeroth pulse is a 0 degrees (no offset)
    # Negative values are allowed
    def pulsesToDegrees(self, pulses):
        # for azimuth axis
        # remember, reduce speed = multiply resolution
        # 400 pulses per rev (2x microstepping)
        # multiplied by 10:1 gearbox on output of stepper
        # multiplied by 13/18 for chain reduction
        # multiplied by 40 for # teeth in worm wheel (guess)
        if self.axis == 'a':
            degrees = pulses/321.0
        elif self.axis == 'e':
            degrees = pulses/615.3
            
        return degrees
        
    # Given position in degrees, return degrees
    # Assume the zeroth degree is at pulse 0 (no offset)
    # Negative values are allowed
    def degreesToPulses(self, degrees):
        if self.axis == 'a':
            degrees = degrees * 321.0
        elif self.axis == 'e':
            degrees = degrees * 615.3
    
    def getImuPosition(self):
        if self.axis == 'a':
            command = "gim 0\n"
        elif self.axis == 'e':      
            command = "gim 1\n"
            
        print("Sendmessage: %s" % command)
        result = sendMessage(2, command)
        print("gia result: %s" % result)
        return float(result)

    def getResolver(self):
        if self.axis == 'a':
            command = "gap 0\n"
        elif self.axis == 'e':      
            command = "gap 1\n"
        
        print("Sendmessage: %s" % command)
        result = sendMessage(2, command)
        print("gap result: %s" % result)
        # strip off the [CR][LF], it messes up my logfile
        return result[:-2]

    # Determine if limit switch is made
    # The class type automatically selects the proper axis
    # Limits are defined as follows:
    # axis       limit    meaning
    #   ----       -----    -------
    #   azimuth     0       max CW
    #   azimuth     1       max CCW
    #   elevation   0       max UP
    #   elevation   1       max DOWN
    # 
    # returns True if limit made
    def isLimitReached(self, limit):
        if self.axis == 'a':
            command = "lim 0 " + str(limit) + ' \n'
        elif self.axis == 'e':
            command = "lim 1 " + str(limit) + ' \n'
        
        print("Sendmessage: %s" % command)
        result = sendMessage(1, command)
        print("lim result: %s" % result)

        # This can fail of bad data comes from serial port
        try:
            if int(result[:-2]) == 1:
                return False
            elif int(result[:-2]) == 0:
                return True
        except ValueError as error:
            return False

# The application starts here
# *******************************************************************************************************

# Initialization for PySimpleGUI
location_layout =   [
                    [sg.Text('Site', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue', key = 'siteName')],
                    [sg.Text('Latitude', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'latitude')],
                    [sg.Text('Longitude', size=(10,1)), sg.Text('', size=(18,1),background_color = 'lightblue', key = 'longitude')],
                    [sg.Text('Elevation', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue', key = 'elevation')],
                    [sg.Text('Local time', size=(10,1)), sg.Text('', size=(18,1),background_color = 'lightblue', key = 'localTime')],
                    [sg.Text('LST', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue', key = 'localSiderealTime')]
                    ]   

position_layout =   [
                    [sg.Text('Azimuth', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'azimuth')],
                    [sg.Text('Altitude', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'altitude')],
                    [sg.Text('Az Resolver', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'azResolver')],
                    [sg.Text('El Resolver', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'elResolver')],
                    [sg.Text('StepsAz', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsAz')],
                    [sg.Text('StepsAlt', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'stepsAlt')],
                    [sg.Text('Az Limit', size=(10,1)), sg.Button('AzCW', button_color=('white', 'green'),enable_events=True, key='butAzCW', 
                        size = (10,1)), sg.Button('AzCCW', button_color=('green', 'red'),enable_events=True, key='butAzCCW', size = (10,1)),],
                    [sg.Text('El Limit', size=(10,1)), sg.Button('ElDown', button_color=('white', 'green'),enable_events=True, key='butElDown',
                         size = (10,1)), sg.Button('ElUp', button_color=('green', 'red'),enable_events=True, key='butElUp',size = (10,1)),],
                    #[sg.Text('Soft limit az', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'softLimitAz')],
                    #[sg.Text('Soft limit alt', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'softLimitAlt')],
                    #[sg.Text('Hard limit az', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'hardLimitAz')],
                    #[sg.Text('Hard limit alt', size=(10,1)), sg.Text('', size=(18,1), background_color = 'lightblue',key = 'hardLimitAlt')],
                    ]

operation_layout =  [
                    [sg.Text('Azimuth Move Relative')],
                    [sg.Button('AZ_CCW'),sg.Button('AZ_CW')],
                    [sg.Text('Elevation Move Relative')],
                    [sg.Button('EL_DOWN'),sg.Button('EL_UP')],
                    #[sg.Text('Homed', size=(10,1)), sg.Button('Az', button_color=('white', 'red'),enable_events=True, key='butAzHome'), sg.Button('El', button_color=('white', 'red'),enable_events=True, key='butElHome')],
                    ]

layout =            [
                    [sg.Frame('Location', location_layout), sg.Frame('Position', position_layout)],
                    [sg.Frame('Operation', operation_layout)],
                    ]

window = sg.Window('Radio Telescope Control', layout)


# ************************************************ STARTUP **********************************************

# There needs to be a logfile
logfile = "log_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".csv"
log = open(logfile, 'w')
# Write the header
log.write("Time,AzResolver,ElResolver,avgAzResolver,avgElResolver" + '\n')

# Define the observer
# Don't include a date, ephem will assume "now"
gumSpring = ephem.Observer()
gumSpring.lon = '-77:57'
gumSpring.lat = '37:47'
gumSpring.elevation = 116

# Instantiate a stepper instance
stepperAz = Stepper(axis = "az")
stepperEl = Stepper(axis = "el")

# Instantiate a radio object
rObject = RadioObject()

# Set some sane values for speed and acceleration
stepperAz.setMaxSpeed(500)
stepperEl.setMaxSpeed(500)
time.sleep(1)               # get rid of these later using response codes
stepperAz.setAccel(500)
stepperEl.setAccel(500)
time.sleep(1)
stepperAz.setSpeed(500)
stepperEl.setSpeed(500)
time.sleep(1)

# Start the comms thread after initialization
commThread = periodicThread(1, "Thread-1")
commThread.start()

# The loop starts here
# *******************************************************************************************************
while True:        # Event Loop

    event, values = window.Read(1000) # Please try and use as high of a timeout value as you can
    
    if event is None or event == 'Quit':    # if user closed the window using X or clicked Quit button
        shutdown()
    if event == 'AZ_CW':
        stepperAz.moveRelative(3000)
    if event == 'AZ_CCW':
        stepperAz.moveRelative(-3000)
    if event == 'EL_UP':
        stepperEl.moveRelative(3000)
    if event == 'EL_DOWN':
        stepperEl.moveRelative(-3000)

    # Updates the information in the window
    # These values can be updated only on change 
    window.Element('elevation').Update(gumSpring.elevation)     # make more generic - sitename variable
    window.Element('latitude').Update(str(gumSpring.lat))
    window.Element('longitude').Update(str(gumSpring.lon))
    
    # These values should be updated quickly
    window.Element('localTime').Update(getCurrentTime())
    window.Element('localSiderealTime').Update(str(getCurrentLST()))
    window.Element('azimuth').Update(str(stepperAz.getImuPosition()))
    window.Element('altitude').Update(str(stepperEl.getImuPosition()))
    window.Element('stepsAz').Update(str(stepperAz.getCurrentStepperPosition()))
    window.Element('stepsAlt').Update(str(stepperEl.getCurrentStepperPosition()))
    window.Element('azResolver').Update(getMovingMedian(azResolverArray))
    window.Element('elResolver').Update(getMovingMedian(elResolverArray))

    # Check for limit switch values
    # Azimuth
    if stepperAz.isLimitReached(0) == True:
        window.FindElement('butAzCW').Update(button_color=('white', 'red'))
    elif stepperAz.isLimitReached(0) == False:
        window.FindElement('butAzCW').Update(button_color=('white', 'green'))

    if stepperAz.isLimitReached(1) == True:
        window.FindElement('butAzCCW').Update(button_color=('white', 'red'))
    elif stepperAz.isLimitReached(1) == False:
        window.FindElement('butAzCCW').Update(button_color=('white', 'green'))

    # Elvation
    if stepperEl.isLimitReached(0) == True:
        window.FindElement('butElUp').Update(button_color=('white', 'red'))
    elif stepperEl.isLimitReached(0) == False:
        window.FindElement('butElUp').Update(button_color=('white', 'green'))

    if stepperEl.isLimitReached(1) == True:
        window.FindElement('butElDown').Update(button_color=('white', 'red'))
    elif stepperEl.isLimitReached(1) == False:
        window.FindElement('butElDown').Update(button_color=('white', 'green'))


    # Update the logfile
    log.write(time.strftime('%H:%M:%S') + ',' +  stepperAz.getResolver() + ',' + 
        stepperEl.getResolver() + ',' + str(getMovingMedian(azResolverArray)) + ',' + str(getMovingMedian(elResolverArray)) + '\n')