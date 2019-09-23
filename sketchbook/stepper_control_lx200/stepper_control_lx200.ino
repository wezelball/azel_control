


// Include Arduino Wire library for I2C
#include <Wire.h>
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
#include <SoftwareSerial.h>
#include "AccelStepper.h" 
 
// Define Slave I2C Address
#define SLAVE_ADDR 9
 
// Define Slave answer size
#define ANSWERSIZE 8

SoftwareSerial mySerial(11, 12); // RX, TX - debug

// Instantiate RTC
RTC_DS3231 rtc;

// AccelStepper Instances
AccelStepper stepperX(1, 5, 6);   // X is the azimuth axis
                                  // 1 = Easy Driver interface
                                  // UNO Pin 5 connected to STEP pin of Easy Driver
                                  // UNO Pin 6 connected to DIR pin of Easy Driver

AccelStepper stepperY(1, 8, 9);   // Y is the elevation axis
                                  // 1 = Easy Driver interface
                                  // UNO Pin a connected to STEP pin of Easy Driver
                                  // UNO Pin b connected to DIR pin of Easy Driver

// ******************************BEGIN GLOBALS ***************************************

// Hard limits
int azLimitCW = 2;
int azLimitCCW = 4;
int elLimitUp = 3;
int elLimitDown = 10;

// Hard limit flags
bool azMovingCW = false;    // az rotating clockwise 
bool azMovingCCW = false;   // az rotating anti-clockwise
bool azCWLimit = false;     // at the CW limit 
bool azCCWLimit = false;    // at the CW limit 

// Stepper motion flags
bool elMovingUp = false;    // elevation rotating upwards
bool elMovingDown = false;  // elevation rotating downwards
bool elDownLimit = false;   // at the down limit
bool elUpLimit = false;     // at the up limit
bool movingX = false;       // azimuth axis is moving
bool movingY = false;       // elevation axis is moving
bool stoppingX = false;     // azimuth axis is deceling to stop
bool stoppingY = false;     // elevation axis is deceling to stop

bool slewingX = false;      // TODO: get rid of this if possible, too specific
bool slewingY = false;      // TODO: get rid of this if possible, too specific
bool stoppingXSlew = false; // TODO: get rid of this if possible, too specific 
bool stoppingYSlew = false; // TODO: get rid of this if possible, too specific 

bool stepperXRun = false; // flag is passed to the Timer1 ISR
bool stepperYRun = false; // flag is passed to the Timer1 ISR

// Set this from CdC
float UTCOffset = -4.0;     // your timezone relative to UTC
double longitude =-77.924;  // your longtitude.
double latitude =37.791;    // your latitude.

double M,Y,D,MN,H,S;        // current UTC time
double A,B;
double LST_degrees;         // variable to store local side real time(LST) in degrees.
double LST_hours;           // variable to store local side real time(LST) in decimal hours.
double azimuth;             // current azimuth - use same reference as CdC
double elevation;           // current elevation - use same reference as CdC
double initialAz = 180.0;   // initial azimuth - use same reference as CdC
double initialEl = 45.0;    // initial elevation - use same reference as CdC
double RA;					        // current right ascension;
double declination;			    // current declination

long azEncoderCount;        // the current azimuth encoder count
long elEncoderCount;        // the current elevation encoder count
// azimuth and encoder resolutions in degrees per pulse
// assume 10:1 gear ratio and 600 PPR encoder
double azEncRes = 0.06;   
double elEncRes = 0.06;

int RA_step_direction = 1;
int DEC_step_direction = 1;
String inputString = "";         		// a string to hold incoming data
boolean stringComplete = false;  		// whether the string is complete
boolean RA_tracking_Enabled = true; // Always track - Currently unused
boolean RA_steppingEnabled = true; 	// Non-default RA movement if we are doing something other than tracking
boolean DEC_steppingEnabled = false;// DEC movement is always optional

// This is just for debugging - send data to mySerial every 1000 msec
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds


// ****************************** END GLOBALS ***************************************


// encoder = 0    azimuth
// encoder = 1    elavation
long getEncoderPosition(int encoder) {
  int i;
   long az;
   long el;
   String az_str;
   String el_str;
  
  // Write a command to the Slave
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((byte)0);
  Wire.endTransmission();

  // Read response from Slave
  // Read back ANSWERSIZE characters
  // This will return a string with az and el values 
  // concantenated, and separated by a colon
  // "123456:-654321"
  // Has been tested with 6 digits each, one of the 
  // values was negative
  Wire.requestFrom(SLAVE_ADDR,ANSWERSIZE);
  
  // Add characters to string
  String response = "";
  while (Wire.available()) {
      char b = Wire.read();
      //Serial.println(b);
      response += b;
  }

  // Parse out the separate values
  i = response.indexOf(':');
  az_str = response.substring(0, i) + '\0';
  el_str = response.substring(i+1) + '\0';

  az = az_str.toInt();
  el = el_str.toInt();
  
  if(encoder == 0)
    return az;

  if(encoder == 1)
    return el;
}

void parseLX200(String thisCommand)
{
 //mySerial.println(thisCommand);
 //mySerial.println(inputString);
 switch (inputString.charAt(0)) { // If this isnt 58 (:), something is wrong!
     case ':':
      switch (inputString.charAt(1)) {
        // ***************** S **********************
        case 'S':// Set Stuff
          switch (inputString.charAt(2)) {
              case 'w': // Slew rate
                Serial.println(1);
                mySerial.println("set max slew");
              break;
          }// end :Sw
          break; //Case S Char2
        // ***************** S **********************
        //
        // ***************** R **********************
        case 'R':// Rate Control - R
          switch (inputString.charAt(2)) {
          case 'C':
            //RA_StepInterval = (initialRA_StepInterval *2);
            //DEC_StepInterval = (initialDEC_StepInterval *2);
            mySerial.println(" Set interval to half default");
          break;
          case 'G':
           //RA_StepInterval = initialRA_StepInterval;
           //DEC_StepInterval = initialDEC_StepInterval;
           mySerial.println(" Set interval to default");
          break;      
          case 'M':
           //RA_StepInterval = initialRA_StepInterval/2;
           //DEC_StepInterval = initialDEC_StepInterval/2;
           mySerial.println(" Set interval to DOUBLE SPEED");
          break;
          case 'S':
           //RA_StepInterval = minimumRA_StepInterval;
           //DEC_StepInterval = minimumDEC_StepInterval;
           mySerial.println(" Set interval to FASTEST");
          break;
      } // CaseR Char2
        break; // End Rate Control
        // ***************** R **********************
        //
        // ***************** M **********************
        case 'M':  // Movement Control - M
          switch (inputString.charAt(2)) {
          case 'w':
            relativeMove(0, 1000);
            //mySerial.println("Move RA forwards (west)");
          break;
          case 'e':
            relativeMove(0, -1000);
            //mySerial.println("Move RA backwards (east) ");
          break;
          case 'n':
            relativeMove(1, 1000);
            //mySerial.println("Move DEC forwards (north)");
          break;
          case 's':
            relativeMove(1, -1000);
            //mySerial.println("Move DEC backwards (south)");
          break;
          } // CaseM Char2
        break; // End movemont control
      // ***************** M **********************
      //
      // ***************** m **********************
      // Stepper test commands
      // remove after debugging
      case 'm':
        switch (inputString.charAt(2)) {
          case 'u': // slew elevation up 1000 steps
            relativeMove(1, 1000);
            //mySerial.println("Slew UP");
            break;
          case 'd': // slew elevation down 1000 steps
            relativeMove(1, -1000);
            //mySerial.println("Slew DOWN");
            break;
          case 'l': // slew azimuth CCW (left) 1000 steps
            relativeMove(0, -1000);
            //mySerial.println("Slew CCW");
            break;
          case 'r': // slew azimuth CW (right) 1000 steps
            relativeMove(0, 1000);
            //mySerial.println("Slew CW");
            break;
        } // Case m Char2
        break;  // Case m
      // ***************** Q **********************
      case 'Q': // Stop Moving - Q
        RA_steppingEnabled = 1; // We still move RA 
        //RA_StepInterval = initialRA_StepInterval; // We just set the speed so that stars should be "stationary" relative to everything else
        DEC_steppingEnabled = 0;
        mySerial.println ("Stepping halted");
        break;
      // ***************** Q **********************
      //
      // ***************** X **********************
      case 'X': // Stop TOTALLY
        RA_steppingEnabled = 0; // Stop moving, for bench testing
        DEC_steppingEnabled = 0;
        mySerial.println ("Stepping totally halted");
        break;
      // ***************** X **********************
      //
      // ***************** G **********************
      case 'G': // Get Data
        switch (inputString.charAt(2)) {
          case 'Z': // Azimuth
            Serial.print(getDMS(azimuth,3));
            //mySerial.println(getDMS(azimuth,3));
            break;
          case 'D': // Declination
            Serial.print(getDMS(declination,2));
            //mySerial.println(getDMS(declination,2));
            break;
          case 'S': // Sidereal Time 
            Serial.print(getHMS(LST_hours));
            //mySerial.println(getHMS(LST_hours));
            break;
          case 'A': // Elevation
            Serial.print(getDMS(elevation,2));
            //mySerial.println(getDMS(elevation,2));
            break;
          case 'R': // right ascension
            Serial.print(getHMS(RA));
            //mySerial.println(getHMS(RA));
            break;
          //break;
          case 'V':
            switch (inputString.charAt(3)) {
              case 'F':
                Serial.println("HomebrewStar"); // Bogus placeholder FIXME
                mySerial.println("HomebrewStar"); 
           break; 
            } // CaseGV Char3       
          break; // CaseG
        } // CaseG Char2
        // ***************** G **********************
       break; // FC Break
      } // Ending First Character Loop
    } // Ending Init Character Loop
} // Ending Function


void LST_time(){
    //Calculates local sidereal time based on this calculation,
    //http://www.stargazing.net/kepler/altaz.html 
    
    DateTime now = rtc.now();
    
    M = (double) now.month();
    Y = (double) now.year();
    D = (double) now.day();
    MN = (double) now.minute();
    H = (double) now.hour();
    S = (double) now.second();
    A = (double)(Y-2000)*365.242199;
    B = (double)(M-1)*30.4368499;
    double JDN2000=A+B+(D-1) + H/24;
    double decimal_time = H+(MN/60)+(S/3600) ;
    double LST = 100.46 + 0.985647 * JDN2000 + longitude + 15*decimal_time;
    LST_degrees = (LST-(floor(LST/360)*360));
    LST_hours = LST_degrees/15;
}

// Return HH:MM:SS#
// The input must be in decimal hours
String getHMS(double h_dec)  {
  int h;
  int m;
  float s;
  String result;
  
  h = int(h_dec);                 // hours (int)
  m = int((float)(h_dec - h) * 60.0);      // minutes
  s = (float(h_dec) - float(h) - float(m)/60.0) * 3600.0;  // seconds

  result = pad_int(h,2) + ':' + pad_int(m,2) + ':' + pad_int(int(s),2) + '#';
  return result;
}

// Return sDD*MM'SS# or sDDD*MM'SS#
// The input must be in decimal degrees
// place is the number of digits in the degrees format to return
// Azimuth has 3 places, where declination has 2
String getDMS(double d_dec, int places)  {
  int d;
  int m;
  float s;
  String sign;		// the polarity of the value
  String result;
  
  d = int(d_dec);                 // hours (int)
  m = int((float)(d_dec - d) * 60.0);      // minutes
  s = (float(d_dec) - float(d) - float(m)/60.0) * 3600.0;  // seconds

  // make everything positive, we already know the sign
  d = abs(d);
  m = abs(m);
  s = abs(s);

  // Determine the sign
  if(d_dec < 0.0) {
      sign = "-";
  } else {
      sign = '+';
  }

  // Build the result string - note the backslash escape
  // Angles with 2 places get a sign
  if(places == 2)
    result = sign + pad_int(d,places) + '*' + pad_int(m,2) + '\'' + pad_int(int(s),2) + '#';
  // Angles with 3 places (azimuth) don't get a sign
  if(places == 3)
    result = pad_int(d,places) + '*' + pad_int(m,2) + '\'' + pad_int(int(s),2) + '#';
  
  return result;
}

// Takes an integer value, and pads leading zeros
// to the number of places
String pad_int(int value, int places) {
  char data[4];   // max size for a 3-digit value
  String result;
  String format;

  if (places == 2) {
    sprintf(data, "%02d", value);
  }
  else if (places == 3) {
    sprintf(data, "%03d", value);
  }
    
  result = data;
  return result;
    
}

void updateAzimuth() {
  azimuth = initialAz + azEncRes * azEncoderCount;
}

void updateElevation() {
  elevation = initialEl + elEncRes * elEncoderCount;
}

void updateEncoders() {
  azEncoderCount = getEncoderPosition(0);
  delay(100);
  elEncoderCount = getEncoderPosition(1);
}

// Given current azimuth/elevation,
// update RA and Dec
// These are all global variables 
// From "Practical Astronomy With Your Calculator", Peter-Duffet Smith, pg. 38
//
// DEFINITELY SIMPLIFY THIS AFTER GETTING IT TO WORK, LOTS OF DOUBLES
void updateEqu()  {
  double decR;    	// declination angle, in radians
  double latR;    	// latitude in radians
  double azR;     	// azimuth angle, in radians
  double altR;    	// elevation angle, in radians 
  double sinDec;  	// sin(declination)
  double cosHA;   	// cosine of the hour angle
  double haRPrime;	// hour angle, in radians  
  double sinA;		  // sin(azimuth)
  double haD;		    // hour angle,  in degrees
  double haH;       // hour angle, hours
  double decD;		  // declination angle, in degrees
  double raD;		    // right ascension, degrees

  // convert all angles in radians before using trig functions
  latR = deg2rad(latitude);
  azR = deg2rad(azimuth);
  altR = deg2rad(elevation);

  // solve sin(dec) = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az)
  sinDec = sin(altR) * sin(latR) + cos(altR) * cos(latR) * cos(azR);
  
  // Declination, in radians
  decR = asin(sinDec);
  // Convert dec to degrees
  decD = rad2deg(decR);

  // solve cos(hourAngle) = (sin(alt) - sin(lat)*sin(dec) )/(cos(lat) * cos(dec) )
  cosHA = (sin(altR) - sin(latR) * sinDec)/( cos(latR) * cos(decR) );
  
  // hour angle, unadjusted
  haRPrime = acos(cosHA);

  // find sin(azimuth) 
  sinA = sin(azR);
  
  // If sin(A) is negative, true hour angle is H'
  // if positive, true hour angle is 360 - H'
  if (sinA < 0)	{
	haD = rad2deg(haRPrime);
  } else if (sinA > 0)	{
	haD = 360.0 - rad2deg(haRPrime);
  }

  // convert H into hours by dividing by 15 (15 degrees per hour)
  haH = haD/15.0;

  // Now find the right ascension
  // LST - H
  raD = LST_hours - haH;
  // If the result is negative, add 24 (hours)
  if (raD < 0)
    raD += 24;
    
  // Update the global variables
  RA = raD;
  declination = decD;
  
}
// Calculate the RA traking speed
double getAzSpeed()  {
  double latR;  // latitude in radians
  double zR;    // zenith distance in radians
  double azR;   // azimuth in radians
  double pi = 3.14159265; // needs no introduction
  double azVelocity;
  
  // Convert angles to radians
  latR = deg2rad(latitude);
  azR = deg2rad(azimuth);
  zR = deg2rad(pi/2 - elevation);
  
  azVelocity = rad2deg((sin(latR)*sin(zR) - cos(latR)*cos(zR)*cos(azR))/sin(zR));
  return azVelocity;
}

double deg2rad(double deg) {
  return (deg * 1000 / 57296);
}

double rad2deg(double rad)  {
  return (rad * 57296 / 1000);
}

// *************** STEPPER MOTION COMMANDS START************************

// Move relative
void relativeMove(int axis, long steps)  {
  if (axis == 0)          // azimuth axis
  { 
    if (steps > 0)
      azMovingCW = true;        // rotating clockwise
    else if (steps < 0)
      azMovingCCW = true;       // rotating anti-clockwise

    // Don't run the motor against a hard stop
    if ((azMovingCW && !azCWLimit) || (azMovingCCW && !azCCWLimit)) {
      stepperX.enableOutputs();
      movingX = true;
      stepperX.move(steps);
      //mySerial.println("az not in limit");
    }
    
  }
  else if (axis == 1)     // elevation axis
  {
    if (steps > 0)
      elMovingUp = true;        // rotating up
    else if (steps < 0)
      elMovingDown = true;      // rotating down

    // Don't run the motor against a hard stop
    if ((elMovingDown && !elDownLimit) || (elMovingUp && !elUpLimit)) {
      stepperY.enableOutputs();
      movingY = true;
      stepperY.move(steps);
      //mySerial.println("el not in limit");
    }

  }
  
}

// Set maximum
void setMaxSpeed(int axis, int speed)  {
  
  if (axis == 0)
    stepperX.setMaxSpeed(speed);  
  else if (axis == 1) 
    stepperY.setMaxSpeed(speed);  

  //mySerial.println("Set max speed");
}

// Sets desired speed for runSpeed()
void setSpeed(int axis, int speed)  {
  if (axis == 0)
    stepperX.setSpeed((float)speed);
  else if (axis == 1) 
    stepperX.setSpeed((float)speed);
    
  //mySerial.println("Set speed");;
}

void runSpeed(int axis)  {     // must add hard limit checks 
  if (axis == 0)
    slewingX = true;
  else if (axis == 1)
    slewingY = true;

  //mySerial.println("Run speed");
}

void stop(int axis)  {
  if (axis == 0)
  {
    stepperX.stop();
    stoppingXSlew = true;
    slewingX = false;
  } 
  else if (axis == 1)
  {
    stepperY.stop();
    stoppingYSlew = true;
    slewingY = false;
  }

  //mySerial.println("Stop");
}

int setAccel(int axis, int accel)  {
  if (axis == 0)
    stepperX.setAcceleration((float)accel);
  else if (axis == 1)
    stepperY.setAcceleration((float)accel);
    
  //mySerial.println("Set accel");
}

long getCurPosition(int axis){
  if (axis == 0)
    return stepperX.currentPosition();
  else if (axis == 1)
    return stepperY.currentPosition();
}

void setCurPosition(int axis, long position) {   // add axis as an argument
  if (axis == 0)
    stepperX.setCurrentPosition(position);
  else if (axis == 1)
    stepperY.setCurrentPosition(position);
    
  //mySerial.println("set position");
}

void moveToAbsolute(int axis, long absolute) {   // must add hard limits check here
  if (axis == 0)
  {
    stepperX.moveTo(absolute);
    movingX = true;
  }
  else if (axis == 1)
  {
    stepperY.moveTo(absolute);
    movingY = true;
  }
  
  // moving = true;
  
  //mySerial.println("move to abs");
}




// *************** STEPPER MOTION COMMANDS END  ************************

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    // Is it the Ack character?
   if (inChar == 6) 
  { 
    mySerial.println("GotAck - SendingA");
    Serial.println("A"); // Altaz Mode
    inputString = ""; //
    return;
  } 
    // add it to the inputString:
    inputString += inChar;

    if (inputString.startsWith(":"))   
    {
      if (inputString.endsWith("#"))   
      {
      // Starts with :, ends with # - OK
      stringComplete = true;
      parseLX200(inputString); // Interpret string and act - do this now as waiting may may lead to another command arriving before we do
    // clear the string:
    inputString = "";
    stringComplete = false;
      }
    }
    else 
    {
    // If it doesnt start with !, not valid, throw it away..
    inputString = "";
    } 
   }

}

/*
 * Gets the value if the limit swtich
 * axis = 0 if azimuth, 1 if elevation
 *   
 *   axis       limit    meaning
 *   ----       -----    -------
 *   azimuth     0       max CW
 *   azimuth     1       max CCW
 *   elevation   0       max UP
 *   elevation   1       max DOWN
 *   
 *   returns     1       limit reached
 *               0       limit not reached
 *   logic is inverted with internal pullup          
 *     
 *   This code could use some optimization!
*/
int getLimit(int axis, int limit) {
   if (axis == 0) {                         // azimuth
    if (limit == 0) {
      if (digitalRead(azLimitCW) == HIGH)   // CW limit not reached
        return 0;

      if (digitalRead(azLimitCW) == LOW)    // CW limit reached
        return 1;
        
    } else if (limit == 1) {
      if (digitalRead(azLimitCCW) == HIGH)  // CCW limit not reached
        return 0;

      if (digitalRead(azLimitCCW) == LOW)   // CCW limit reached
        return 1;
    }
  } else if (axis == 1) {
    if (limit == 0) {
      if (digitalRead(elLimitUp) == HIGH)   // UP limit not reached
        return 0;

      if (digitalRead(elLimitUp) == LOW)    // UP limit reached
        return 1;
        
    } else if (limit == 1) {
      if (digitalRead(elLimitDown) == HIGH) // DOWN limit not reached
        return 0;

      if (digitalRead(elLimitDown) == LOW)  // DOWN limit reached
        return 1;
    }
  }
}

// Timer1 interrupt at 2kHz services stepper motion 
ISR(TIMER1_COMPA_vect){
  if(stepperXRun) {
    stepperX.run();
    //Serial.println('a');
  }
    
  if(stepperYRun) {
    stepperY.run();
    //Serial.println('e');
  }
}

void setup() {
 
  // Setup timer1 to interrupt at 2 kHz
  cli();//stop interrupts
  
  TCCR1A = 0;// set entire TCCR2A register to 0
  TCCR1B = 0;// same for TCCR2B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  //OCR1A = 62;// = (16*10^6) / (2000*64) - 1 (must be <65536)
  OCR1A = 15;// = (16*10^6) / (2000*64) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
  
  // Initialize I2C communications as Master
  Wire.begin();
  
  // Setup serial monitor
  Serial.begin(9600);

  // Serial dubugging port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");

  // Stepper setup

  // Hard limits
  pinMode(azLimitCW, INPUT_PULLUP);
  pinMode(azLimitCCW, INPUT_PULLUP);
  pinMode(elLimitUp, INPUT_PULLUP);
  pinMode(elLimitDown, INPUT_PULLUP);
  
  // Setup stepper enable pins
  stepperX.setEnablePin(7);   // using an enable pin
  stepperY.setEnablePin(7);

  delay(3000); // wait for console opening

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  /*
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  */

  rtc.adjust(DateTime(__DATE__, __TIME__));  // sets the clock to the time when this sketch is compiled
  delay(1000);
  DateTime now = rtc.now();
  DateTime UTCTime(now.unixtime() - 3600 * UTCOffset);   // Adjust the time from local to UTC
  rtc.adjust(UTCTime);

  // Set up encoder max speeds and accels
  setMaxSpeed(0, 2000);     // azimuth
  setMaxSpeed(1, 2000);     // elevation
  setAccel(0, 1000);         // azimuth
  setAccel(1, 1000);         // elevation
  setSpeed(0, 2000);        // azimuth
  setSpeed(1, 2000);        // elevation

  // For debugging - print data once a second
  startMillis = millis();  //initial start time

  mySerial.println("Setup complete");
}
 
void loop() {

  // Update the RTC
  DateTime now = rtc.now();

  // Update LST
  LST_time();
  updateEncoders();
  updateAzimuth();
  updateElevation();
  updateEqu();

  // For debugging - print once per second
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    mySerial.println(getAzSpeed(), 4);
    
    startMillis = currentMillis;  //IMPORTANT to save the start time
  }
  

  // ******************************* Motion ***********************************
  
  // Relative move - az
  if (movingX && (abs(stepperX.distanceToGo())) > 0) {
    //stepperX.run();
    stepperXRun = true;
  } else  {
    movingX = false;
    stepperXRun = false;
  }
  
  // Relative move - el
  if (movingY && (abs(stepperY.distanceToGo())) > 0) {
    //stepperY.run();
    stepperYRun = true;
  } else  {
    movingY = false;
    stepperYRun = false;
  }


  // ******************************* Motion ***********************************


  // ************************* Check hard limits ******************************
  if ((getLimit(0,0) == 0) && azMovingCW) {           // az max CW
    stepperX.disableOutputs();                  // stop as fast as possible
    azMovingCW = false;
    azCWLimit = true;
  } else if ((getLimit(0,1) == 0) && azMovingCCW) {   // az max CCW
    stepperX.disableOutputs();                  // stop as fast as possible
    azMovingCCW = false;
    azCCWLimit = true;
  } else {                                      // reset flags
    azCWLimit = false;
    azCCWLimit = false;
  }

  // Check hard limits
  if ((getLimit(1,0) == 0) && elMovingUp) {           // el max up
    stepperY.disableOutputs();                  // stop as fast as possible
    elMovingUp = false;
    elUpLimit = true;
  } else if ((getLimit(1,1) == 0) && elMovingDown) {  // el max down
    stepperY.disableOutputs();                  // stop as fast as possible
    elMovingDown = false;
    elDownLimit = true;
  } else {                                      // reset flags
    elUpLimit = false;
    elDownLimit = false;
  }
  
  // ************************* Check hard limits ******************************

  
  /*
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print("\t");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  Serial.print("LST:\t");
  Serial.println(getHMS(LST_hours));
  delay(500);
  Serial.print("Az:\t");
  Serial.println(azimuth, 4);
  delay(500);
  Serial.print("El:\t");
  Serial.println(elevation, 4);
  delay(500);
  //Serial.print("AzEnc:\t");
  //Serial.println(azEncoderCount);
  //delay(500);
  //Serial.print("ElEnc:\t");
  //Serial.println(elEncoderCount);
  //delay(500);
  Serial.print("RA:\t");
  Serial.println(RA, 4);
  delay(500);
  Serial.print("Dec:\t");
  Serial.println(declination, 4);
  delay(500);
  Serial.println("");

  Serial.print("LST:\t");
  Serial.println(LST_hours, 4);
  
  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
  Serial.println("");

  delay(1000); 
  // Print to Serial Monitor
  Serial.print("Azimuth:\t");
  Serial.println(getEncoderPosition(0));
  delay(250);
  Serial.print("Elevation:\t");
  Serial.println(getEncoderPosition(1));
  mySerial.println("Test..");
  */
  
}
