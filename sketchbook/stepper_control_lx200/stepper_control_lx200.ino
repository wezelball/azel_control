

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

float UTCOffset = -4.0;     // your timezone relative to UTC

double M,Y,D,MN,H,S;        // current UTC time
double A,B;
double longitude =-77.924;  // your longtitude.
double latitude =37.791;    // your latitude.
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
double azEncRes = 1.667e-5; // azimuth encoder resolution in degrees per pulse
double elEncRes = 1.667e-5; // elevation encoder resolution in degrees per pulse

int RA_step_direction = 1;
int DEC_step_direction = 1;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean RA_tracking_Enabled = true; // Always track - Currently unused
boolean RA_steppingEnabled = true; // Non-default RA movement if we are doing something other than tracking
boolean DEC_steppingEnabled = false; // DEC movement is always optional

// ****************************** END GLOBALS ***************************************


// encoder = 0    azimuth
// encoder = 1    elavation
long getEncoderPosition(int encoder) {
  // Write a command to the Slave
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((byte)encoder);
  Wire.endTransmission();

  // Read response from Slave
  // Read back ANSWERSIZE characters
  Wire.requestFrom(SLAVE_ADDR,ANSWERSIZE);
  
  // Add characters to string
  String response = "";
  while (Wire.available()) {
      char b = Wire.read();
      //Serial.println(b);
      response += b;
  }

  return response.toInt();
}

void parseLX200(String thisCommand)
{
 //mySerial.println(thisCommand);
 mySerial.println(inputString);
 switch (inputString.charAt(0)) { // If this isnt 58 (:), something is wrong!
     case ':':
      switch (inputString.charAt(1)) {
        case 'S':// Set Stuff
          switch (inputString.charAt(2)) {
              case 'w': // Slew rate
                Serial.println(1);
                mySerial.println("set max slew");
              break;
          }// end :Sw
          break; //Case S Char2
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
        case 'M':  // Movement Control - M
          switch (inputString.charAt(2)) {
          case 'w':
            RA_step_direction = 1;
            RA_steppingEnabled = true;
            // We really just need to speed things up
            //RA_StepInterval = (initialRA_StepInterval /4);
            mySerial.println("Move RA forwards (west)");
          break;
          case 'e':
            RA_step_direction = 1;
            RA_steppingEnabled = true;
            // We really just need to slow things down
            //RA_StepInterval = (initialRA_StepInterval *4);
            mySerial.println("Move RA backwards (east) ");
          break;
          case 'n':
            DEC_step_direction = 0;
            DEC_steppingEnabled = true;
            mySerial.println("Move DEC forwards (north)");
          break;
          case 's':
            DEC_step_direction = 1;
            DEC_steppingEnabled = true;
            mySerial.println("Move DEC backwards (south)");
          break;
          } // CaseM Char2
        break; // End movemont control
      case 'Q': // Stop Moving - Q
        RA_steppingEnabled = 1; // We still move RA 
        //RA_StepInterval = initialRA_StepInterval; // We just set the speed so that stars should be "stationary" relative to everything else
        DEC_steppingEnabled = 0;
        mySerial.println ("Stepping halted");
        break;
      case 'X': // Stop TOTALLY
  RA_steppingEnabled = 0; // Stop moving, for bench testing
  DEC_steppingEnabled = 0;
  mySerial.println ("Stepping totally halted");
  break;
      case 'G': // Get Data
        switch (inputString.charAt(2)) {
          case 'Z': // Azimuth
            Serial.print("123*56\'57#"); // Bogus placeholder FIXME
            mySerial.println("sent az");
            break;
          case 'D': // Declination
            Serial.print("+68*58\'17#"); // Bogus placeholder FIXME
            mySerial.println("sent dec");
            break;
          case 'S': // Sidereal Time 
            Serial.print(getHMS(LST_hours));
            mySerial.println(getHMS(LST_hours));
            break;
          case 'A': // Elevation
            Serial.print("+30*25\'40#");
            mySerial.println("+30*25\'40#");
            //mySerial.println("sent el");
            break;
          case 'R': // right ascension
            Serial.print("09:57:04#");
            mySerial.println("sent RA");
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

// Return H:M:S#
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
  //double haR;		  // hour angle, in radians
  double haD;		    // hour angle,  in degrees
  double haH;       // hour angle, hours
  double decD;		  // declination angle, in degrees
  double raD;		    // right ascension, degrees

  // Test variables for debugging
  /*
  double latitude_test = 52.0;
  double elevation_test = 19.334444;
  double azimuth_test = 283.271111;
  double LST_hours_test = 0.401389;
  */
  

  // convert all angles in radians before using trig functions
  //latR = deg2rad(latitude_test);
  latR = deg2rad(latitude);
  //azR = deg2rad(azimuth_test);
  azR = deg2rad(azimuth);
  //altR = deg2rad(elevation_test);
  altR = deg2rad(elevation);

  // solve sin(dec) = sin(alt) * sin(lat) + cos(alt) * cos(lat) * cos(az)
  sinDec = sin(altR) * sin(latR) + cos(altR) * cos(latR) * cos(azR);
  //mySerial.print("sinDec:\t");
  //mySerial.println(sinDec, 4);
  
  // Declination, in radians
  decR = asin(sinDec);
  // Convert dec to degrees
  decD = rad2deg(decR);
  //mySerial.print("Dec:\t");
  //mySerial.println(decD, 4);

  // solve cos(hourAngle) = (sin(alt) - sin(lat)*sin(dec) )/(cos(lat) * cos(dec) )
  cosHA = (sin(altR) - sin(latR) * sinDec)/( cos(latR) * cos(decR) );
  //mySerial.print("cosHA:\t");
  //mySerial.println(cosHA, 4);
  
  // hour angle, unadjusted
  haRPrime = acos(cosHA);
  // find sin(azimuth) 
  sinA = sin(azR);
  //mySerial.print("sinA:\t");
  //mySerial.println(sinA, 4);
  
  
  // If sin(A) is negative, true hour angle is H'
  // if positive, true hour angle is 360 - H'
  if (sinA < 0)	{
	//haR = haRPrime;
	haD = rad2deg(haRPrime);
  } else if (sinA > 0)	{
	haD = 360.0 - rad2deg(haRPrime);
  }

  //mySerial.print("haD:\t");
  //mySerial.println(haD, 4);

  // convert H into hours by dividing by 15 (15 degrees per hour)
  haH = haD/15.0;

  //mySerial.print("haH:\t");
  //mySerial.println(haH, 4);
  
  // Now find the right ascension
  // LST - H
  //raD = LST_hours_test - haH;
  raD = LST_hours - haH;
  // If the result is negative, add 24 (hours)
  if (raD < 0)
    raD += 24;
  
  //mySerial.print("raD:\t");
  //mySerial.println(raD, 4);
  
  // Update the global variables
  RA = raD;
  declination = decD;
  
}


double deg2rad(double deg) {
  return (deg * 1000 / 57296);
}

double rad2deg(double rad)  {
  return (rad * 57296 / 1000);
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    //mySerial.print("rcv: ");
    //mySerial.println(inChar);
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
    //mySerial.println("Something");
    //mySerial.println(inChar);
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

void setup() {
 
  // Initialize I2C communications as Master
  Wire.begin();
  
  // Setup serial monitor
  Serial.begin(9600);

  mySerial.begin(9600);
  mySerial.println("Hello, world?");

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
  //updateRA();
  //updateDec();

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
  /* 
  // Fun test code
  

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
  // Fun test code
  */
  
}
