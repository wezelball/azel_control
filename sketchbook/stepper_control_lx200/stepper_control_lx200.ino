

// Include Arduino Wire library for I2C
#include <Wire.h>
// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"
// #include <SoftwareSerial.h>
#include "AccelStepper.h" 
 
// Define Slave I2C Address
#define SLAVE_ADDR 9
 
// Define Slave answer size
#define ANSWERSIZE 8

// SoftwareSerial mySerial(12, 13); // RX, TX - debug

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

int numcount = 0;
String val = "0";      // This will be used for ascii to integer conversion
unsigned long RA_step_number = 107374182 ; // max long divided by two, for tracking steps.
unsigned long DEC_step_number = 107374182 ; // max long divided by two, for tracking steps.
int RA_step_direction = 1;
int DEC_step_direction = 1;
char cmdEnd;
char cmdBuffer;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean RA_tracking_Enabled = true; // Always track - Currently unused
boolean RA_steppingEnabled = true; // Non-default RA movement if we are doing something other than tracking
boolean DEC_steppingEnabled = false; // DEC movement is always optional


// the follow variables are a long because the time, measured in RA_Microseconds,
// will quickly become a bigger number than can be stored in an int.

// ************************************* DEPRECATED ******************************************

// Init RA timer variables
long previousStepRA_Micros = 0;        // will store last time Stepper was moved
// 10416 gives 1rp60s, 
// 17186 gives 1rev per 100 seconds
// 34372 - 1 rev per 195 seconds?
int minimumRA_StepInterval = 6000; // Stepping delays lower than this generally fail
long initialRA_StepInterval = 34783; // Delay between steps, timed with stopwatch so accuracy may be lacking
int maximumRA_StepInterval = ((initialRA_StepInterval - minimumRA_StepInterval) + initialRA_StepInterval);
unsigned long RA_StepInterval = initialRA_StepInterval;
// Init DEC timer intervals
long previousStepDEC_Micros = 0;        // will store last time Stepper was moved
// 10416 gives 1rp60s, 
// 17186 gives 1rev per 100 seconds
// 34372 - 1 rev per 195 seconds?
int minimumDEC_StepInterval = 6000; // Stepping delays lower than this generally fail
long initialDEC_StepInterval = 12000; // Delay between steps, its a number that works
int maximumDEC_StepInterval = ((initialDEC_StepInterval - minimumDEC_StepInterval) + initialDEC_StepInterval);
unsigned long DEC_StepInterval = initialDEC_StepInterval;
//long autoDrifttimerStart = 0;
long SerialOutputInterval = 100000;
int SerialInputInterval = 10000;
long previousSerialIn_Micros = 0;  // last time serial buffer was polled
long previousSerialOut_Micros = 0;  // Last time serial data was sent via timer

// ************************************* DEPRECATED ******************************************

char cmdSwitch;
char newTime;
String newDelay;
char numberArray[5];

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
 //mySerial.println(inputString);
 switch (inputString.charAt(0)) { // If this isnt 58 (:), something is wrong!
     case ':':
      switch (inputString.charAt(1)) {
        case 'S':// Set Stuff
          switch (inputString.charAt(2)) {
              case 'w': // Slew rate
                Serial.println(1);
                //mySerial.println("set max slew");
              break;
          }// end :Sw
          break; //Case S Char2
        case 'R':// Rate Control - R
          switch (inputString.charAt(2)) {
          case 'C':
            RA_StepInterval = (initialRA_StepInterval *2);
            DEC_StepInterval = (initialDEC_StepInterval *2);
            Serial.println(" Set interval to half default");
          break;
          case 'G':
           RA_StepInterval = initialRA_StepInterval;
           DEC_StepInterval = initialDEC_StepInterval;
           Serial.println(" Set interval to default");
          break;      
          case 'M':
           RA_StepInterval = initialRA_StepInterval/2;
           DEC_StepInterval = initialDEC_StepInterval/2;
           Serial.println(" Set interval to DOUBLE SPEED");
          break;
          case 'S':
           RA_StepInterval = minimumRA_StepInterval;
           DEC_StepInterval = minimumDEC_StepInterval;
           Serial.println(" Set interval to FASTEST");
          break;
      } // CaseR Char2
        break; // End Rate Control
        case 'M':  // Movement Control - M
          switch (inputString.charAt(2)) {
          case 'w':
            RA_step_direction = 1;
            RA_steppingEnabled = true;
            // We really just need to speed things up
            RA_StepInterval = (initialRA_StepInterval /4);
            Serial.println("Move RA forwards (west)");
          break;
          case 'e':
            RA_step_direction = 1;
            RA_steppingEnabled = true;
            // We really just need to slow things down
            RA_StepInterval = (initialRA_StepInterval *4);
            Serial.println("Move RA backwards (east) ");
          break;
          case 'n':
            DEC_step_direction = 0;
            DEC_steppingEnabled = true;
            Serial.println("Move DEC forwards (north)");
          break;
          case 's':
            DEC_step_direction = 1;
            DEC_steppingEnabled = true;
            Serial.println("Move DEC backwards (south)");
          break;
          } // CaseM Char2
        break; // End movemont control
      case 'Q': // Stop Moving - Q
        RA_steppingEnabled = 1; // We still move RA 
        RA_StepInterval = initialRA_StepInterval; // We just set the speed so that stars should be "stationary" relative to everything else
        DEC_steppingEnabled = 0;
        Serial.println ("Stepping halted");
        break;
      case 'X': // Stop TOTALLY
  RA_steppingEnabled = 0; // Stop moving, for bench testing
  DEC_steppingEnabled = 0;
  Serial.println ("Stepping totally halted");
  break;
      case 'G': // Get Data
        switch (inputString.charAt(2)) {
          case 'Z': // Azimuth
            Serial.println("123*56#"); // Bogus placeholder FIXME
            //mySerial.println("sent azimuth");
          case 'D': // Declination
            Serial.println("+23*56#"); // Bogus placeholder FIXME
            //mySerial.println("sent declination");
          case 'S': // Sidereal Time 
            Serial.println("12:34:56#"); // Bogus placeholder FIXME
            //mySerial.println("sent sidereal");
          break;
          case 'V':
            switch (inputString.charAt(3)) {
              case 'F':
                Serial.println("HomebrewStar"); // Bogus placeholder FIXME
                //mySerial.println("HomebrewStar"); 
           break; 
            } // CaseGV Char3       
          break; // CaseG
        } // CaseG Char2
       break; // FC Break
      } // Ending First Character Loop
    } // Ending Init Character Loop
} // Ending Function

void setup() {
 
  // Initialize I2C communications as Master
  Wire.begin();
  
  // Setup serial monitor
  Serial.begin(9600);

  //mySerial.begin(4800);
  //mySerial.println("Hello, world?");

  delay(3000); // wait for console opening

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

}
 
void loop() {

  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

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
    //mySerial.println("GotAck - SendingP");
    Serial.println("P"); // Polar Mode
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
