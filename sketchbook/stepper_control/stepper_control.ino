/*****************************************************************************

  How to Use CommandLine:
    Create a sketch.  Look below for a sample setup and main loop code and copy and paste it in into the new sketch.

   Create a new tab.  (Use the drop down menu (little triangle) on the far right of the Arduino Editor.
   Name the tab CommandLine.h
   Paste this file into it.

  Test:
     Download the sketch you just created to your Arduino as usual and open the Serial Window.  Typey these commands followed by return:
      add 5, 10
      subtract 10, 5

    Look at the add and subtract commands included and then write your own!


*****************************************************************************
  Here's what's going on under the covers
*****************************************************************************
  Simple and Clear Command Line Interpreter

     This file will allow you to type commands into the Serial Window like,
        add 23,599
        blink 5
        playSong Yesterday

     to your sketch running on the Arduino and execute them.

     Implementation note:  This will use C strings as opposed to String Objects based on the assumption that if you need a commandLine interpreter,
     you are probably short on space too and the String object tends to be space inefficient.

   1)  Simple Protocol
         Commands are words and numbers either space or comma spearated
         The first word is the command, each additional word is an argument
         "\n" terminates each command

   2)  Using the C library routine strtok:
       A command is a word separated by spaces or commas.  A word separated by certain characters (like space or comma) is called a token.
       To get tokens one by one, I use the C lib routing strtok (part of C stdlib.h see below how to include it).
           It's part of C language library <string.h> which you can look up online.  Basically you:
              1) pass it a string (and the delimeters you use, i.e. space and comman) and it will return the first token from the string
              2) on subsequent calls, pass it NULL (instead of the string ptr) and it will continue where it left off with the initial string.
        I've written a couple of basic helper routines:
            readNumber: uses strtok and atoi (atoi: ascii to int, again part of C stdlib.h) to return an integer.
              Note that atoi returns an int and if you are using 1 byte ints like uint8_t you'll have to get the lowByte().
            readWord: returns a ptr to a text word

   4)  DoMyCommand: A list of if-then-elses for each command.  You could make this a case statement if all commands were a single char.
      Using a word is more readable.
          For the purposes of this example we have:
              Add
              Subtract
              nullCommand
*/
/******************sample main loop code ************************************

  #include "CommandLine.h"

  void
  setup() {
  Serial.begin(115200);
  }

  void
  loop() {
  bool received = getCommandLineFromSerialPort(CommandLine);      //global CommandLine is defined in CommandLine.h
  if (received) DoMyCommand(CommandLine);
  }

**********************************************************************************/


/*************************************************************************************************************
    getCommandLineFromSerialPort()
      Return the string of the next command. Commands are delimited by return"
      Handle BackSpace character
      Make all chars lowercase
*************************************************************************************************************/
// ****************************** ADAFRUIT *************************************************
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include<utility/imumaths.h>
// ****************************** ADAFRUIT *************************************************

#include "CommandLine.h"
#include "AccelStepper.h" 


// Library created by Mike McCauley at http://www.airspayce.com/mikem/arduino/AccelStepper/

// *********************************** Global vars here **********************************
// AccelStepper Setup
AccelStepper stepperX(1, 5, 6);   // X is the azimuth axis
                                  // 1 = Easy Driver interface
                                  // UNO Pin 5 connected to STEP pin of Easy Driver
                                  // UNO Pin 6 connected to DIR pin of Easy Driver

AccelStepper stepperY(1, 8, 9);   // Y is the elevation axis
                                  // 1 = Easy Driver interface
                                  // UNO Pin a connected to STEP pin of Easy Driver
                                  // UNO Pin b connected to DIR pin of Easy Driver


// Adafruit orientation sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);                                  


// Hard limits 
int azLimitCW = 2;
int azLimitCCW = 4;
int elLimitUp = 3;
int elLimitDown = 10;

// Hard limit flags
bool azCW = false;          // az rotating clockwise 
bool azCCW = false;         // az rotating anti-clockwise
bool azCWLimit = false;     // at the CW limit 
bool azCCWLimit = false;    // at the CW limit 

bool elUp = false;          // elevation rotating upwards
bool elDown = false;        // elevation rotating downwards
bool elDownLimit = false;   // at the down limit
bool elUpLimit = false;     // at the up limit


// Motion flags
bool stoppingXSlew = false;  // 
bool stoppingYSlew = false;  // 
bool slewingX = false;       // constant speed motion
bool slewingY = false;       // constant speed motion
bool movingX = false;        // either an absolute or relative move
bool movingY = false;        // either an absolute or relative move




// *********************************** Global vars here **********************************

bool getCommandLineFromSerialPort(char * commandLine)
{
  static uint8_t charsRead = 0;                      //note: COMAND_BUFFER_LENGTH must be less than 255 chars long
  //read asynchronously until full command input
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
      case LF:
        commandLine[charsRead] = NULLCHAR;       //null terminate our command char array
        if (charsRead > 0)  {
          charsRead = 0;                           //charsRead is static, so have to reset
          //Serial.println(commandLine);
          return true;
        }
        break;
      case BS:                                    // handle backspace in input: put a space in last char
        if (charsRead > 0) {                        //and adjust commandLine and charsRead
          commandLine[--charsRead] = NULLCHAR;
          Serial << byte(BS) << byte(SPACE) << byte(BS);  //no idea how this works, found it on the Internet
        }
        break;
      default:
        // c = tolower(c);
        if (charsRead < COMMAND_BUFFER_LENGTH) {
          commandLine[charsRead++] = c;
        }
        commandLine[charsRead] = NULLCHAR;     //just in case
        break;
    }
  }
  return false;
}


/* ****************************
   readNumber: return a 16bit (for Arduino Uno) signed integer from the command line
   readWord: get a text word from the command line

*/
int
readIntNumber () {
  char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
  return atoi(numTextPtr);                              //K&R string.h  pg. 251
}

int
readLongNumber () {
  char * numTextPtr = strtok(NULL, delimiters);         //K&R string.h  pg. 250
  return atol(numTextPtr);                              //K&R string.h  pg. 251
}

char * readWord() {
  char * word = strtok(NULL, delimiters);               //K&R string.h  pg. 250
  return word;
}

void
nullCommand(char * ptrToCommandName) {
  print2("Command not found: ", ptrToCommandName);      //see above for macro print2
}


/****************************************************
   Add your commands here
*/

int accelCommand() {
  int firstOperand = readIntNumber();
  int secondOperand = readIntNumber();    
  return setAccel(firstOperand, secondOperand);
}

int setSpeedCommand()  {
  int firstOperand = readIntNumber();
  int secondOperand = readIntNumber();
  return setSpeed(firstOperand, secondOperand);
}

int setMaxSpeedCommand()  {
  int firstOperand = readIntNumber();
  int secondOperand = readIntNumber();
  return setMaxSpeed(firstOperand, secondOperand);
}

int moveRelativeCommand() {
  int firstOperand = readIntNumber();
  long secondOperand = readLongNumber();
  return relativeMove(firstOperand, secondOperand);
}

int runSpeedCommand() {
  int firstOperand = readIntNumber();
  return runSpeed(firstOperand);
}

int stopCommand() {
  int firstOperand = readIntNumber();
  return stop(firstOperand);  
}

long getCurPosCommand() {
  int firstOperand = readIntNumber();
  return getCurPosition(firstOperand);
}

int setCurPosCommand()  {
  //long firstOperand = readLongNumber();
  int firstOperand = readIntNumber();
  long secondOperand = readLongNumber();
  return setCurPosition(firstOperand, secondOperand);
}

int moveToCommand() {                       // add axis as an argument
  int firstOperand = readIntNumber();
  long secondOperand = readLongNumber();
  return moveToAbsolute(firstOperand, secondOperand);
}

int pingCommand() {
  return ping();
}

float getImuCommand() {
  int firstOperand = readIntNumber();
  return getImu(firstOperand);
}

int getAnalogPotCommand() {
  int firstOperand = readIntNumber();
  return getAnalogPot(firstOperand);
}

int getLimitCommand() {
  int firstOperand = readIntNumber();
  int secondOperand = readIntNumber();
  return getLimit(firstOperand,secondOperand);
}


/****************************************************
   DoMyCommand
*/
bool DoMyCommand(char * commandLine) {
  //  print2("\nCommand: ", commandLine);
  int result;
  float floatResult;

  char * ptrToCommandName = strtok(commandLine, delimiters);
  //  print2("commandName= ", ptrToCommandName);

  if (strcmp(ptrToCommandName, stopCommandToken) == 0) {                 
    result = stopCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, setMaxSpeedToken) == 0) {           
    result = setMaxSpeedCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, moveRelCommandToken) == 0) {        
     result = moveRelativeCommand();
     Serial.println(result);
  } else if (strcmp(ptrToCommandName, setSpeedToken) == 0) {
    result = setSpeedCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, runSpeedToken) == 0) {
    result = runSpeedCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, accelCommandToken) == 0) {
    result = accelCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, getCurPosCommandToken) == 0) {
    result = getCurPosCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, setCurPosCommandToken) == 0) {    
    result = setCurPosCommand();
    Serial.println(result);                                             
  } else if (strcmp(ptrToCommandName, moveToCommandToken) == 0) {       // copy-paste here
    result = moveToCommand();
    Serial.println(result);                                             // copy-paste here
  } else if (strcmp(ptrToCommandName, pingCommandToken) == 0) { 
    result = pingCommand();
    Serial.println(result);                                             
  } else if (strcmp(ptrToCommandName, getImuCommandToken) == 0) {       
    floatResult = getImuCommand();
    Serial.println(floatResult, 2);
  } else if (strcmp(ptrToCommandName, getAnalogPotToken) == 0) {       
    result = getAnalogPotCommand();
    Serial.println(result);
  } else if (strcmp(ptrToCommandName, getLimitCommandToken) == 0) {       
    result = getLimitCommand();
    Serial.println(result);
  } else {
    nullCommand(ptrToCommandName);
  }

}

/*
 * This is where the stepper commands are
 * 
 */
// *******************************************
// Moves relative
// I need to put this code in the 
// loop so it doesnt block while running
int relativeMove(int axis, long steps)  {
  if (axis == 0)          // azimuth axis
  { 
    if (steps > 0)
      azCW = true;        // rotating clockwise
    else if (steps < 0)
      azCCW = true;       // rotating anti-clockwise

    // Don't run the motor against a hard stop
    if ((azCW && !azCWLimit) || (azCCW && !azCCWLimit)) {
      stepperX.enableOutputs();
      movingX = true;
      stepperX.move(steps);
    }
    

  }
  else if (axis == 1)     // elevation axis
  {
    if (steps > 0)
      elUp = true;        // rotating up
    else if (steps < 0)
      elDown = true;      // rotating down

    // Don't run the motor against a hard stop
    if ((elDown && !elDownLimit) || (elUp && !elUpLimit)) {
      stepperY.enableOutputs();
      movingY = true;
      stepperY.move(steps);
    }

  }
  
  return 1;
}

int setMaxSpeed(int axis, int speed)  {
  
  if (axis == 0)
    stepperX.setMaxSpeed(speed);  
  else if (axis == 1) 
    stepperY.setMaxSpeed(speed);  

  return 2;
}

// Sets desired speed for runSpeed()
int setSpeed(int axis, int speed)  {
  if (axis == 0)
    stepperX.setSpeed((float)speed);
  else if (axis == 1) 
    stepperX.setSpeed((float)speed);
    
  return 3;
}

int runSpeed(int axis)  {     // must add hard limit checks 
  if (axis == 0)
    slewingX = true;
  else if (axis == 1)
    slewingY = true;

  return 4;
}

int stop(int axis)  {
  //Serial.print("stop");
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

  return 5;
}

int setAccel(int axis, int accel)  {
  if (axis == 0)
    stepperX.setAcceleration((float)accel);
  else if (axis == 1)
    stepperY.setAcceleration((float)accel);
    
  return 6;
}

long getCurPosition(int axis){
  if (axis == 0)
    return stepperX.currentPosition();
  else if (axis == 1)
    return stepperY.currentPosition();
}

int setCurPosition(int axis, long position) {   // add axis as an argument
  if (axis == 0)
    stepperX.setCurrentPosition(position);
  else if (axis == 1)
    stepperY.setCurrentPosition(position);
    
  return 7;
}

int moveToAbsolute(int axis, long absolute) {   // must add hard limits check here
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
  
  return 8;
}

int ping()  {
  return 9;
}

float getImu(int axis)  {
  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  if (axis == 0)
    return (float)(event.orientation.x);
  else if (axis == 1)
    return (float)(event.orientation.z);
}

int getAnalogPot(int axis)  {
    int value;

    if (axis == 0)
      value  = analogRead(A0);
    else if (axis == 1)
      value  = analogRead(A1);

    return value;
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

// **********************************************
void setup() {
  Serial.begin(115200);        // serial port    

  // Hard limits
  pinMode(azLimitCW, INPUT_PULLUP);
  pinMode(azLimitCCW, INPUT_PULLUP);
  pinMode(elLimitUp, INPUT_PULLUP);
  pinMode(elLimitDown, INPUT_PULLUP);
  
  // Setup stepper enable pins
  stepperX.setEnablePin(7);   // using an enable pin
  stepperY.setEnablePin(7);

  // Initialize the IMU sensor
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {

  // Look for serial data from port
  bool received = getCommandLineFromSerialPort(CommandLine);      //global CommandLine is defined in CommandLine.h
  if (received) DoMyCommand(CommandLine);

  // Maintains slewing if called for
  if (slewingX || stoppingXSlew)  {
    stepperX.runSpeed();
  } else if (stoppingXSlew && !stepperX.isRunning()) {
    stoppingXSlew = false;
  } 

  // Maintains slewing if called for
  if (slewingY || stoppingYSlew)  {
    stepperY.runSpeed();
  } else if (stoppingYSlew && !stepperY.isRunning()) {
    stoppingYSlew = false;
  }

  // Relative move
  if (movingX && (abs(stepperX.distanceToGo())) > 0) {
    stepperX.run();
  } else  {
    movingX = false;
  }

  // Relative move
  if (movingY && (abs(stepperY.distanceToGo())) > 0) {
    stepperY.run();
  } else  {
    movingY = false;
  }

  // Check hard limits
  if ((getLimit(0,0) == 0) && azCW) {           // az max CW
    stepperX.disableOutputs();                  // stop as fast as possible
    azCW = false;
    azCWLimit = true;
  } else if ((getLimit(0,1) == 0) && azCCW) {   // az max CCW
    stepperX.disableOutputs();                  // stop as fast as possible
    azCCW = false;
    azCCWLimit = true;
  } else {                                      // reset flags
    azCWLimit = false;
    azCCWLimit = false;
  }

  // Check hard limits
  if ((getLimit(1,0) == 0) && elUp) {           // el max up
    stepperY.disableOutputs();                  // stop as fast as possible
    elUp = false;
    elUpLimit = true;
  } else if ((getLimit(1,1) == 0) && elDown) {  // el max down
    stepperY.disableOutputs();                  // stop as fast as possible
    elDown = false;
    elDownLimit = true;
  } else {                                      // reset flags
    elUpLimit = false;
    elDownLimit = false;
  }

  
}
