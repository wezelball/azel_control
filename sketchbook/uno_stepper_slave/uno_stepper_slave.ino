/*
 * uno_stepper_slave.ino
 * Arduino Uno dedicated to az-el stepper motion control
 * Dave Cohen
 * 
 */

// Include Arduino Wire library for I2C
#include <Wire.h>
#include "AccelStepper.h"

// Define Slave answer size
#define ANSWERSIZE 8
#define SLAVE_ADDRESS 0x04

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

long position;
char str[8];
String answer = "";  // must be 8 bytes, or 7 characters
String rawCommand;
int command;    // command from RPi
long param;     // command parameter from RPi

// Hard limits
int azLimitCW = 2;
int azLimitCCW = 4;
int elLimitUp = 10;
int elLimitDown = 3;

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

// ****************************** END GLOBALS ****************************************

// **************************** BEGIN UTILITY FUNCTIONS ******************************

// Accepts full command string from RPi and returns command number as integer
int parseCommand(String cmd) {
  int i;
  String cmdString;

  i = cmd.indexOf(':');
  cmdString = cmd.substring(0, i) + '\0';

  //Serial.print("command: ");
  //Serial.println(cmdString.toInt());
  
  return (int)cmdString.toInt();
}

// Accepts full command string from RPi and returns parameter as long
long parseParameter(String cmd) {
  int i;
  String paramString;  

  i = cmd.indexOf(':');
  paramString = cmd.substring(i+1) + '\0';

  //Serial.print("param: ");
  //Serial.println(paramString.toInt());

  return paramString.toInt();
}


// ***************************** END UTILITY FUNCTIONS *******************************


// *********************** BEGIN STEPPER MOTION COMMANDS *****************************

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
}

// Sets desired speed for runSpeed()
void setSpeed(int axis, int speed)  {
  if (axis == 0)
    stepperX.setSpeed((float)speed);
  else if (axis == 1) 
    stepperX.setSpeed((float)speed);
}

void runSpeed(int axis)  {     // must add hard limit checks 
  if (axis == 0)
    slewingX = true;
  else if (axis == 1)
    slewingY = true;
}

// Stops with deceleration ramp
void stop(int axis)  {
  if (axis == 0)
  {
    stepperX.stop();
    stoppingXSlew = true;
    slewingX = false;
    azMovingCW = false;
    azMovingCCW = false;
  } 
  else if (axis == 1)
  {
    stepperY.stop();
    stoppingYSlew = true;
    slewingY = false;
    elMovingUp = false;
    elMovingDown = false;
  }

}

// There is a bug when this function is called
// and a later move is performed, it starts
// at full speed with no accel
/*
 * Stops immediately, no decel ramp   
 * axis = 0 stop azimuth
 * axis = 1 stop elevation
 * any other number = stop both
 */
void fastStop(int axis) {
  if (axis == 0)
  {
    stepperX.stop();
    stepperX.disableOutputs();
    stoppingXSlew = false;
    slewingX = false;
    azMovingCW = false;
    azMovingCCW = false;
  } 
  else if (axis == 1)
  {
    stepperY.stop();
    stepperY.disableOutputs();
    stoppingYSlew = false;
    slewingY = false;
    elMovingUp = false;
    elMovingDown = false;
  }
  else
  {
    stepperX.disableOutputs();
    stepperY.disableOutputs();
    stoppingXSlew = false;
    slewingX = false;
    stoppingYSlew = false;
    slewingY = false;
    azMovingCW = false;
    azMovingCCW = false;
    elMovingUp = false;
    elMovingDown = false;   
  }
  
}

int setAccel(int axis, int accel)  {
  if (axis == 0)        // azimuth
    stepperX.setAcceleration((float)accel);
  else if (axis == 1)   // elevation
    stepperY.setAcceleration((float)accel);
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


// *********************** END STEPPER MOTION COMMANDS *******************************

// *********************************** BEGIN i2C *************************************
void receiveEvent(int howMany) {
    
  int numOfBytes = Wire.available();
  
  byte b = Wire.read();  //cmd

  //display message received, as char
  for(int i=0; i<numOfBytes-1; i++){
    char data = Wire.read();
    rawCommand.concat(data);
    //Serial.print(data);  
  }

  command = parseCommand(rawCommand);
  param = parseParameter(rawCommand);
  rawCommand = "";

  Serial.print("command: ");
  Serial.println(command);
  
  Serial.print("param: ");
  Serial.println(param);

  Serial.println();

  // Process the i2c command from master RPi
  processCommand();
}

void requestEvent() {
 
  // Setup byte variable in the correct size
  byte response[ANSWERSIZE];
  
  // Format answer as array
  for (byte i=0;i<answer.length();i++) {
    response[i] = (byte)answer.charAt(i);
  }
  
  // Send response back to Master
  Wire.write(response,sizeof(response));

  //Serial.print("answer: ");
  //Serial.println(answer);

}

// *********************************** END i2C *************************************

// Called from i2c receive event
void processCommand(){
  switch(command) {
    case 1:     // azimuth relative move
      relativeMove(0, param);
      command = 0;
      param = 0;
      answer = "azRelMv\n";
      break;
    case 2:     // elevation relative move
      relativeMove(1, param);
      command = 0;
      param = 0;
      answer = "elRelMv\n";
      break;
    case 3:     // stop all with decel
      stop(0);
      stop (1);
      answer = "stopAll\n";
      break;
    case 4:     // stop azimuth with decel
      stop(0);  
      answer = "stopAz_\n";
      break;
    case 5:     // stop elevation with decel
      stop(1);
      answer = "stopEl_\n";
      break;
    case 6:     // stop azimuith immediate
      fastStop(0);
      answer = "fStopAz\n";
      break;
    case 7:     // stop elevation immediate
      fastStop(1);
      answer = "fStopEl\n";
      break;
    case 8:     // azimuth CW limit
      answer = "_______" + String(getLimit(0,0));   
      break;
    case 9:     // azimuth CCW limit
      answer = "_______" + String(getLimit(0,1));   
      break;
    case 10:    // elevation UP limit
      answer = "_______" + String(getLimit(1,0));   
      break;
    case 11:    // elevation DOWN limit
      answer = "_______" + String(getLimit(1,1));
    case 12:    // set azimuth speed
      setSpeed(0, param);
      command = 0;
      param = 0;
      answer = "azSpeed\n";
      break;
    case 13:    // set elevation speed
      setSpeed(1, param);
      command = 0;
      param = 0;
      answer = "elSpeed\n";
      break;
    case 14:    // set azimuth accel
      setAccel(0, param);
      command = 0;
      param = 0;
      answer = "azAccel\n";
      break;
    case 15:    // set elevation accel
      setAccel(1, param);
      command = 0;
      param = 0;
      answer = "elAccel\n";
      break;
    case 16:    // set azimuth max speed
      setMaxSpeed(0, param);
      command = 0;
      param = 0;
      answer = "azMxSpd\n";
      break;
    case 17:    // set elevation max speed
      setMaxSpeed(1, param);
      command = 0;
      param = 0;
      answer = "elMxSpd\n";
      break;
    case 18:    // get current stepper position
      memset(str, '\0', sizeof(str));
      ltoa(getCurPosition(param), str, 10);
      answer = str;
      //Serial.print("answer: ");
      //Serial.println(answer);
      command = 0;
      param = 0;
      break;
    case 19:    // is stepper running?
      if (param == 0) {           // azimuth
        if (stepperX.isRunning()) {
          answer = "1";
        } else {
          answer = "0";
        } 
      } else if (param == 1)  {   // elevation
        if (stepperY.isRunning()) {
          answer = "1";
        } else {
          answer = "0";
        }
      }
      command = 0;
      param = 0;
      break;
    default:
      command = 0;
      param = 0;
      break;
  }
  
}


void setup() {
 
  // Setup Serial Monitor 
  Serial.begin(9600);


  Wire.begin(SLAVE_ADDRESS);

  // Read the data from RPi
  Wire.onReceive(receiveEvent);
  // Write data to RPi
  Wire.onRequest(requestEvent);

  // Stepper and limits setup
  
  // Hard limits
  pinMode(azLimitCW, INPUT_PULLUP);
  pinMode(azLimitCCW, INPUT_PULLUP);
  pinMode(elLimitUp, INPUT_PULLUP);
  pinMode(elLimitDown, INPUT_PULLUP);
  
  // Setup stepper enable pins
  stepperX.setEnablePin(7);   // using an enable pin
  stepperY.setEnablePin(7);
}

void loop() {


  // ******************************* Motion ***********************************
  
  // Relative move - az
  if (movingX && (abs(stepperX.distanceToGo())) > 0) {
    stepperX.run();
    stepperXRun = true;
  } else  {
    movingX = false;
    stepperXRun = false;
    //stepperX.stop();
  }
  
  // Relative move - el
  if (movingY && (abs(stepperY.distanceToGo())) > 0) {
    stepperY.run();
    stepperYRun = true;
  } else  {
    movingY = false;
    stepperYRun = false;
    //stepperY.stop();
  }

  // ******************************* Motion ***********************************

  // ************************* Check hard limits ******************************
  if ((getLimit(0,0) == 0) && azMovingCW) {           // az max CW
    fastStop(0);
    azCWLimit = true;
  } else if ((getLimit(0,1) == 0) && azMovingCCW) {   // az max CCW
    fastStop(0);
    azCCWLimit = true;
  } else {                                      // reset flags
    azCWLimit = false;
    azCCWLimit = false;
  }

  // Check hard limits
  if ((getLimit(1,0) == 0) && elMovingUp) {           // el max up
    fastStop(1);
    elUpLimit = true;
  } else if ((getLimit(1,1) == 0) && elMovingDown) {  // el max down
    fastStop(1);
    elDownLimit = true;
  } else {                                      // reset flags
    elUpLimit = false;
    elDownLimit = false;
  }

  // ************************* Check hard limits ******************************
 
}
