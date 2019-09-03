//Name this tab: CommandLine.h

#include <string.h>
#include <stdlib.h>

//this following macro is good for debugging, e.g.  print2("myVar= ", myVar);
#define print2(x,y) (Serial.print(x), Serial.println(y))


#define CR '\r'
#define LF '\n'
#define BS '\b'
#define NULLCHAR '\0'
#define SPACE ' '

#define COMMAND_BUFFER_LENGTH        25                        //length of serial buffer for incoming commands
char   CommandLine[COMMAND_BUFFER_LENGTH + 1];                 //Read commands into this buffer from Serial.  +1 in length for a termination char

const char *delimiters            = ", \n";                    //commands can be separated by return, space or comma

/*************************************************************************************************************
     Command Names Here
*/
const char *setSpeedToken         = "spd";                     
const char *moveRelCommandToken   = "mvr";
const char *setMaxSpeedToken      = "sms";
const char *runSpeedToken         = "rsp";
const char *stopCommandToken      = "stp";
const char *accelCommandToken     = "acl";
const char *getCurPosCommandToken = "gcp";
const char *setCurPosCommandToken = "scp";
const char *moveToCommandToken    = "mva";
const char *pingCommandToken      = "png";
const char *getImuCommandToken    = "gim";
const char *getAnalogPotToken     = "gap";
const char *getLimitCommandToken  = "lim";
