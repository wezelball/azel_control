
#include <Encoder.h>
 
// Include Arduino Wire library for I2C
#include <Wire.h>
 
// Define Slave I2C Address
#define SLAVE_ADDR 9
 
// Define Slave answer size
#define ANSWERSIZE 8

// Instantiate the encoders
Encoder azEnc(2,4);
Encoder elEnc(3,5);

long position;    // encoder position
char str[8];      // string value of position
String answer;    // reply to master
int command;      // command from master
 
void setup() {
 
  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);
  
  // Function to run when data requested from master
  Wire.onRequest(requestEvent); 
  
  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
  
  // Setup Serial Monitor 
  Serial.begin(9600);
}
 
void receiveEvent() {
 
  // Read while data received
  while (0 < Wire.available()) {
    byte x = Wire.read();
    command = int(x);
  }
  
}
 
void requestEvent() {
 
  // Setup byte variable in the correct size
  byte response[ANSWERSIZE];
  
  // Format answer as array
  for (byte i=0;i<ANSWERSIZE;i++) {
    response[i] = (byte)answer.charAt(i);
  }
  
  // Send response back to Master
  Wire.write(response,sizeof(response));
}
 
void loop() {

  // What command did master send?
  if(command == 0) {
    position = azEnc.read();
    ltoa(position,str,10);
    answer = str;
  }
  else if (command == 1) {
    position = elEnc.read();
    ltoa(position,str,10);
    answer = str;
  } else {
    answer = "999999";
  }

  delay(100);
  //Serial.println(answer);
}
