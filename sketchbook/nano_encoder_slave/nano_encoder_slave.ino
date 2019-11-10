/*
 * nano_encoder_slave.ino
 * Arduino Nano dedicated to reading az-el encoders
 * Dave Cohen
 * 
 */

// Include Encoder library
#include <Encoder.h>
// Include Arduino Wire library for I2C
#include <Wire.h>
// #include <SoftwareSerial.h>

// Define Slave answer size
#define ANSWERSIZE 16
#define SLAVE_ADDRESS 0x05

// Set up temporary software serial
// SoftwareSerial mySerial(11, 12); // RX, TX - debug

// Instantiate the encoders
Encoder azEnc(3,5);
Encoder elEnc(2,4);

long position;
char str[17];
String answer;
char az[8];
char el[8];
int command;

void setup() {
 
  // Setup Serial Monitor 
  Serial.begin(9600);

  // Software serial
  // mySerial.begin(9600);
  
  Wire.begin(SLAVE_ADDRESS);

  // Read the data from RPi
  Wire.onReceive(receiveEvent);
  // Write data to RPi
  Wire.onRequest(requestEvent);
}

void loop() {
  position = azEnc.read();
  //position = 1276481l;
  ltoa(position, az, 10);
  position = elEnc.read();
  //position = 327681l;
  ltoa(position,el,10);
  sprintf(str, "%s:%s",az,el);  
  //str = position;
  answer = str;

  /*
  delay(100);

  // Print what was reveived from Uno to hardware port
  while (mySerial.available()>0) {
    Serial.println(mySerial.readString());
  }
  */
}

void receiveEvent(int howMany) {
    
  int numOfBytes = Wire.available();
  Serial.print("Num bytes: ");
  Serial.println(numOfBytes);
  
  byte b = Wire.read();  //cmd
  Serial.print("cmd: ");
  Serial.println(b);

  //display message received, as char
  for(int i=0; i<numOfBytes-1; i++){
    char data = Wire.read();
    Serial.print(data);  
  }
  Serial.println();
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
