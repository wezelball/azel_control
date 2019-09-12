
// Include Arduino Wire library for I2C
#include <Wire.h>
 
// Define Slave I2C Address
#define SLAVE_ADDR 9
 
// Define Slave answer size
#define ANSWERSIZE 8

// Make current time global
// Fucking C/C++ !!!
int currentUTCTime[3];

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

void updateTime()  {
  
  // send request to receive data starting at register 0
  Wire.beginTransmission(0x68); // 0x68 is DS3231 device address
  Wire.write((byte)0); // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(0x68, 3); // request three bytes (seconds, minutes, hours)
 
  while(Wire.available())
  { 
    int seconds = Wire.read(); // get seconds
    int minutes = Wire.read(); // get minutes
    int hours = Wire.read();   // get hours
 
    seconds = (((seconds & 0b11110000)>>4)*10 + (seconds & 0b00001111)); // convert BCD to decimal
    minutes = (((minutes & 0b11110000)>>4)*10 + (minutes & 0b00001111)); // convert BCD to decimal
    hours = (((hours & 0b00100000)>>5)*20 + ((hours & 0b00010000)>>4)*10 + (hours & 0b00001111)); // convert BCD to decimal (assume 24 hour mode)

    currentUTCTime[2] = hours;
    currentUTCTime[1] = minutes;
    currentUTCTime[0] = seconds;    
  }
}

void setup() {
 
  // Initialize I2C communications as Master
  Wire.begin();
  
  // Setup serial monitor
  Serial.begin(9600);

  // clear /EOSC bit
  // Sometimes necessary to ensure that the clock
  // keeps running on just battery power. Once set,
  // it shouldn't need to be reset but it's a good
  // idea to make sure.
  Wire.beginTransmission(0x68); // address DS3231
  Wire.write(0x0E); // select register
  Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
  Wire.endTransmission();
}
 
void loop() {
  
  // Print to Serial Monitor
  Serial.print("Azimuth:\t");
  Serial.println(getEncoderPosition(0));
  Serial.print("Elevation:\t");
  Serial.println(getEncoderPosition(1));
  delay(1000);
  updateTime();   // update the current UTC time
  Serial.print("Time:\t");
  Serial.print(currentUTCTime[2]); Serial.print(":"); Serial.print(currentUTCTime[1]); Serial.print(":"); Serial.println(currentUTCTime[0]);
  
}
