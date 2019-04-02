// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield    evshield(0x34,0x36);
NXTColor    Color;

void setup() {
  // Start Serial for output
  Serial.begin(115200);
  
  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  Color.init(&evshield, BAS1);
  Color.setType(Type_COLORNONE);
  
  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop() {
  // Color red
  Color.setType(Type_COLORRED);
  Serial.println("Red");
  delay(1000);

  // Color green
  Color.setType(Type_COLORGREEN); 
  Serial.println("Green");
  delay(1000);

  // Color blue
  Color.setType(Type_COLORBLUE);
  Serial.println("Blue");
  delay(1000);

  // Color white
  Color.setType(Type_COLORFULL);
  Serial.println("White");
  delay(1000);

  // None color
  Color.setType(Type_COLORNONE);
  Serial.println("Off");
  delay(1000);
}
