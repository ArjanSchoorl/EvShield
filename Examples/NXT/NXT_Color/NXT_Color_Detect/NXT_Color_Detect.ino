// Add all required libraries
#include <EVShield.h>
#include <LegoLib.h>
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
  Color.setType(Type_COLORFULL);
  
  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop() {
  // Create variable(s)
  int val;

  // Get the values
  val = Color.readColor();

  // Print the sensor values
  Serial.print("Color sensor value: "); Serial.println(val);
  delay(1000);
}
