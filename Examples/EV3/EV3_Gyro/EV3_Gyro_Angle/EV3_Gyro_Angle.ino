// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield    evshield(0x34,0x36);
EV3Gyro     Gyro;

void setup() {
  // Start Serial for output
  Serial.begin(115200);
  
  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  Gyro.init(&evshield, BAS1);
  Gyro.setMode(MODE_Gyro_Angle);
  
  Serial.println("Setup done");
  Serial.println("Turn your gyro to see change in value");
  
  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop() {
  // Create variable(s)
  int val;

  // Get the values
  val = Gyro.getAngle();

  // Print the sensor values
  Serial.print("Angle: "); Serial.println(val);
  delay(1000);
}
