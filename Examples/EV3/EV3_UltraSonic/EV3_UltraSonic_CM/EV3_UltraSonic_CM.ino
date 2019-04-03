// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
EV3Ultrasonic US;

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  US.init(&evshield, BAS1);
  US.setMode(MODE_Sonar_CM);

  Serial.println("Setup done");
  Serial.println("Move object back and forth in front of ultrasonic sensor");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  // Create variable(s)
  int val;

  // Get the values
  val = US.getDist();

  // Print the sensor values
  Serial.print("Distance in cm: ");
  Serial.println(val);
  delay(1000);
}
