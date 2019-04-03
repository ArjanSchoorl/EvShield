// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
EV3Touch Touch;

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  Touch.init(&evshield, BAS1);

  Serial.println("Setup done");
  Serial.println("Press the touch sensor to see changes in the values");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  // Create variable(s)
  int val;
  int count;

  // Get the values
  val = Touch.isPressed();
  count = Touch.getBumpCount();

  // Print the sensor values
  Serial.print("Touched: ");
  Serial.println(val);
  Serial.print("Count: ");
  Serial.println(count);
  delay(1000);
}
