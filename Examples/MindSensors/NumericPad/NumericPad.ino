// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
NumericPad myNP(0xB3);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  myNP.init(&evshield, BAS1);
  myNP.InitializeKeypad();

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  // Create variable(s)
  int keys;
  keys = myNP.GetKeysPressed();

  // Loop Code
  Serial.print("Keys Pressed: ");
  Serial.println(keys);
  delay(1000);
}
