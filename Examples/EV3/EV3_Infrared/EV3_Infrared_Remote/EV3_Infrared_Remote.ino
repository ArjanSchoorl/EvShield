// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
EV3Infrared IR;

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  IR.init(&evshield, BAS1);
  IR.setMode(MODE_Infrared_Remote);

  Serial.println("Setup done");
  Serial.println("Set LEGO remote to specified channel, push button, and move side to side");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  // Create variable(s)
  int button;
  int channel = 1;

  // Get the values
  button = IR.readChannelButton(channel);

  // Print the sensor values
  Serial.print("Button: ");
  Serial.println(button);
  delay(1000);
}
