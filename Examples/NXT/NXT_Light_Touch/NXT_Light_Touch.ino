// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield    evshield(0x34,0x36);
NXTTouch    Touch;
NXTLight    Light;

void setup() {
  // Start Serial for output
  Serial.begin(115200);
  
  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  Touch.init(&evshield, BAS1);
  Light.init(&evshield, BAS2);

  Serial.println("Setup done");
  Serial.println("Press the touch sensor to see changes in the values");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop() {
  // Create variable(s)
  int touchPressed;
  int lightval;

  // Get the values
  touchPressed = Touch.isPressed();
  lightval = Light.readRaw();

  // Check if touchsensor is pressed
  if (touchPressed == true){
      Serial.println("Changing light sensor to reflected light mode");
      Light.setReflected();
  }
  else{
      Serial.println("Changing light sensor to ambient light mode");
      Light.setAmbient();
  }

  // Print the value of the light sensor
  Serial.print("Light value:"); Serial.println(lightval);
  delay(1000);
}
