// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

#define shortrange false
#define longrange true
int inByte = 0;

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
SumoEyes seyes;

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  seyes.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  int seyesValue;
  SE_Zone obzone;
  char str[256];
  char aa[80];
  bool touchPressed;
  bool status;
  bool zone;

  Serial.println("----------------------------------------------");
  zone = shortrange; //set the range to short
  //zone = longrange;	//set the range to long
  if (zone)
  {
    status = seyes.setLongRange();
    delay(500);
  }
  else
  {
    status = seyes.setShortRange();
    delay(500);
  }
  seyesValue = seyes.readRaw();
  obzone = seyes.detectObstacleZone();

  sprintf(str, "Sumo Eyes (Range: %s) raw value: %d, obzone: %d (%s)",
          zone ? "Long" : "Short",
          seyesValue, obzone,
          seyes.OBZoneToString(obzone));
  Serial.println(str);

  Serial.println("----------------------------------------------");

  delay(1500);
}
