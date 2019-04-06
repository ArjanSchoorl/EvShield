// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
LightSensorArray lsa(0x14);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  lsa.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256];
  uint8_t result;
  char str2[20];
  byte test = 1;
  int cal;
  int i;
  cal = lsa.getCalibrated();

  strcpy(aa, lsa.getFirmwareVersion());
  sprintf(str, "Lineleader Firmware Version: %s", aa);
  Serial.println(str);

  strcpy(aa, lsa.getDeviceID());
  sprintf(str, "Lineleader Device ID %s", aa);
  Serial.println(str);

  strcpy(aa, lsa.getVendorID());
  sprintf(str, "Lineleader Vendor ID: %s", aa);
  Serial.println(str);

  while (test == 1)
  {
    Serial.println("Hold over White and press LEFT button to continue.");
    evshield.waitForButtonPress(BTN_LEFT);
    lsa.calibrateWhite();
    delay(100);
    Serial.println("Hold over Black and press RIGHT button to continue.");
    evshield.waitForButtonPress(BTN_RIGHT);
    lsa.calibrateBlack();
    delay(100);
    Serial.println("LightSensorArray Calibrated!");
    test = 2;
  }

  Serial.println("Should see values close to 100 when on white and close 0 when on black");

  /** Display 8 different light sensor values */
  for (i = 0; i < 8; i++)
  {
    //sprintf (str, "LSArray: sensor array: %d = %d", i, cal[i] );
    sprintf(str, "LSArray: sensor array: %d = %d", i);
    Serial.println(str);
  }

  Serial.println("-------------");
  delay(1500);
}
