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
  char str[256]; //declare character string length
  uint8_t *cal;  //declare "cal" pointer
  int i;         //declare integer "i"

  /** Display Firmware Version */
  strcpy(aa, lsa.getFirmwareVersion());
  sprintf(str, "LSArray: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /** Display Device ID */
  strcpy(aa, lsa.getDeviceID());
  sprintf(str, "LSArray: DeviceID: %s", aa);
  Serial.println(str);
  /** Display Vendor ID */
  strcpy(aa, lsa.getVendorID());
  sprintf(str, "LSArray: VendorID: %s", aa);
  Serial.println(str);
  /** Display 8 different light sensor values */
  cal = lsa.getCalibrated();
  for (i = 0; i < 8; i++)
  {
    sprintf(str, "LSArray: sensor array: %d = %d", i, cal[i]);
    Serial.println(str);
  }

  Serial.println("-------------");
  delay(1500);
}
