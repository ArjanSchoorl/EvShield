// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
NXTServo nservo(0xB0);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  nservo.init(&evshield, BAS1);

  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  byte test = 1;

  while (test == 1)
  {
    /**  Displays Firmware Version of sensor
*/
    strcpy(aa, nservo.getFirmwareVersion());
    sprintf(str, "Firmware Version: %s", aa);
    Serial.println(str);
    /**  Displays Device ID of sensor
 */
    strcpy(aa, nservo.getDeviceID());
    sprintf(str, "Device ID: %s", aa);
    Serial.println(str);
    /**  Gets and displays Vendor ID of sensor
 */
    strcpy(aa, nservo.getVendorID());
    sprintf(str, "VendorID: %s", aa);
    Serial.println(str);

    /**  Run servo 1 to to desired neutral position with desired neutral speed.
*/
    nservo.runServo(1, 150, 100);
    delay(1000);
    /**  Store values for servo(s) as neutral values.
*/
    nservo.storeInitial(1);
    delay(1000);
    test = 2;
    Serial.println("New value stored");
    Serial.println("-------------");
    delay(1500);
  }
}
