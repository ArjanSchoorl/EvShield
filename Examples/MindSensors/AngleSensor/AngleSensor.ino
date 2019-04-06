// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
AngleSensor angsens(0x30);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  angsens.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  uint8_t result;
  char str2[20]; //sets length of character string 2
  char str3[20]; //sets length of character string 3
  long angle;    //declares "angle" variable
  long myread;   //declares "myread" variable

  /**  Displays Firmeware Version of sensor
*/
  strcpy(aa, angsens.getFirmwareVersion());
  sprintf(str, "angsens: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, angsens.getDeviceID());
  sprintf(str, "angsens: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, angsens.getVendorID());
  sprintf(str, "angsens: VendorID: %s", aa);
  Serial.println(str);
  /**  Displays Angle
 */
  angle = angsens.getAngle();
  sprintf(str, "angsens: Angle:       %ld", angle);
  Serial.println(str);
  /**  Displays Raw Reading
 */
  myread = angsens.getRawReading();
  sprintf(str, "angsens: Raw Reading: %ld", myread);
  Serial.println(str);

  Serial.println("-------------");
  delay(1500);
}