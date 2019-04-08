// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
PiLight pilyt(0x30);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  pilyt.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  color mycolor; //declares color variable
                 /**  Displays Firmeware Version of sensor
*/
  strcpy(aa, pilyt.getFirmwareVersion());
  sprintf(str, "pilight: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, pilyt.getDeviceID());
  sprintf(str, "pilight: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, pilyt.getVendorID());
  sprintf(str, "pilight: VendorID: %s", aa);
  Serial.println(str);
  delay(1000);
  /**  Sets and Displays the color values
 */
  pilyt.createPiLight(25, 0, 0); //red
  pilyt.readPiLight(mycolor);
  sprintf(str, "pilight: color      r:%d | g:%d | b:%d", mycolor.r, mycolor.g, mycolor.b);
  Serial.println(str);
  delay(1000);

  pilyt.createPiLight(0, 25, 0); //green
  pilyt.readPiLight(mycolor);
  sprintf(str, "pilight: color      r:%d | g:%d | b:%d", mycolor.r, mycolor.g, mycolor.b);
  Serial.println(str);
  delay(1000);

  pilyt.createPiLight(0, 0, 25); //blue
  pilyt.readPiLight(mycolor);
  sprintf(str, "pilight: color      r:%d | g:%d | b:%d", mycolor.r, mycolor.g, mycolor.b);
  Serial.println(str);
  Serial.println("-------------");
  delay(1000);
}
