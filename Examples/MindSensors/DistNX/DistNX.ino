// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
DISTNx dist(0x02);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  dist.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  int distance;  //declares "distance" variable
  int volt;      //declares "volt" variable
  int type;      //declares "type" variable

  /**  Displays Firmware Version of sensor
*/
  strcpy(aa, dist.getFirmwareVersion());
  sprintf(str, "DistNx: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, dist.getDeviceID());
  sprintf(str, "DistNx: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, dist.getVendorID());
  sprintf(str, "DistNx: VendorID: %s", aa);
  Serial.println(str);
  /**  Displays distance value in mm
 */
  distance = dist.getDist();
  sprintf(str, "DistNx: Distance mm:   %d", distance);
  Serial.println(str);
  /**  Displays Voltage
 */
  volt = dist.getVolt();
  sprintf(str, "DistNx: Voltage:   %d", volt);
  Serial.println(str);
  /**  Displays Sensor Type
 */
  type = dist.getType();
  sprintf(str, "DistNx: SensorType:  %d", type);
  Serial.println(str);

  Serial.println("-------------");
  delay(1500);
}
