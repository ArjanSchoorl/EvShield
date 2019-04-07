// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
PFMate pfmate(0x48);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  pfmate.init(&evshield, BAS1);

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

  /**  Displays Firmeware Version of sensor
*/
  strcpy(aa, pfmate.getFirmwareVersion());
  sprintf(str, "pfmate: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, pfmate.getDeviceID());
  sprintf(str, "pfmate: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, pfmate.getVendorID());
  sprintf(str, "pfmate: VendorID: %s", aa);
  Serial.println(str);
  /**  Run motors at full speed
*/
  pfmate.controlMotor(PF_Channel_1, PF_Control_Both, PF_Operation_Forward, PF_Speed_Full);
  delay(2000);
  /**  Stop motors 
*/
  pfmate.controlMotor(PF_Channel_1, PF_Control_Both, PF_Operation_Brake, PF_Speed_Full);
  delay(2000);

  Serial.println("-------------");
  delay(1500);
}
