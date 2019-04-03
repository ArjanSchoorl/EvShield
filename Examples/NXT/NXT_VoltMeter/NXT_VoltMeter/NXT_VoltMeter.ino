// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
VoltMeter vm(0x26);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  vm.init(&evshield, BAS1);

  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  int avolt;     //declares "avolt" variable
  int rvolt;     //declares "rvolt" variable
  int refV;      //declares "refV" variable

  /**  Displays Firmeware Version of sensor
*/
  strcpy(aa, vm.getFirmwareVersion());
  sprintf(str, "VMeter: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, vm.getDeviceID());
  sprintf(str, "VMeter: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, vm.getVendorID());
  sprintf(str, "VMeter: VendorID: %s", aa);
  Serial.println(str);
  /**  Displays Absolute Voltage value
 */
  avolt = vm.getAVoltage();
  sprintf(str, "VMeter: Absolute Volts:     %d", avolt);
  Serial.println(str);
  /**  Displays Relative Voltage value
 */
  rvolt = vm.getRVoltage();
  sprintf(str, "VMeter: Relative Volts:     %d", rvolt);
  Serial.println(str);
  /**  Displays Reference Voltage value
 */
  refV = vm.getReference();
  sprintf(str, "VMeter: Reference Voltage:  %d", refV);
  Serial.println(str);

  Serial.println("-------------");
  delay(1500);
}