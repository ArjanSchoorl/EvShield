// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
NXTMMX mmx(0x06);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  mmx.init(&evshield, BAS1);
  byte brake = 1;
  mmx.stop(3, brake);

  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  byte motor1 = 1;
  byte motor2 = 2;
  byte motorboth = 3;
  byte speed = 100;
  byte forward = 1;
  byte reverse = 0;
  byte direction = forward;
  byte brake = 1;
  byte float1 = 0;
  byte isdone = 1;
  byte isnotdone = 0;
  long rotations = 5;
  long enc1;
  long enc2;
  int volt;

  /**  Displays Firmware Version of sensor
*/
  strcpy(aa, mmx.getFirmwareVersion());
  sprintf(str, "FirmwarVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, mmx.getDeviceID());
  sprintf(str, "Device ID: %s", aa); //Encoder value of motor 1
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, mmx.getVendorID());
  sprintf(str, "VendorID: %s", aa);
  Serial.println(str);
  /**  Displays Battery Voltage 
 */
  volt = mmx.getBatteryVoltage();
  sprintf(str, "Battery Voltage:   %d", volt);
  Serial.println(str);
  /**  Displays Encoder value of motor 1
 */
  enc1 = mmx.getEncoderPosition(motor1);
  sprintf(str, "Encoder 1:   %d", enc1);
  Serial.println(str);
  /**  Displays  Encoder value of motor 2
 */
  enc2 = mmx.getEncoderPosition(motor2);
  sprintf(str, "Encoder 2:  %d", enc2);
  Serial.println(str);

  mmx.runRotations(motor1, direction, speed, rotations, isdone, brake); //Run specified motor(s)
  delay(5000);                                                          //Wait for some time.

  Serial.println("-------------");
}