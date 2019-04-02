// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield    evshield(0x34,0x36);
CurrentMeter im (0x28);

void setup() {
  // Start Serial for output
  Serial.begin(115200);
  
  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  im.init(&evshield, BAS1);

  Serial.println("Setup done");
  Serial.println("Press the touch sensor to see changes in the values");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop() {
  char aa[80];        
  char str[256];         //sets length of character string
  int  acurr;            //declares "acurr" variable 
  int  rcurr;            //declares "rcurr" variable 
  int  refI;             //declares "refI" variable
  
/**  Displays Firmeware Version of sensor
*/
  strcpy(aa, im.getFirmwareVersion() );
  sprintf (str, "IMeter: getFirmwareVersion: %s", aa);
  Serial.println(str);
/**  Displays Device ID of sensor
 */
  strcpy(aa, im.getDeviceID() );
  sprintf (str, "IMeter: DeviceID: %s", aa);
  Serial.println(str);
/**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, im.getVendorID() );
  sprintf (str, "IMeter: VendorID: %s", aa);
  Serial.println(str);
/**  Displays Absolute Current value
 */
  acurr = im.getACurrent();
  sprintf (str, "IMeter: Absolute Current:   %d", acurr);
  Serial.println(str);
/**  Displays Relative Current value
 */  
  rcurr = im.getRCurrent();
  sprintf (str, "IMeter: Relative Current:   %d",  rcurr);
  Serial.println(str);
/**  Displays Reference Current value
 */    
  refI = im.getReference();
  sprintf (str, "IMeter: Reference Current:  %d",  refI);
  Serial.println(str);
  
  Serial.println( "-------------" );
  delay (1500);
}
