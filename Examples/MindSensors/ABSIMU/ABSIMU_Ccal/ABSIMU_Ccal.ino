// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
AbsoluteIMU imu(0x22);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  imu.init(&evshield, BAS1);

  // Setup Code

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);

  Serial.println("Press  LEFT button to begin calibration");
  evshield.waitForButtonPress(BTN_LEFT);
  imu.beginCompassCalibration();

  Serial.println("Press Right button when calibration finished");
  evshield.waitForButtonPress(BTN_RIGHT);
  imu.endCompassCalibration();
}

void loop()
{
  char aa[80];
  char str[256]; //sets length of character string
  uint8_t result;
  char str2[20];         //sets length of character string 2
  char str3[20];         //sets length of character string 3
  gyro mygyro;           //declares mygyro variable
  cmps mycmps;           //declares mycmps variable
  accl myaccl;           //declares myaccl variable
  magnetic_field mymgnt; //declares mymgnt variable

  strcpy(aa, imu.getFirmwareVersion());
  sprintf(str, "imu: getFirmwareVersion: %s", aa);
  Serial.println(str);
  /**  Displays Device ID of sensor
 */
  strcpy(aa, imu.getDeviceID());
  sprintf(str, "imu: DeviceID: %s", aa);
  Serial.println(str);
  /**  Gets and displays Vendor ID of sensor
 */
  strcpy(aa, imu.getVendorID());
  sprintf(str, "imu: VendorID: %s", aa);
  Serial.println(str);
  /**  Displays Magnetometer reading
 */
  imu.readGyro(mygyro);
  sprintf(str, "imu: gyro      x:%d | y:%d | z:%d", mygyro.gx, mygyro.gy, mygyro.gz);
  Serial.println(str);
  /**  Displays Tilt reading
 */
  imu.readAccelerometer(myaccl);
  sprintf(str, "imu: tilt      x:%d | y:%d | z:%d", myaccl.tx, myaccl.ty, myaccl.tz);
  Serial.println(str);
  /**  Displays Accelerometer reading
 */
  sprintf(str, "imu: accl      x:%d | y:%d | z:%d", myaccl.ax, myaccl.ay, myaccl.az);
  Serial.println(str);
  /**  Displays Magnetic Field reading
 */
  imu.readMagneticField(mymgnt);
  sprintf(str, "imu: magfield  x:%d | y:%d | z:%d", mymgnt.mx, mymgnt.my, mymgnt.mz);
  Serial.println(str);
  /**  Displays Compass reading
 */
  imu.readCompass(mycmps);
  sprintf(str, "imu: compass:  %d", mycmps.heading);
  Serial.println(str);
  sprintf(str, "imu: error:  %d", mycmps.error);
  Serial.println(str);

  Serial.println("-------------");
  delay(1500);
}
