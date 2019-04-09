// Add all required libraries
#include <EVShield.h>
#include <EVLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
NXTCam Cam;

int nblobs;
uint8_t color[8];
uint8_t left[8];
uint8_t top[8];
uint8_t bottom[8];
uint8_t right[8];

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  Cam.init(&evshield, BAS1);
  Cam.disableTracking();

  // Setup Cam for Object mode and sort by size.
  // Also let it begin tracking.
  Cam.selectObjectMode();
  Cam.sortSize();
  Cam.enableTracking();
  delay(1000);

  Serial.println("Setup done");

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  char aa[80];
  char str[256];
  uint8_t result;
  char str2[20];

  strcpy(aa, Cam.getFirmwareVersion());
  sprintf(str, "Cam: getFirmwareVersion: %s", aa);
  Serial.println(str);

  strcpy(aa, Cam.getDeviceID());
  sprintf(str, "Cam: DeviceID: %s", aa);
  Serial.println(str);

  strcpy(aa, Cam.getVendorID());
  sprintf(str, "Cam: VendorID: %s", aa);
  Serial.println(str);

  Serial.println("-------------");

  Cam.issueCommand('J'); // lock buffer
  delay(500);

  Cam.getBlobs(&nblobs, color, left, top, right, bottom);
  delay(500);

  Cam.issueCommand('K'); // unlock buffer
  Serial.println(nblobs);

  for (int i = 0; i < nblobs; i++)
  {
    sprintf(str, "color[%d]: %d  ", (i + 1), color[i]);
    Serial.print(str);
    str[0] = '\0';
    sprintf(str, "left[%d]: %d  ", (i + 1), left[i]);
    Serial.print(str);
    str[0] = '\0';
    sprintf(str, "top[%d]: %d  ", (i + 1), top[i]);
    Serial.print(str);
    str[0] = '\0';
    sprintf(str, "right[%d]: %d  ", (i + 1), right[i]);
    Serial.print(str);
    str[0] = '\0';
    sprintf(str, "bottom[%d]: %d  ", (i + 1), bottom[i]);
    Serial.println(str);
    str[0] = '\0';
  }
  Serial.println("-------------");
}