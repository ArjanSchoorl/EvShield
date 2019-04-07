// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);
LineLeader ll(0x02);

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);
  ll.init(&evshield, BAS1);
  // Setup Code

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
  byte test = 1;

  strcpy(aa, ll.getFirmwareVersion());
  sprintf(str, "Lineleader Firmware Version: %s", aa);
  Serial.println(str);

  strcpy(aa, ll.getDeviceID());
  sprintf(str, "Lineleader Device ID %s", aa);
  Serial.println(str);

  strcpy(aa, ll.getVendorID());
  sprintf(str, "Lineleader Vendor ID: %s", aa);
  Serial.println(str);

  while (test == 1)
  {
    Serial.println("Hold over White and press LEFT button to continue.");
    evshield.waitForButtonPress(BTN_LEFT);
    ll.calibrateWhite();
    delay(100);
    Serial.println("Hold over Black and press RIGHT button to continue.");
    evshield.waitForButtonPress(BTN_RIGHT);
    ll.calibrateBlack();
    delay(100);
    Serial.println("LineLeader Calibrated!");
    test = 2;
  }

  Serial.println("Should see values close to 100 when on white and close 0 when on black");

  result = ll.getResult();
  format_bin(result, str2);
  sprintf(str, "ll: sensor array: %s", str2);
  Serial.println(str);

  Serial.println("-------------");
  delay(1500);
}
