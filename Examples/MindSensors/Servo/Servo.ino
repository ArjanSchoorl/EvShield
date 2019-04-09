// Add all required libraries
#include <EVShield.h>
#include <MindsensorsLib.h>
#include <Wire.h>
#include <Servo.h>

// Create variables to use in this program
EVShield evshield(0x34, 0x36);

//Create objects for each servo pin
Servo myservo3;
Servo myservo5;
Servo myservo6;
Servo myservo9;
Servo myservo10;
Servo myservo11;

int pos = 0;

void setup()
{
  // Start Serial for output
  Serial.begin(115200);

  // Initialize the shield i2c interface
  // And initialize the sensor(s) and indicate where it is connected
  evshield.init(HardwareI2C);

  //Attach the servo objects to Arduino pins.
  myservo3.attach(3);
  myservo5.attach(5);
  myservo6.attach(6);
  myservo9.attach(9);
  myservo10.attach(10);
  myservo11.attach(11);

  // Wait until the Go button has been pressed
  Serial.println("Press Go button");
  evshield.waitForButtonPress(BTN_GO);
}

void loop()
{
  //Move all servos to maximum position
  pos = 255;
  myservo3.write(pos);  // tell servo to go to 'pos'
  myservo5.write(pos);  // tell servo to go to 'pos'
  myservo6.write(pos);  // tell servo to go to 'pos'
  myservo9.write(pos);  // tell servo to go to 'pos'
  myservo10.write(pos); // tell servo to go to 'pos'
  myservo11.write(pos); // tell servo to go to 'pos'

  delay(1000);

  //Move all servos to minimum position
  pos = 0;
  myservo3.write(pos);  // tell servo to go to 'pos'
  myservo5.write(pos);  // tell servo to go to 'pos'
  myservo6.write(pos);  // tell servo to go to 'pos'
  myservo9.write(pos);  // tell servo to go to 'pos'
  myservo10.write(pos); // tell servo to go to 'pos'
  myservo11.write(pos); // tell servo to go to 'pos'

  delay(1000);
}
