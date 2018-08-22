// EVShield.cpp

// Initial version: 2010-06-07 by Clinton Blackmore
// Large ports of the code is ported from the NXC library for the device,
// written by Deepak Patil.
// 12/18/2014  Nitin Patil --  modified to work with EVshield   
// Feb 2017  Seth Tenembaum -- modified to work with PiStorms
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "EVShield.h"
#include "Wire.h"

#if defined(ESP8266)
  extern "C" {
    #include "user_interface.h" /* for os_timer on ESP2866 */
  }
  static void pingEV(void *pArg);
  os_timer_t pingEVtimer;
#else
  #if defined(ARDUINO_ARC32_TOOLS)
    #include "CurieTimerOne.h"
  #else
    #include "MsTimer2.h"
  #endif
  static void pingEV();
#endif


#if defined(__AVR__)
	static void callbackLED();
#elif defined(__PIC32MX__)
	uint32_t callbackLED(uint32_t);
#endif

byte initCounter = 0;
bool btnState_go, btnState_left, btnState_right;
uint8_t redLED, redLED_cp;
uint8_t greenLED, greenLED_cp;
uint8_t blueLED, blueLED_cp;

bool toggle2 = 0;

bool format_bin(uint8_t i, char *s)
{
  int j;
  int b = 0x80;
 
  s[0] = '\0';
  for ( j = 0; j < 8; j++) {
    if ( i&b ) {
      strcat(s, "1");
    } else {
      strcat(s, "0");
    }
    b = b>>1;
  }
}


EVShield::EVShield(uint8_t i2c_address_a, uint8_t i2c_address_b)
{
  if ( i2c_address_a != Bank_A) bank_a.setAddress(i2c_address_a);
  if ( i2c_address_b != Bank_B) bank_b.setAddress(i2c_address_b);
}

void EVShield::init(Protocols protocol)
{
    while (initCounter < 5){
    //Serial.println(initCounter);
    I2CTimer();
	initProtocols(protocol);
   }
  
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  bank_a.writeByte(S1_MODE, Type_NONE); // set BAS1 type to none so it doesn't interfere with the following i2c communicaiton
  bank_a.writeByte(COMMAND, PS_TS_LOAD); // copy from permanent memory to temporary memory
  
  delay(2); // normally it only takes 2 milliseconds or so to load the values
  unsigned long timeout = millis() + 1000; // wait for up to a second
  while (bank_a.readByte(PS_TS_CALIBRATION_DATA_READY) != 1) // wait for ready byte
  {
    delay(10);
    if (millis() > timeout)
    {
      Serial.println("Failed to load touchscreen calibration values.");
      useOldTouchscreen = true;
      break;
    }
  }
  
  if (!useOldTouchscreen) {
    x1 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x00);
    y1 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x02);
    x2 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x04);
    y2 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x06);
    x3 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x08);
    y3 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x0A);
    x4 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x0C);
    y4 = bank_a.readInteger(PS_TS_CALIBRATION_DATA + 0x0E);
  }
  #endif
}

void EVShield::initProtocols(Protocols protocol)
{
  m_protocol = protocol;
  if (!m_protocol ) {
    bank_a._i2c_buffer = bank_a._buffer;
    bank_b._i2c_buffer = bank_b._buffer;
  } else {
    bank_a._i2c_buffer = bank_a._so_buffer;
    bank_b._i2c_buffer = bank_b._so_buffer;
  }
  bank_a.init((void *) this, (BankPort)-1);
  bank_b.init((void *) this, (BankPort)-1);

// ensure firmware compatibility.
// assuming that both banks are identical,
// it's adequate to check on one of the banks
  char v[10];
  char d[10];
  char str[80];
  
  strcpy(d, bank_a.getDeviceID());  
  strcpy(v, bank_a.getFirmwareVersion());
  if ( ( strcmp(d, "PiStorms") == 0 && (strcmp(v, "V1.09") >= 0 )) ||
       ( strcmp(d, "EVShld")   == 0 && (strcmp(v, "V1.09") >= 0)) )
  {
    // firmware is ok for this library
    initCounter = 6;
  } else {
    ++initCounter;
    if (initCounter == 5){
      if (strcmp(d, "PiStorms") == 0 || strcmp(d, "EVShld") == 0) {
        sprintf (str,"ERROR: Version mismatch. Reported Device: %s, Version: %s", d, v);
        Serial.println(str);
        Serial.println("V1.09 or later expected");
        sprintf (str,"Please upgrade your %s Firmware", strcmp(d, "EVShld") == 0 ? "EVShield" : "PiStorms");
        Serial.println(str);
      } else {
        Serial.println("ERROR: Unsupported device.");
        Serial.println("EVShield or PiStorms expected.");
        Serial.println("Please ensure your device is properly connected and functioning properly.");
        Serial.println("If you are trying to use an NXShield, please use the NXShield library from sourceforge.net/projects/nxshield");
      }
      pinMode(13, OUTPUT);
      while (true) { // stop here with red blinking light.
        ledSetRGB(100, 0, 0);
        digitalWrite(13, HIGH);
        delay(500);
        ledSetRGB(0, 0, 0);
        digitalWrite(13, LOW);
        delay(500);
      }
    }
    delay(100);
  }
  // end of firmware compatibility check
}

void EVShield::I2CTimer()
{
#if defined(ESP8266)
  os_timer_setfn(&pingEVtimer, pingEV, NULL);
  os_timer_arm(&pingEVtimer, 300, true); // 300ms period, true to make it repeat;
#elif defined(ARDUINO_ARC32_TOOLS)
  CurieTimerOne.start(300000, pingEV); // in microseconds
#else
  //TCNT2  = 0; 
  MsTimer2::set(300, pingEV); // 300ms period
  MsTimer2::start(); 
#endif
}

void EVShield::initLEDTimers()
{
  #if defined(__AVR__)

	  MsTimer2::set(3, callbackLED);
	  MsTimer2::start();
#elif defined(__PIC32MX__)
	  attachCoreTimerService(callbackLED);
#endif
}

EVShieldBankB::EVShieldBankB(uint8_t i2c_address)
: EVShieldBank(i2c_address)
{

}

EVShieldBank::EVShieldBank(uint8_t i2c_address)
: EVShieldI2C(i2c_address)
{

}
/*
void EVShield::I2CTimer(){
  //set timer2 interrupt at 64kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 64khz increments or 300ms
  OCR2A = 74;// = (16*10^6) / (64000*3.3333) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 64 prescaler
  TCCR2B |= (1 << CS22);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);  
}
*/
// provided for backword compatibility with nxshield programs.
int EVShieldBank::nxshieldGetBatteryVoltage()
{
  return evshieldGetBatteryVoltage();
}
// Voltage value returned in milli-volts.
int EVShieldBank::evshieldGetBatteryVoltage()
{
  int v;
  int factor = 40;
  v = readByte(VOLTAGE);
  return (v * factor);
}

uint8_t EVShieldBank::EVShieldIssueCommand(char command)
{
  return writeByte(COMMAND, (uint8_t)command);
}

// Set/get the encoder target for the motor (ie. a position to go
// to and stop at)
bool EVShieldBank::motorSetEncoderTarget(Motor which_motor, long target)
{
  uint8_t reg = (which_motor == Motor_1) ? SETPT_M1 : SETPT_M2;
  return writeLong(reg, target);
}

long EVShieldBank::motorGetEncoderTarget(Motor which_motor)
{
  uint8_t reg = (which_motor == Motor_1) ? SETPT_M1 : SETPT_M2;
  return (long)readLong(reg);
}

// Set/get the speed of the motor
// (I believe this is in the range [-100, +100])
bool EVShieldBank::motorSetSpeed(Motor which_motor, int speed)
{
  uint8_t reg = (which_motor == Motor_1) ? SPEED_M1 : SPEED_M2;
  return writeByte(reg, (uint8_t)(int8_t)speed);
}
int8_t EVShieldBank::motorGetSpeed(Motor which_motor)
{
  uint8_t reg = (which_motor == Motor_1) ? SPEED_M1 : SPEED_M2;
  return (int8_t)readByte(reg);
}

// This is the time, in seconds, for the motor to run
bool EVShieldBank::motorSetTimeToRun(Motor which_motor, int seconds)
{
  uint8_t reg = (which_motor == Motor_1) ? TIME_M1 : TIME_M2; 
  return writeByte(reg, seconds);
}
uint8_t EVShieldBank::motorGetTimeToRun(Motor which_motor)
{
  uint8_t reg = (which_motor == Motor_1) ? TIME_M1 : TIME_M2; 
  return readByte(reg);
}

// Command Register 'B' is currently unused, but reserved for future expansion
// If you set it, you must set it to zero.
bool EVShieldBank::motorSetCommandRegB(Motor which_motor, uint8_t value)
{
  uint8_t reg = (which_motor == Motor_1) ? CMD_B_M1 : CMD_B_M2; 
  return writeByte(reg, value);
}
uint8_t EVShieldBank::motorGetCommandRegB(Motor which_motor)
{
  uint8_t reg = (which_motor == Motor_1) ? CMD_B_M1 : CMD_B_M2; 
  return readByte(reg);
}

// See User's Guide for what command register A does
bool EVShieldBank::motorSetCommandRegA(Motor which_motor, uint8_t value)
{
  uint8_t reg = (which_motor == Motor_1) ? CMD_A_M1 : CMD_A_M2; 
  return writeByte(reg, value);
}
uint8_t EVShieldBank::motorGetCommandRegA(Motor which_motor)
{
  uint8_t reg = (which_motor == Motor_1) ? CMD_A_M1 : CMD_A_M2; 
  return readByte(reg);
}

// Get the current encoder position
int32_t EVShieldBank::motorGetEncoderPosition(Motor which_motor)
{
  uint8_t location = (which_motor == Motor_1) ? POSITION_M1 : POSITION_M2;
  return (int32_t)readLong(location);
}

// See User's Guide for documentation on the status byte
uint8_t EVShieldBank::motorGetStatusByte(Motor which_motor)
{
  uint8_t location = (which_motor == Motor_1) ? STATUS_M1 : STATUS_M2;
  return readByte(location);
}

// (I couldn't find an explanation for this in the User's Guide)
uint8_t EVShieldBank::motorGetTasksRunningByte(Motor which_motor)
{
  uint8_t location = (which_motor == Motor_1) ? TASKS_M1 : TASKS_M2;
  return readByte(location);
}

// Set the PID that controls how we stop as we approach the
// angle we're set to stop at
bool EVShieldBank::motorSetEncoderPID(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
  writeIntToBuffer(_i2c_buffer + 0, Kp);
  writeIntToBuffer(_i2c_buffer + 2, Ki);
  writeIntToBuffer(_i2c_buffer + 4, Kd);
  return writeRegisters(ENCODER_PID, 6);
}

// Sets the PID that controls how well that motor maintains its speed
bool EVShieldBank::motorSetSpeedPID(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
  writeIntToBuffer(_i2c_buffer + 0, Kp);
  writeIntToBuffer(_i2c_buffer + 2, Ki);
  writeIntToBuffer(_i2c_buffer + 4, Kd);
  return writeRegisters(SPEED_PID, 6);
}

bool EVShieldBank::centerLedSetRGB(uint8_t R, uint8_t G, uint8_t B)
{
  bool b;
  writeByteToBuffer(_i2c_buffer, R);
  writeByteToBuffer(_i2c_buffer+1,G);
  writeByteToBuffer(_i2c_buffer+2,B);
  b = writeRegisters(CENTER_RGB_LED, 3);
  delay(1);   // required to avoid subsequent i2c errors.
  return b;
}
// Set the RGBLED that shows RGB color

// TODO: it's noticed that i2c call made after ledSetRGB call fails.
// a delay is added to avoid the errors, but
// see why it fails and find a better solution.

bool EVShieldBank::ledSetRGB(uint8_t R, uint8_t G, uint8_t B)
{
  bool b;
  writeByteToBuffer(_i2c_buffer, R);
  writeByteToBuffer(_i2c_buffer+1,G);
  writeByteToBuffer(_i2c_buffer+2,B);
  b = writeRegisters(RGB_LED, 3);
  delay(1);   // required to avoid subsequent i2c errors.
  return b;
}

// See user's guide for details.
bool EVShieldBank::motorSetPassCount(uint8_t pass_count)
{
  return writeByte(PASS_COUNT, pass_count);
}

// Sets tolerance which adjust accuracy while positioning.
// See user's guide for more details.
bool EVShieldBank::motorSetTolerance(uint8_t tolerance)
{
  return writeByte(TOLERANCE, tolerance);
}


// Special I2C commands

// Resets all encoder values and motor parameters.  Leaves PIDs untouched.
bool EVShieldBank::motorReset()
{
  return EVShieldIssueCommand('R');
}

// Tells the motors to start at the same time.
bool EVShieldBank::motorStartBothInSync()
{
  return EVShieldIssueCommand('S');
}

// Reset the encoder for motor 1 or motor 2
bool EVShieldBank::motorResetEncoder(Motor which_motor)
{
    char code;
    switch (which_motor) {
        case Motor_1:
            code = 'r';
            return EVShieldIssueCommand(code);
        case Motor_2:
            code = 's';
            return EVShieldIssueCommand(code);
        case Motor_Both:
            code = 'r';
            EVShieldIssueCommand(code);
            code = 's';
            return EVShieldIssueCommand(code);
        default:
            return -1;
            break;
    }
}


// This function sets the speed, the number of seconds, and
// the control (a.k.a. command register A)
bool EVShieldBank::motorSetSpeedTimeAndControl(
  Motor which_motors,  // Motor_ 1, 2, or Both
  int     speed,      // in range [-100, +100]
  uint8_t duration,    // in seconds
  uint8_t control)    // bit flags for control purposes
{
  if (which_motors == Motor_Both)
  {
    control &= ~CONTROL_GO;  // Clear the 'go right now' flag
    bool m1 = motorSetSpeedTimeAndControl(Motor_1, speed, duration, control);
    bool m2 = motorSetSpeedTimeAndControl(Motor_2, speed, duration, control);
    motorStartBothInSync();
    return m1 && m2;
  }

  _i2c_buffer[0] = (uint8_t)(int8_t)speed;
  _i2c_buffer[1] = duration;
  _i2c_buffer[2] = 0;      // command register B
  _i2c_buffer[3] = control;  // command register A

  uint8_t reg = (which_motors == Motor_1) ? SPEED_M1 : SPEED_M2;
  return writeRegisters(reg, 4);
}

void evshieldSetEncoderSpeedTimeAndControlInBuffer(
  uint8_t* buffer,  // pointer to the buffer
  long   encoder,  // encoder value
  int   speed,    // speed, in range [-100, +100]
  uint8_t duration,  // in seconds
  uint8_t control)  // control flags
{
  writeLongToBuffer(buffer + 0, (uint32_t)(int32_t)encoder);
  buffer[4] = (uint8_t)(int8_t)speed;
  buffer[5] = duration;
  buffer[6] = 0;      // command register B
  buffer[7] = control;  // command register A
}


// This function sets the speed, the number of seconds, and
// the control (a.k.a. command register A)
bool EVShieldBank::motorSetEncoderSpeedTimeAndControl(
  Motor which_motors,  // Motor_ 1, 2, or Both
  long    encoder,    // encoder/tachometer position
  int    speed,      // speed, in range [-100, +100]
  uint8_t duration,    // in seconds
  uint8_t control)    // control flags
{
  if (which_motors == Motor_Both)
  {
    // The motor control registers are back to back, and both can be written in one command
    control &= ~CONTROL_GO;  // Clear the 'go right now' flag
    evshieldSetEncoderSpeedTimeAndControlInBuffer(_i2c_buffer, encoder, speed, duration, control);
    evshieldSetEncoderSpeedTimeAndControlInBuffer(_i2c_buffer + 8, encoder, speed, duration, control);
    bool success = writeRegisters(SETPT_M1, 16);
    motorStartBothInSync();
    return success;
  }

  // Or, just issue the command for one motor
  evshieldSetEncoderSpeedTimeAndControlInBuffer(_i2c_buffer, encoder, speed, duration, control);
  uint8_t reg = (which_motors == Motor_1) ? SETPT_M1 : SETPT_M2;
  return writeRegisters(reg, 8);
}

// returns 0 when a motor has completed a timed move
uint8_t EVShieldBank::motorIsTimeDone(Motor which_motors)
{
  uint8_t s1, s2;
  if (which_motors == Motor_Both)
  {
    s1 = motorGetStatusByte(Motor_1);
    s2 = motorGetStatusByte(Motor_2);
    
    if ( (s1 & STATUS_TIME) == 0 && (s2 & STATUS_TIME) == 0 )
    {
      // if stall bit was on there was an error
      /*if ( (s1 & STATUS_STALL) != 0 || (s2 & STATUS_STALL) != 0 )
      {
        return STATUS_STALL;
      } else {*/
        return 0;
      //}
    }
  } else {
    s1 = motorGetStatusByte(which_motors);
    if ( (s1 & STATUS_TIME) == 0 ) {
      /*if ( (s1 & STATUS_STALL) != 0 )
      {
        return STATUS_STALL;
      } else {*/
        return 0;
      //}
    }
  }

}

// waited until a timed command finishes
uint8_t EVShieldBank::motorWaitUntilTimeDone(Motor which_motors)
{
  uint8_t s;
  delay(50);  // this delay is required for the status byte to be available for reading.
  s = motorIsTimeDone(which_motors);  // fixed.
  while (( s & STATUS_TIME ) != 0 ) {
    delay (50);
    s = motorIsTimeDone(which_motors);  // fixed.
  }
}

// True when a command based on using the motor encoder completes
uint8_t EVShieldBank::motorIsTachoDone(Motor which_motors)
{
  uint8_t s1, s2;
  if (which_motors == Motor_Both)
  {
    s1 = motorGetStatusByte(Motor_1);
    s2 = motorGetStatusByte(Motor_2);
    
    if ( (s1 & STATUS_TACHO) == 0 && (s2 & STATUS_TACHO) == 0 )
    {
      // if stall bit was on there was an error
      /*if ( (s1 & STATUS_STALL) != 0 || (s2 & STATUS_STALL) != 0 )
      {
        return STATUS_STALL;
      } else {*/
        return 0;
      //}
    }
  } else {
    s1 = motorGetStatusByte(which_motors);
    if ( (s1 & STATUS_TACHO) == 0 ) {
      /*if ( (s1 & STATUS_STALL) != 0 )
      {
        return STATUS_STALL;
      } else {*/
        return 0;
      //}
    }
  }
}

// waited until a turn-by-degrees command ends
uint8_t EVShieldBank::motorWaitUntilTachoDone(Motor which_motors)
{
  uint8_t s;
  delay(50);  // this delay is required for the status byte to be available for reading.
  s = motorIsTachoDone(which_motors);
  while (( s & STATUS_TACHO ) != 0 ) {
    delay (50);
    s = motorIsTachoDone(which_motors);
  }
}


// Utility functions for motor control

// Take a speed and direction and give just a speed
inline int calcFinalSpeed(int initialSpeed, Direction direction)
{
  if (direction == Direction_Forward)
    return initialSpeed;
  return -initialSpeed;
}

// Calculate the bits that control what happens when this action finishes
inline uint8_t calcNextActionBits(Next_Action next_action)
{
  if (next_action == Next_Action_Brake)
    return CONTROL_BRK;
  else if (next_action == Next_Action_BrakeHold)
    return CONTROL_BRK | CONTROL_ON;
}

void EVShieldBank::motorRun(
  Motor which_motors,      // Motor_ 1, 2, or Both
  Direction direction,        // Direction_ Forward or Reverse
  int   speed)          // in range [-100, +100]
{
  uint8_t ctrl = CONTROL_SPEED | CONTROL_GO;
  int sp = calcFinalSpeed(speed, direction);
  motorSetSpeedTimeAndControl(which_motors, sp, 0, ctrl);
}

// runs the motors for a given number of seconds
uint8_t EVShieldBank::motorRunSeconds(
    Motor which_motors,      // Motor_ 1, 2, or Both
    Direction direction,        // Direction_ Forward or Reverse
    int     speed,          // [-100, +100]
    uint8_t duration,        // in seconds
    Completion_Wait wait_for_completion,  // Completion_ Wait_For or Dont_Wait
    Next_Action next_action)      // Next_Action_ Brake, BrakeHold or Float
{
  uint8_t ctrl = CONTROL_SPEED | CONTROL_TIME | CONTROL_GO;
  ctrl |= calcNextActionBits(next_action);
  int sp = calcFinalSpeed(speed, direction);
  motorSetSpeedTimeAndControl(which_motors, sp, duration, ctrl);

  if (wait_for_completion == Completion_Wait_For)
  {
    return motorWaitUntilTimeDone(which_motors);
  }
}

// runs the motors until the tachometer reaches a certain position
uint8_t EVShieldBank::motorRunTachometer(
    Motor which_motors,      // Motor_ 1, 2, or Both
    Direction direction,        // Direction_ Forward or Reverse
    int   speed,          // [-100, +100]
    long    tachometer,        // in degrees
    Move relative,        // Move_ Relative or Absolute
    Completion_Wait wait_for_completion,  // Completion_ Wait_For or Dont_Wait
    Next_Action next_action)      // Next_Action_ Brake, BrakeHold or Float
{
  uint8_t ctrl = CONTROL_SPEED | CONTROL_TACHO | CONTROL_GO;
  ctrl |= calcNextActionBits(next_action);
  int final_speed = calcFinalSpeed(speed, direction);
  uint8_t s;

  // The tachometer can be absolute or relative.
  // If it is absolute, we ignore the direction parameter.
  long final_tach = tachometer;

  if (relative == Move_Relative)
  {
    ctrl |= CONTROL_RELATIVE;

    // a (relative) forward command is always a positive tachometer reading
    final_tach = abs(tachometer);
    if (final_speed < 0)
    {
      // and a (relative) reverse command is always negative
      final_tach = -final_tach;
    }
  }

  motorSetEncoderSpeedTimeAndControl(which_motors, final_tach, final_speed, 0, ctrl);

  if (wait_for_completion == Completion_Wait_For)
  {
    //delay(50);
    s = motorWaitUntilTachoDone(which_motors);
  }
  return s;
}

// Turns the motors the specified number of degrees
uint8_t EVShieldBank::motorRunDegrees(
    Motor which_motors,      // Motor_ 1, 2, or Both
    Direction direction,        // Direction_ Forward or Reverse
    int   speed,          // [-100, +100]
    long    degrees,        // in degrees
    Completion_Wait wait_for_completion,  // Completion_ Wait_For or Dont_Wait
    Next_Action next_action)      // Next_Action_ Brake, BrakeHold or Float
{
  return motorRunTachometer(which_motors, direction, speed, degrees,
      Move_Relative, wait_for_completion, next_action);
}

// runs the motor(s) the specified number of rotations
uint8_t EVShieldBank::motorRunRotations(
    Motor which_motors,      // Motor_ 1, 2, or Both
    Direction direction,        // Direction_ Forward or Reverse
    int   speed,          // [-100, +100]
    long    rotations,        // number of full rotations of the motor
    Completion_Wait wait_for_completion,  // Completion_ Wait_For or Dont_Wait
    Next_Action next_action)      // Next_Action_ Brake, BrakeHold or Float
{
  return motorRunTachometer(which_motors, direction, speed, 360 * rotations,
      Move_Relative, wait_for_completion, next_action);
}

// The stop command will only stop the motor(s) by making them float/coast
// or brake.  Even if you specify Next_Action_BrakeHold, the motor
// will only brake, not hold.
bool EVShieldBank::motorStop(Motor which_motors, Next_Action next_action)
{
  if (which_motors >= Motor_1 && which_motors <= Motor_Both)
  {
    // The magic variables become clear in the user's guide
    uint8_t base_code = (next_action != Next_Action_Float) ? 'A' - 1 : 'a' - 1;

    return EVShieldIssueCommand(base_code + which_motors);
  }

  setWriteErrorCode(5);  // bad parameters
  return false;
}

bool EVShieldBank::sensorSetType(uint8_t which_sensor, uint8_t sensor_type)
{
  /*
   * which_sensor can only be 1 or 2,
   * reject any other value
   */
  switch (which_sensor) {
    case 1:
      return writeByte(S1_MODE, sensor_type);
      break;
    case 2:
      return writeByte(S2_MODE, sensor_type);
      break;
    default:
      return false;
      break;
  }
  return false;
}

/* register locations
  unsigned char Sensor1_mode;    //0x8A
  unsigned char Sensor2_mode;        //0x8B
  unsigned int Sensor_1_reading;    //0x70
  unsigned int Sensor_2_reading;    //0x8E
*/
int EVShieldBank::sensorReadRaw(uint8_t which_sensor)
{
  /*
   * sensor can only be 1 or 2,
   * otherwise return -1;
   */
  switch (which_sensor) {
    case 1:
        return readInteger(S1_ANALOG);    
        break;
    case 2:
        return readInteger(S2_ANALOG);
        break;
    default:
      return -1;
      break;
  }
}

bool EVShieldBankB::sensorSetType(uint8_t which_sensor, uint8_t sensor_type)
{
  /*
   * which_sensor can only be 1 or 2,
   * reject any other value
    */
  
  
  switch (which_sensor) {
    case 1:
      return EVShieldBankB::writeByte(S1_MODE, sensor_type);
      break;
    case 2:
      return EVShieldBankB::writeByte(S2_MODE, sensor_type);
      break;
    default:
      return false;
      break;
    }
  return false;
}

int EVShieldBankB::sensorReadRaw(uint8_t which_sensor)
{
  int a;
  switch (which_sensor) {
    case 1:
      // sensor port 1 behaves same as BankA
      // so call parent function.
      return EVShieldBankB::readInteger(S1_ANALOG); 
      break;
      
    case 2:
      return EVShieldBankB::readInteger(S2_ANALOG); 
      break;
    default:
      return -1;
      break;
  }
}

#if defined(ESP8266)
void pingEV(void *pArg)
#else
void pingEV()
#endif
{
    #if defined(ARDUINO_ARC32_TOOLS) || defined(ESP8266) || defined(ARDUINO_AVR_NANO)
        Wire.beginTransmission(0x34);
        Wire.endTransmission();
    #else
        TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
        while ((TWCR & (1<<TWINT)) == 0);
        TWDR = 0x34;
        TWCR = (1<<TWINT)|(1<<TWEN);
        while ((TWCR & (1<<TWINT)) == 0);
        TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
        TCNT2  = 0;//initialize counter value to 0    
        /*
        if (toggle2)
        {
          digitalWrite(13, HIGH);
          toggle2 = 0;
        }
        else
        {
          digitalWrite(13, LOW);
          toggle2 = 1; 
        }
        */
    #endif
}

#if defined(__AVR__)

void callbackLED()
{
  static uint8_t index = 1;

  //pinMode(BTN_GO,INPUT);
  //btnState_go = !digitalRead(BTN_GO);
  //pinMode(BTN_GO,OUTPUT);

  pinMode(BTN_LEFT,INPUT);
  btnState_left = !digitalRead(BTN_LEFT);
  pinMode(BTN_LEFT,OUTPUT);

  pinMode(BTN_RIGHT,INPUT);
  btnState_right = !digitalRead(BTN_RIGHT);
  pinMode(BTN_RIGHT,OUTPUT);

  digitalWrite(LED_RED, !redLED_cp&0x01);
  digitalWrite(LED_GREEN, !greenLED_cp&0x01);
  digitalWrite(LED_BLUE, !blueLED_cp&0x01);

  if (index == 8){
    index = 1;
    redLED_cp   = redLED;
    greenLED_cp = greenLED;
    blueLED_cp  = blueLED;
  }
  else{
    redLED_cp    = redLED_cp   >>1;
    greenLED_cp  = greenLED_cp >>1;
    blueLED_cp   = blueLED_cp  >>1;
    index ++;
  }

}
#elif defined(__PIC32MX__)
uint32_t callbackLED(uint32_t currentTime)
{
  static uint8_t index = 1;

  //pinMode(BTN_GO,INPUT);
  //btnState_go = !digitalRead(BTN_GO);
  //pinMode(BTN_GO,OUTPUT);

  pinMode(BTN_LEFT,INPUT);
  btnState_left = !digitalRead(BTN_LEFT);
  pinMode(BTN_LEFT,OUTPUT);

  pinMode(BTN_RIGHT,INPUT);
  btnState_right = !digitalRead(BTN_RIGHT);
  pinMode(BTN_RIGHT,OUTPUT);

  digitalWrite(LED_RED, !redLED_cp&0x01);
  digitalWrite(LED_GREEN, !greenLED_cp&0x01);
  digitalWrite(LED_BLUE, !blueLED_cp&0x01);

  if (index == 8){
    index = 1;
    redLED_cp   = redLED;
    greenLED_cp = greenLED;
    blueLED_cp  = blueLED;
  }
  else{
    redLED_cp    = redLED_cp   >>1;
    greenLED_cp  = greenLED_cp >>1;
    blueLED_cp   = blueLED_cp  >>1;
    index ++;
  }
	return (currentTime + CORE_TICK_RATE*3);
}

#endif

bool EVShield::getButtonState(uint8_t btn) {
  #if !(defined(ESP8266) || defined(ARDUINO_AVR_NANO))
  uint8_t bVal;
  bVal = bank_a.readByte(BTN_PRESS);

  return (bVal == btn);
  #else
  return ( (btn == BTN_GO    && bank_a.readByte(BTN_PRESS) % 2)
        || (btn == BTN_LEFT  && getFunctionButton() == 1)
        || (btn == BTN_RIGHT && getFunctionButton() == 2)          );
  #endif
}

void EVShield::waitForButtonPress(uint8_t btn, uint8_t led_pattern) {
  while(!getButtonState(btn)){
      switch (led_pattern) {
        case 1:
          ledBreathingPattern();
          break;
        case 2:
          ledHeartBeatPattern();
          break;
        default:
          delay (300);
          break;
      }
  }
  if (led_pattern != 0) ledSetRGB(0,0,0);
}

void EVShield::ledBreathingPattern() {
    static int breathNow = 0;
    int i;

    if ( breathNow > 800 && breathNow < 6400 ) {
        // LED intensity rising
        i = breathNow/800;
        ledSetRGB(0, i, i);
        delayMicroseconds(150);
        if ( i == 8 ) delayMicroseconds(200);
    } else if (breathNow > 6400 && breathNow < 13400 ) {
        // LED intensity falling
        i = (14400-breathNow)/1000;
        ledSetRGB(0, i, i);
        delayMicroseconds(200);
        if ( i == 8 ) delayMicroseconds(200);
    } else {
        // LED intensity stable.
        ledSetRGB(0,1,1);
        delayMicroseconds(50);
    }
    breathNow ++;
}

void EVShield::ledSetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
  bank_a.ledSetRGB(red,green,blue);
  //delay(100);
  bank_b.ledSetRGB(red,green,blue);
  //delay(100);
}

void EVShield::ledHeartBeatPattern() {
  static int breathNow = 0;
  int i;

  if ( breathNow > 800 && breathNow < 6400 ) {
    // LED intensity rising
    i = breathNow/800;
    ledSetRGB(0, i, i);
    //delayMicroseconds(150);
    if ( i == 8 ) delayMicroseconds(200);
  } else if (breathNow > 6400 && breathNow < 13400 ) {
    // LED intensity falling
    i = (14400-breathNow)/1000;
    ledSetRGB(0, i, i);
    //delayMicroseconds(200);
    if ( i == 8 ) delayMicroseconds(200);
  } else {
    // LED intensity stable.
    ledSetRGB(0,1,1);
    delayMicroseconds(10);
  }
  breathNow ++;
}

uint16_t EVShield::RAW_X()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  return bank_a.readInteger(PS_TS_RAWX);
  #else
  return 0;
  #endif
}

uint16_t EVShield::RAW_Y()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  return bank_a.readInteger(PS_TS_RAWY);
  #else
  return 0;
  #endif
}

// helper function to getReading
double distanceToLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) // some point and two points forming the line
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  // multiply by 1.0 to avoid integer truncation, don't need parentheses because the multiplication operator has left-to-right associativity
  return 1.0 * abs( (y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1 ) / sqrt( pow((y2-y1),2) + pow((x2-x1),2) );
  #else
  return 0.0;
  #endif
}

void EVShield::getReading(uint16_t *retx, uint16_t *rety) // returnX, returnY to avoid shadowing local x, y
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  uint16_t x = RAW_X();
  uint16_t y = RAW_Y();

  if ( x < min(x1,min(x2,min(x3,x4))) \
    || x > max(x1,max(x2,max(x3,x4))) \
    || y < min(y1,min(y2,min(y3,y4))) \
    || y > max(y1,max(y2,max(y3,y4))) )
  {
    *retx = 0;
    *rety = 0;
    return;
  }
  
  // careful not to divide by 0
  if ( y2-y1 == 0 \
    || x4-x1 == 0 \
    || y3-y4 == 0 \
    || x3-x2 == 0 )
  {
    *retx = 0;
    *rety = 0;
    return;
  }
  
  // http://math.stackexchange.com/a/104595/363240
  double dU0 = distanceToLine(x, y, x1, y1, x2, y2) / (y2-y1) * 320;
  double dV0 = distanceToLine(x, y, x1, y1, x4, y4) / (x4-x1) * 240;

  double dU1 = distanceToLine(x, y, x4, y4, x3, y3) / (y3-y4) * 320;
  double dV1 = distanceToLine(x, y, x2, y2, x3, y3) / (x3-x2) * 240;
  
  // careful not to divide by 0
  if ( dU0+dU1 == 0 \
    || dV0+dV1 == 0 )
  {
    *retx = 0;
    *rety = 0;
    return;
  }
  
  *retx = 320 * dU0/(dU0+dU1);
  *rety = 240 * dV0/(dV0+dV1);
  #else
  return;
  #endif
}

#if !(defined(ESP8266) || defined(ARDUINO_AVR_NANO))
  #warning from EVShield: Touchscreen methods are only supported on PiStorms (getTouchscreenValues, TS_X, TS_Y, isTouched, checkButton, getFunctionButton)
#endif

void EVShield::getTouchscreenValues(uint16_t *x, uint16_t *y)
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  if (useOldTouchscreen) {
    *x = TS_X();
    *y = TS_Y();
    return;
  }
  
  const uint8_t tolerance = 5;
  
  uint16_t x1, y1;
  getReading(&x1, &y1);
  uint16_t x2, y2;
  getReading(&x2, &y2);
  
  if (abs(x2-x1) < tolerance and abs(y2-y1) < tolerance)
  {
    *x = x2;
    *y = y2;
  } else {
    *x = 0;
    *y = 0;
  }
  #else
  return;
  #endif
}

uint16_t EVShield::TS_X()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  if (useOldTouchscreen) {
    return 320-bank_a.readInteger(PS_TS_X);
  }
  
  uint16_t x, y;
  getTouchscreenValues(&x, &y);
  return x;
  #else
  return 0;
  #endif
}

uint16_t EVShield::TS_Y()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  if (useOldTouchscreen) {
    return bank_a.readInteger(PS_TS_Y);
  }
  uint16_t x, y;
  getTouchscreenValues(&x, &y);
  return y;
  #else
  return 0;
  #endif
}

bool EVShield::isTouched()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  uint16_t x, y;
  getTouchscreenValues(&x, &y);
  return !(x==0 && y==0);
  #else
  return false;
  #endif
}

bool EVShield::checkButton(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  uint16_t tsx, tsy; // touch screen x, touch screen y
  getTouchscreenValues(&tsx, &tsy);
  
  if (tsx==0 && tsy==0)
  {
    return false;
  }
  
  // 0,0 is top-left corner
  // if left of right edge, right of left edge, above bottom edge, and below top edge
  return tsx<=x+width && tsx>=x && tsy<=y+height && tsy>=y;
  #else
  return false;
  #endif
}

uint8_t EVShield::getFunctionButton()
{
  #if defined(ESP8266) || defined(ARDUINO_AVR_NANO)
  if (useOldTouchscreen) {
    uint8_t v = bank_a.readByte(BTN_PRESS);
    if (v % 2 == 1) v--; // subtract out GO button (1)
    if (v == 8)  return 1;
    if (v == 16) return 2;
    if (v == 24) return 3;
    if (v == 40) return 4;
    return 0;
  }
  
  
  uint16_t x = RAW_X();
  uint16_t xborder;
  
  if (x4 > x1)  { // lower values left
    xborder = max(x1, x2); // where the touchscreen ends and the software buttons begin
    if (!(x < xborder+200 && x > xborder-200))
      return 0;
  } else { // greater values left
    xborder = min(x1, x2);
    if (!(x > xborder-200 && x < xborder+200))
      return 0;
  }
  
  uint16_t y = RAW_Y(),
           ymin = min(y1, y2),
           ymax = max(y1, y2),
           yQuarter = (ymax-ymin)/4; // a quarter of the distance between the two y extremes
  
  if (y < ymin + 0 * yQuarter)
    return 0; // too low
  if (y < ymin + 1 * yQuarter)
    return 4;
  if (y < ymin + 2 * yQuarter)
    return 3;
  if (y < ymin + 3 * yQuarter)
    return 2;
  if (y < ymin + 4 * yQuarter)
    return 1;
  if (y >= ymin + 4 * yQuarter)
    return 0; // too high

  return 0; // some other weird error occured, execution should not reach this point
  #else
  return 0;
  #endif
}
