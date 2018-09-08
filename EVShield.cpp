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

#if defined(ARDUINO_ARC32_TOOLS)
  #include "CurieTimerOne.h"
#else
  #include "MsTimer2.h"
#endif

extern "C" {
#if ( ARDUINO == 10608 )
#include "../../hardware/arduino/avr/libraries/Wire/src/utility/twi.h"
#elif ( ARDUINO == 10605 )
#include "../../hardware/arduino/avr/libraries/Wire/utility/twi.h"
#elif defined(ESP8266)
// previously included: ".../hardware/esp8266/2.3.0/cores/esp8266/twi.h"
#else
uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
#endif
}

byte initCounter = 0;
bool btnState_go, btnState_left, btnState_right;
uint8_t redLED, redLED_cp;
uint8_t greenLED, greenLED_cp;
uint8_t blueLED, blueLED_cp;

bool toggle2 = 0;

//EVShieldAGS Library
EVShieldAGS::EVShieldAGS()
{
  mp_shield = NULL;
}

EVShieldAGS::EVShieldAGS(EVShield * shield, BankPort bp)
{
  mp_shield = shield;
  m_bp = bp;
}

bool EVShieldAGS::setType(uint8_t type)
{
  if ( mp_shield == NULL) return false;
  switch (m_bp) {
    case BAS1:
      return mp_shield->bank_a.sensorSetType(S1, type);
      break;
    case BAS2:
      return mp_shield->bank_a.sensorSetType(S2, type);
      break;
    case BBS1:
      return mp_shield->bank_b.sensorSetType(S1, type);
      break;
    case BBS2:
      return mp_shield->bank_b.sensorSetType(S2, type);
      break;
  }
}

int EVShieldAGS::readRaw()
{
  if ( mp_shield == NULL) return -1;

  switch (m_bp) {
    case BAS1:
      return mp_shield->bank_a.sensorReadRaw(S1);
      break;
    case BAS2:
      return mp_shield->bank_a.sensorReadRaw(S2);
      break;
    case BBS1:
      return mp_shield->bank_b.sensorReadRaw(S1);
      break;
    case BBS2:
      return mp_shield->bank_b.sensorReadRaw(S2);
      break;
  }
}

bool EVShieldAGS::init(EVShield * shield, BankPort bp)
{
  mp_shield = shield;
  m_bp = bp;
  return true;
}

//EVShieldI2C Library
EVShieldI2C::EVShieldI2C(uint8_t i2c_address)
  : BaseI2CDevice(i2c_address), SoftI2cMaster(i2c_address)
{
}

uint8_t  EVShieldI2C::readByte  (uint8_t location)
{
  if (!m_protocol) return BaseI2CDevice::readByte( location );
  else             return SoftI2cMaster::readByte( location );
}


uint16_t EVShieldI2C::readInteger  (uint8_t location)
{
  if (!m_protocol) return BaseI2CDevice::readInteger( location );
  else             return SoftI2cMaster::readInteger( location );
}


uint32_t EVShieldI2C::readLong  (uint8_t location)
{
  if (!m_protocol) return BaseI2CDevice::readLong( location );
  else             return SoftI2cMaster::readLong( location );
}


uint8_t*  EVShieldI2C::readRegisters  (uint8_t  startRegister, uint8_t  bytes, uint8_t* buf)
{
  if (!m_protocol) return BaseI2CDevice::readRegisters(startRegister, bytes, buf);
	else             return SoftI2cMaster::readRegisters(startRegister, bytes, buf);
}


char*    EVShieldI2C::readString  (uint8_t  location, uint8_t  bytes_to_read,
            uint8_t* buffer, uint8_t  buffer_length)
{
  if (!m_protocol) return BaseI2CDevice::readString(location, bytes_to_read, buffer, buffer_length);
  else             return SoftI2cMaster::readString(location, bytes_to_read, buffer, buffer_length);
}


bool EVShieldI2C::writeRegisters  (uint8_t start_register, uint8_t bytes_to_write, uint8_t* buffer)
{
  if (!m_protocol) return BaseI2CDevice::writeRegisters(start_register, bytes_to_write, buffer);
  else             return SoftI2cMaster::writeRegisters(start_register, bytes_to_write, buffer);
}

bool EVShieldI2C::writeByte  (uint8_t location, uint8_t data)
{
    uint8_t dd[3];
    if (!m_protocol) {
        BaseI2CDevice::writeByte(location, data);
    } else {
        SoftI2cMaster::writeByte(location, data);
    }
    return true;
}

bool EVShieldI2C::writeInteger(uint8_t location, uint16_t data)
{
  if (!m_protocol) return BaseI2CDevice::writeInteger(location, data);
  else             return SoftI2cMaster::writeInteger(location, data);
}

bool EVShieldI2C::writeLong  (uint8_t location, uint32_t data)
{
  if (!m_protocol) return BaseI2CDevice::writeLong(location, data);
  else             return SoftI2cMaster::writeLong(location, data);
}

uint8_t EVShieldI2C::getErrorCode  ( )
{
  if (!m_protocol) return BaseI2CDevice::getWriteErrorCode();
  else             return SoftI2cMaster::getWriteErrorCode();
}

bool EVShieldI2C::checkAddress  ( )
{
  if (!m_protocol) return BaseI2CDevice::checkAddress();
  else             return SoftI2cMaster::checkAddress();
}

bool EVShieldI2C::setAddress  (uint8_t address)
{
    // regardless of protocol, set the address
  BaseI2CDevice::setAddress(address);
  SoftI2cMaster::setAddress(address);
	return true;
}


// READ INFORMATION OFF OF THE DEVICE
// returns a string with the current firmware version of the device
char* EVShieldI2C::getFirmwareVersion()
{
  return readString(0, 8);
}

// returns a string indicating the vendor of the device
char* EVShieldI2C::getVendorID()
{
  return readString(0x08, 8);
}

// returns a string indicating the device's ID
char* EVShieldI2C::getDeviceID()
{
  return readString(0x10, 8);
}
// returns a string indicating the features on this device
// some devices may return null.
char* EVShieldI2C::getFeatureSet()
{
  return readString(0x18, 8);
}

void EVShieldI2C::init(void * shield, BankPort bp)
{
    mp_shield = shield;
    // on all banks hardware as well as software protocols are possible.
    // so store the main shield's protocol value with us.
    // and initialize with appropriate function
    //
    // For BAS2, BBS1, BBS2 only software i2c is possible.
    m_protocol = ((EVShield *)shield)->m_protocol;
    switch (m_protocol) {
        case HardwareI2C:
            BaseI2CDevice::initProtocol ( );
            break;
        case SoftwareI2C:
            SoftI2cMaster::initProtocol ( ); // no arguments, ie use default h/w i2c pins: (A5,A4)
            break;
    }
    switch (bp) {
        case BAS1:
            ((EVShield *)shield)->bank_a.writeByte(S1_MODE,Type_I2C);
            break;
        case BAS2:
            ((EVShield *)shield)->bank_a.writeByte(S2_MODE,Type_I2C);
            break;
        case BBS1:
            ((EVShield *)shield)->bank_b.writeByte(S1_MODE,Type_I2C);
            break;
        case BBS2:
            ((EVShield *)shield)->bank_b.writeByte(S2_MODE,Type_I2C);
            break;
    }
}

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
  if (direction == Forward)
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

//EVShieldUART
EVShieldUART::EVShieldUART()
{
    mp_shield = NULL;
}

EVShieldUART::EVShieldUART(EVShield * shield, BankPort bp)
{
    mp_shield = shield;
    m_bp = bp;

    switch(m_bp) {
        case BAS1:
        case BBS1:
            m_offset = 0;
            break;
        case BAS2:
        case BBS2:
            m_offset = 52;
            break;
    }
}

bool EVShieldUART::setType(uint8_t type)
{
    if ( mp_shield == NULL) return false;
    switch (m_bp) {
        case BAS1:
            return mp_shield->bank_a.sensorSetType(S1, type);

        case BAS2:
            return mp_shield->bank_a.sensorSetType(S2, type);

        case BBS1:
            return mp_shield->bank_b.sensorSetType(S1, type);

        case BBS2:
            return mp_shield->bank_b.sensorSetType(S2, type);

    }
}

bool EVShieldUART::writeLocation(uint8_t loc, uint8_t data)
{
    if ( mp_shield == NULL) return false;

    switch (m_bp) {
        case BAS1:
        case BAS2:
            return mp_shield->bank_a.writeByte(loc, data);

        case BBS1:
        case BBS2:
            return mp_shield->bank_b.writeByte(loc, data);

    }
}

int16_t EVShieldUART::readLocationInt(uint8_t loc)
{
    if ( mp_shield == NULL) return -1;

    switch (m_bp) {
        case BAS1:
        case BAS2:
            return mp_shield->bank_a.readInteger(loc);

        case BBS1:
        case BBS2:
            return mp_shield->bank_b.readInteger(loc);

    }
}

uint8_t EVShieldUART::readLocationByte(uint8_t loc)
{
    if ( mp_shield == NULL) return -1;

    switch (m_bp) {
        case BAS1:
        case BAS2:
            return mp_shield->bank_a.readByte(loc);

        case BBS1:
        case BBS2:
            return mp_shield->bank_b.readByte(loc);

    }
}

bool EVShieldUART::init(EVShield * shield, BankPort bp)
{
    mp_shield = shield;
    m_bp = bp;
    switch(m_bp) {
        case BAS1:
        case BBS1:
            m_offset = 0;
            break;
        case BAS2:
        case BBS2:
            m_offset = 52;
            break;
    }
    return true;
}

uint8_t	EVShieldUART::getMode( )
{
    if ( mp_shield == NULL) return -1;
    switch (m_bp) {
        case BAS1:
        case BAS2:
            return mp_shield->bank_a.readByte(0x81+m_offset);
        case BBS1:
        case BBS2:
            return mp_shield->bank_b.readByte(0x81+m_offset);
    }
}


uint8_t	EVShieldUART::setMode(char newMode)
{
    if ( mp_shield == NULL) return -1;
    switch (m_bp) {
        case BAS1:
        case BAS2:
            return mp_shield->bank_a.writeByte(0x81+m_offset, (uint8_t) newMode);
        case BBS1:
        case BBS2:
            return mp_shield->bank_b.writeByte(0x81+m_offset, (uint8_t) newMode);
    }
}

bool	EVShieldUART::isDeviceReady()
{
    if ( mp_shield == NULL) return false;
    switch (m_bp) {
        case BAS1:
        case BAS2:
            return (mp_shield->bank_a.readByte(0x70+m_offset) == 1);
        case BBS1:
        case BBS2:
            return (mp_shield->bank_b.readByte(0x70+m_offset) == 1);
    }

}


bool	EVShieldUART::readAndPrint(uint8_t loc, uint8_t len)
{
    uint8_t result;
    Serial.print(" ");
    for (int i=loc; i<loc+len; i++) {
        Serial.print (readLocationByte(i), DEC); Serial.print(" ");
    }
    //Serial.println("");
}


uint16_t	EVShieldUART::readValue()
{
    uint16_t result;
    result = readLocationInt(0x83+m_offset);
    return (result);
}

// BaseI2CDevice
// Max I2C message length is 16 bytes.  
const int BUFFER_LEN = 16;  


// Initialize static variables
uint8_t* BaseI2CDevice::_buffer = 0;
bool BaseI2CDevice::b_initialized = false;


BaseI2CDevice::BaseI2CDevice(uint8_t i2c_address)
{
  // As I understand it, an I2C bus can address 127 different devices (a 7-bit quantity).
  // When used, the 7-bit quantity is shifted right one bit, and the last bit is clear
  // for a read operation and set for a write operation.  Arduino's Wire library expects
  // a 7-bit address, but most tech specs list the 8-bit address.  Hence, we drop
  // the least significant bit (and Wire.h shifts the address and sets the read/write
  // bit as appropriate.)
  b_initialized = false;
  _device_address = i2c_address >> 1;

  _buffer = (uint8_t*) calloc(BUFFER_LEN, sizeof(uint8_t));
}

void BaseI2CDevice::initProtocol()
{
  if ( b_initialized ) return;
  #if defined(ESP8266)
  Wire.begin(D2,D3);
  #else
  Wire.begin();
  #endif
  b_initialized = true;
}

// READING FUNCTIONS

// Reads registers of an I2C device.
// See the documentation for your device to know what a given register
// or register range indicates.
uint8_t* BaseI2CDevice::readRegisters(
  uint8_t  start_register,   // start of the register range
  uint8_t  bytes_to_read,   // number of bytes to read (max 16 for lego devices)
  uint8_t* buffer,      // (optional) user-supplied buffer
  uint8_t  buffer_length,    // (optional) length of user-supplied buffer
  bool     clear_buffer)    // should we zero out the buffer first? (optional)
{
  #if defined(ARDUINO_ARC32_TOOLS)
    CurieTimerOne.rdRstTickCount();
  #else
    MsTimer2::reset();
  #endif
  if (!buffer)
  {
    buffer = _buffer;
  }

  if (!buffer_length)
  {
    buffer_length = BUFFER_LEN;
  }

  bytes_to_read = min(bytes_to_read, buffer_length);  // avoid buffer overflow

  if (clear_buffer)
  {
    memset(buffer, 0, buffer_length);
  }

  // We write to the I2C device to tell it where we want to read from
  Wire.beginTransmission(_device_address);
#if defined(ARDUINO) && ARDUINO >= 100
    Wire.write(start_register);
#else
    Wire.send(start_register);
#endif
    //Wire.send(bytes_to_read);
    Wire.endTransmission();

    // Now we can read the data from the device
  Wire.requestFrom(_device_address, bytes_to_read);

    for (uint8_t index = 0; Wire.available(); ++index)
    {
#if defined(ARDUINO) && ARDUINO >= 100
      buffer[index] = Wire.read();
#else
      buffer[index] = Wire.receive();
#endif
    }

    _write_error_code = Wire.endTransmission();
        
  #if defined(ARDUINO_ARC32_TOOLS)
    CurieTimerOne.rdRstTickCount();
  #else
    MsTimer2::reset();
  #endif
  return buffer;
}

// Reads a byte from the given register on the I2C device.
uint8_t BaseI2CDevice::readByte(uint8_t location)
{
  readRegisters(location, 1);
  return _buffer[0];
}

// Reads two bytes from the given register pair on the I2C device.
int16_t BaseI2CDevice::readInteger(uint8_t location)
{
  readRegisters(location, 2);

  // I believe the data has the least significant byte first
  return readIntFromBuffer(_buffer);
}

// Reads four bytes from the given registers, starting at the specified location, on the I2C device.
uint32_t BaseI2CDevice::readLong(uint8_t location)
{
  readRegisters(location, 4);
  return readLongFromBuffer(_buffer);
}

// Reads a string.  Be certain that your buffer is large enough
// to hold the string and a trailing 'nul'!
char* BaseI2CDevice::readString(
    uint8_t  location,       // starting location of the string
    uint8_t  bytes_to_read,   // number of bytes to read
    uint8_t* buffer,      // optional user-supplied buffer
    uint8_t  buffer_length)    // length of user-supplied buffer)
{
  return (char *)readRegisters(location, bytes_to_read, buffer, buffer_length, true);
}


// WRITING FUNCTIONS

// Returns true if the write was successful.
// If not true, you may check the result by calling getWriteErrorCode.
bool BaseI2CDevice::writeRegisters(
  uint8_t  start_register,   // start of the register range
  uint8_t  bytes_to_write,   // number of bytes to write
  uint8_t* buffer)    // optional user-supplied buffer
{
  #if defined(ARDUINO_ARC32_TOOLS)
    CurieTimerOne.rdRstTickCount();
  #else
    MsTimer2::reset();
  #endif
  if (!buffer)
  {
    buffer = _buffer;
  }

  // We write to the I2C device to tell it where we want to read from and how many bytes
  Wire.beginTransmission(_device_address);
#if defined(ARDUINO) && ARDUINO >= 100
  Wire.write(start_register);
#else
  Wire.send(start_register);
#endif

  // Send the data
  for (uint8_t index = 0; index < bytes_to_write; ++index)
  {
#if defined(ARDUINO) && ARDUINO >= 100
    Wire.write(buffer[index]);
#else
    Wire.send(buffer[index]);
#endif
  }

  _write_error_code = Wire.endTransmission();

  #if defined(ARDUINO_ARC32_TOOLS)
    CurieTimerOne.rdRstTickCount();
  #else
    MsTimer2::reset();
  #endif
  return _write_error_code == 0;  // 0 indicates success
}

// Writes a byte to a given register of the I2C device
bool BaseI2CDevice::writeByte(uint8_t location, uint8_t data)
{
  return writeRegisters(location, 1, &data);
}

// Writes two bytes to a given register of the I2C device
bool BaseI2CDevice::writeInteger(uint8_t location, uint16_t data)
{
  writeIntToBuffer(_buffer, data);
  return writeRegisters(location, 2, _buffer);
}

// Writes four bytes to a given register of the I2C device
bool BaseI2CDevice::writeLong(uint8_t location, uint32_t data)
{
  writeLongToBuffer(_buffer, data);
  return writeRegisters(location, 4, _buffer);
}

// This is the status value returned from the last write command.
// A return value of zero indicates success.
// Non-zero results indicate failures.  From libraries/Wire/utility/twi.c, they are:
//          1 .. length to long for buffer
//          2 .. address send, NACK received
//          3 .. data send, NACK received
//          4 .. other twi error (lost bus arbitration, bus error, ..)
uint8_t BaseI2CDevice::getWriteErrorCode()
{
  return _write_error_code;
}

// READ SOME INFORMATION OFF OF THE DEVICE
// returns a string with the current firmware version of the device
char* BaseI2CDevice::getFirmwareVersion()
{
  return readString(0, 8);
}

// returns a string indicating the vendor of the device
char* BaseI2CDevice::getVendorID()
{
  return readString(0x08, 8);
}

// returns a string indicating the device's ID
char* BaseI2CDevice::getDeviceID()
{
  return readString(0x10, 8);
}

/** returns a string indicating the features on this device
 some devices may return null.
*/
char* BaseI2CDevice::getFeatureSet()
{
  return readString(0x18, 8);
}

// It is very unusual to do this
void BaseI2CDevice::setWriteErrorCode(uint8_t code)
{
  _write_error_code = code;
}


/**
 * checkAddress()
 * this function checks to see if there is 
 * any device at its specified address 
 */

bool BaseI2CDevice::checkAddress()
{
  uint8_t *txBuffer;
  int8_t x = 1;
#if defined(__PIC32MX__)
  x = twi_writeTo(_device_address, txBuffer, 0, 1) == 0;
#else
  #if (defined(ARDUINO) && ARDUINO <= 100) || defined(ESP8266)
    x = twi_writeTo(_device_address, txBuffer, 0, 1) == 0;
  #else
    x = twi_writeTo(_device_address, txBuffer, 0, 1, 1) == 0;
  #endif
#endif
  return (x != 0);
}


/**
 * setAddress(address)
 * this function set's the i2c address
 * for this instance to given address 
 * Note that, generally i2c address of a physical device does not change.
 * Use this function if there are multiple devices on your bus and you want to 
 * conserve processor memory from instantiating another class instance.
 */

bool BaseI2CDevice::setAddress(uint8_t i2c_address)
{
  _device_address = i2c_address >> 1;
  return true;
}

uint8_t BaseI2CDevice::getAddress()
{
  return _device_address;
}

//SoftI2cMaster
const int BUFF_LEN = 16;
//------------------------------------------------------------------------------
// WARNING don't change anything unless you verify the change with a scope
//------------------------------------------------------------------------------
// set device address to begin
SoftI2cMaster::SoftI2cMaster(uint8_t devAddr){
  initialized = false;
  deviceAddr = devAddr;
  _so_buffer = (uint8_t*) calloc(BUFF_LEN, sizeof(uint8_t));
}
//------------------------------------------------------------------------------
// init pins and set bus high
void SoftI2cMaster::initProtocol(uint8_t sclPin, uint8_t sdaPin){
  if ( initialized ) return;

  if (!sclPin && !sdaPin){
   #if defined(__AVR__)
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
      sclPin_ = 19;   // A5
      sdaPin_ = 18;   // A4
    #else
      sclPin_ = 21;
      sdaPin_ = 20;
    #endif
#elif defined(__PIC32MX__)
     #if defined(_BOARD_UNO_)
     	 sclPin_ = 46;   // use SCL pin 
    	 sdaPin_ = 45;   // use SDA pin 
     #elif defined(_BOARD_MEGA_)
     	 sclPin_ = 21;
       sdaPin_ = 20;
     #endif

#endif
  }
  else {
    sclPin_ = sclPin;
    sdaPin_ = sdaPin;
  }
  digitalWrite(sclPin_, LOW);
  digitalWrite(sdaPin_, LOW);
  pinMode(sclPin_,INPUT);
  pinMode(sdaPin_,INPUT);
  initialized = true;
}
//------------------------------------------------------------------------------
// read a byte and send Ack if last is false else Nak to terminate read
uint8_t SoftI2cMaster::read(uint8_t last){
  uint8_t b = 0;
  // make sure pullup enabled
  pinMode(sdaPin_,INPUT);
  // read byte
  for (uint8_t i = 0; i < 8; i++) {
  // don't change this loop unless you verify the change with a scope
  b <<= 1;
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,INPUT);
  if (digitalRead(sdaPin_)) b |= 1;
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,OUTPUT);
  }
  // send Ack or Nak
  pinMode(sdaPin_, OUTPUT);
  if (last)  pinMode(sdaPin_, INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,OUTPUT);
  pinMode(sdaPin_,INPUT);
  return b;
}
//------------------------------------------------------------------------------
// send new address and read/write with stop
uint8_t SoftI2cMaster::restart(uint8_t addressRW){
  uint8_t e;
  delayMicroseconds(I2C_DELAY_USEC);
  stop();
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,OUTPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  e = start(addressRW);
  delayMicroseconds(I2C_DELAY_USEC);
  return e;
}
//------------------------------------------------------------------------------
// issue a start condition for i2c address with read/write bit
uint8_t SoftI2cMaster::start(uint8_t addressRW){
  pinMode(sdaPin_,OUTPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,OUTPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  return write(addressRW);
}
//------------------------------------------------------------------------------
// issue a stop condition
void SoftI2cMaster::stop(void){
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sdaPin_,INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
}
//------------------------------------------------------------------------------
// write byte and return true for Ack or false for Nak
uint8_t SoftI2cMaster::write(uint8_t b){
  // write byte
  for (uint8_t m = 0X80; m != 0; m >>= 1) {
    if(m & b) pinMode(sdaPin_, INPUT);
    else pinMode(sdaPin_, OUTPUT);
    // don't change this loop unless you verify the change with a scope
    pinMode(sclPin_,INPUT);
    delayMicroseconds(I2C_DELAY_USEC);
    pinMode(sclPin_,OUTPUT);
    delayMicroseconds(I2C_DELAY_USEC);
  }
  // get Ack or Nak
  pinMode(sdaPin_,INPUT);
  pinMode(sdaPin_, INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sclPin_,INPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  b = digitalRead(sdaPin_);
  pinMode(sclPin_,OUTPUT);
  delayMicroseconds(I2C_DELAY_USEC);
  pinMode(sdaPin_, OUTPUT);
  if ( b != 0 ) stop ();
  return b == 0;
}
//------------------------------------------------------------------------------
// read number of bytes from start register and return values; optional buffer
uint8_t* SoftI2cMaster::readRegisters(uint8_t startRegister, uint8_t bytes, uint8_t* buf){
  //delay(20);
  if (!buf)    buf = _so_buffer;
  bytes = min(bytes,BUFF_LEN);  // avoid buffer overflow
  memset(buf, 0, BUFF_LEN);
  _error_code = 0;
  // issue a start condition, send device address and write direction bit
  if (!start(deviceAddr | I2C_WRITE)) {
		_error_code = 2;
	  return false;
	}
  // send the start register address  
  if (!write(startRegister)) {
		_error_code = 1;
	  return false;
	}
  // issue a repeated stop and start condition, send device address and read direction bit  
  if (!restart(deviceAddr | I2C_READ)) {
		_error_code = 3;
	  return false;
	}
  // read data from the device and store into buffer  
  for (uint8_t i = 0; i < bytes; i++) {
    // send Ack until last byte then send Ack
    buf[i] = read(i == (bytes-1));
  }
  // issue a stop condition  
  stop();
  return buf;
}
//------------------------------------------------------------------------------
// write number of bytes from buffer, location is included in the buf
bool SoftI2cMaster::writeRegistersWithLocation(int bytes, uint8_t* buf){
  if (!start(deviceAddr | I2C_WRITE)) return false;
  for (int i = 0; i < bytes; i++){
    if (!write(buf[i]))  return false;
  }
  stop();
  return true;
}

//------------------------------------------------------------------------------
// write number of bytes from buffer at specified location.
bool SoftI2cMaster::writeRegisters  (uint8_t location, uint8_t bytes_to_write,
              uint8_t* buffer)
{
  if (!buffer) buffer = _so_buffer;

  uint8_t buf[BUFF_LEN];
  memset(buf, 0, BUFF_LEN);
  buf[0] = location;
  memmove(buf+1, buffer, bytes_to_write);
  return writeRegistersWithLocation(bytes_to_write+1, buf);
}

//------------------------------------------------------------------------------
// write a single byte at speccified location.
bool SoftI2cMaster::writeByte  (uint8_t location, uint8_t data)
{
  return writeRegisters(location, 1, &data);
}

//------------------------------------------------------------------------------
// write one integer (2 bytes) at speccified location.
bool SoftI2cMaster::writeInteger(uint8_t location, uint16_t data)
{
  uint8_t buf[2];
  buf[0] = data & 0xFF;
  buf[1] = (data >> 8) & 0xFF;
  return writeRegisters(location, 2, buf);
}

//------------------------------------------------------------------------------
// write one long (4 bytes) at speccified location.
bool SoftI2cMaster::writeLong  (uint8_t location, uint32_t data)
{
  uint8_t buf[4];

  buf[0] = data & 0xFF;
  buf[1] = (data >>  8) & 0xFF;
  buf[2] = (data >> 16) & 0xFF;
  buf[3] = (data >> 24) & 0xFF;
  return writeRegisters(location, 4, buf);
}

//------------------------------------------------------------------------------
// read string: convert read registers to character string
char* SoftI2cMaster::readString(uint8_t startRegister, uint8_t bytes,
                    uint8_t* buf, uint8_t len)
{
	char *x;
	x = (char *) readRegisters(startRegister, bytes, buf);
	if ( x == false ) {
	  return (char *) "";
	}
  return (char *) x;
}
//------------------------------------------------------------------------------
// read one byte, total of 8 bits
uint8_t SoftI2cMaster::readByte (uint8_t location){
  if ( readRegisters(location, 1) != false ) {
		return _so_buffer[0];
  } else {
	  return 0;
	}
}
//------------------------------------------------------------------------------
// read integer, total of 2 bytes or 16 bits
int16_t SoftI2cMaster::readInteger (uint8_t location){

  if ( readRegisters(location, 2) != false ) {
		return _so_buffer[0] | (_so_buffer[1] << 8);
  } else {
	  return 0;
	}
}
//------------------------------------------------------------------------------
// read long integer, total of 4 bytes or 32 bits
uint32_t SoftI2cMaster::readLong (uint8_t location){

  if ( readRegisters(location, 4) != false ) {
		return (uint32_t)_so_buffer[0] |
               (((uint32_t)_so_buffer[1]) << 8) |
               (((uint32_t)_so_buffer[2]) << 16) |
               (((uint32_t)_so_buffer[3]) << 24);
  } else {
	  return 0;
	}
}

// This is the status value returned from the last write command.
// A return value of zero indicates success.
// Non-zero results indicate failures.  From libraries/Wire/utility/twi.c, they are:
//                1: write fail
//                2: start fail
//                3: restart fail
uint8_t SoftI2cMaster::getWriteErrorCode()
{
  return _error_code;
}

//------------------------------------------------------------------------------
// get firmware version
char* SoftI2cMaster::getFirmwareVersion(){
  return readString(0x00, 8);
}
//------------------------------------------------------------------------------
// get vendor ID
char* SoftI2cMaster::getVendorID(){  
  return readString(0x08, 8);
}
//------------------------------------------------------------------------------
// get device ID
char* SoftI2cMaster::getDeviceID(){
  return readString(0x10, 8);
}
/**
 * this function checks to see if there is 
 * any device at its specified address 
 */
bool SoftI2cMaster::checkAddress()
{
// FIXME: This function is not working correctly.
  uint8_t *txBuffer;
  int8_t x = 1;
	if (!start(deviceAddr | I2C_WRITE)) return false;
	stop();
	return true;
}

bool SoftI2cMaster::setAddress(uint8_t address)
{
  deviceAddr = address;
  return true;
}
