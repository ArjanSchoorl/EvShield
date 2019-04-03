//EVShield interface library for Mindsensor Sensors

#include "MindsensorsLib.h"

AbsoluteIMU::AbsoluteIMU(uint8_t i2c_address)
 : EVShieldI2C(i2c_address)
{

}

uint8_t	AbsoluteIMU::issueCommand(char command)
{
	return	writeByte(IMU_Command, (uint8_t) command);
}

void AbsoluteIMU::readGyro(gyro &currgyro)
{
	char *b;
	char str[200];
 	b = readString(0x53, 6);
	currgyro.gx = readIntFromBuffer((uint8_t *)&b[0]);
	currgyro.gy = readIntFromBuffer((uint8_t *)&b[2]);
	currgyro.gz = readIntFromBuffer((uint8_t *)&b[4]);
	currgyro.error = getErrorCode();
}

void AbsoluteIMU::readCompass(cmps &currcmps)
{
	char str[200];
 	currcmps.heading = readInteger(0x4b);
	currcmps.error = getErrorCode();
}

void AbsoluteIMU::readAccelerometer(accl &currAccl)
{
	char *b;
	char str[200];
 	currAccl.tx = readByte(0x42);
	currAccl.ty = readByte(0x43);
	currAccl.tz = readByte(0x44);
	b = readString(0x45, 6);
	currAccl.ax = readIntFromBuffer((uint8_t *)&b[0]);
	currAccl.ay = readIntFromBuffer((uint8_t *)&b[2]);
	currAccl.az = readIntFromBuffer((uint8_t *)&b[4]);
	currAccl.error = getErrorCode();
}
void AbsoluteIMU::readMagneticField(magnetic_field &currmagnetic_field)
{
	char *b;
	char str[200];
 	b = readString(0x4d, 6);
	currmagnetic_field.mx = readIntFromBuffer((uint8_t *)&b[0]);
	currmagnetic_field.my = readIntFromBuffer((uint8_t *)&b[2]);
	currmagnetic_field.mz = readIntFromBuffer((uint8_t *)&b[4]);
	currmagnetic_field.error = getErrorCode();
}
bool AbsoluteIMU::beginCompassCalibration()
{
	return issueCommand('C');
}
bool AbsoluteIMU::endCompassCalibration()
{
	return issueCommand('c');
}

/**
 This bool interfaces with mindsensors Angle sensor attached to EVShield
 */
AngleSensor::AngleSensor(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

long AngleSensor::getAngle()
{
    return readLong(ANGLE);
}

long AngleSensor::getRawReading()
{

    return readLong(RAW_READING);
}

void AngleSensor::reset()
{
    writeByte(0x41, (uint8_t)'r');
}

/**
 This bool interfaces with mindsensors Angle sensor attached to EVShield
 */
DISTNx::DISTNx(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}
uint8_t DISTNx::issueCommand(char command)
{
    return writeByte(DISTNx_Command, (uint8_t)command);
}

bool DISTNx::energize()
{
    return issueCommand('E');
}

bool DISTNx::deEnergize()
{
    return issueCommand('D');
}

int DISTNx::getDist()
{
    return readInteger(DISTNx_Distance);
}

int DISTNx::getVolt()
{
    return readInteger(DISTNx_Voltage);
}

short DISTNx::getType()
{
    return readByte(DISTNx_SensorType);
}

/**
 This bool interfaces with mindsensors Thermometer attached to EVShield
 */
IRThermometer::IRThermometer(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}
uint8_t IRThermometer::issueCommand(char command)
{
    return writeByte(IRThermometer_Command, (uint8_t)command);
}

float IRThermometer::getAmbientTemperatureC()
{
    return (float)readInteger(IRThermometer_Ambient_Temperature_C) / 100.0;
}

float IRThermometer::getTargetTemperatureC()
{
    return (float)readInteger(IRThermometer_Target_Temperature_C) / 100.0;
}

float IRThermometer::getAmbientTemperatureF()
{
    return (float)readInteger(IRThermometer_Ambient_Temperature_F) / 100.0;
}

float IRThermometer::getTargetTemperatureF()
{
    return (float)readInteger(IRThermometer_Target_Temperature_F) / 100.0;
}

/**
 This bool interfaces with mindsensors LightSensorArray attached to EVShield
 */
LightSensorArray::LightSensorArray(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

uint8_t LightSensorArray::issueCommand(char command)
{
    return writeByte(LightSensorArray_Command, (uint8_t)command);
}

bool LightSensorArray::calibrateWhite()
{
    return issueCommand('W');
}

bool LightSensorArray::calibrateBlack()
{
    return issueCommand('B');
}

bool LightSensorArray::sleep()
{
    return issueCommand('D');
}

bool LightSensorArray::wakeUp()
{
    return issueCommand('P');
}

bool LightSensorArray::configureUS()
{
    return issueCommand('A');
}

bool LightSensorArray::configureEurope()
{
    return issueCommand('E');
}

bool LightSensorArray::configureUniversal()
{
    return issueCommand('U');
}

uint8_t *LightSensorArray::getCalibrated()
{
    return (uint8_t *)readString(LightSensorArray_Calibrated, 8);
}

uint8_t *LightSensorArray::getUncalibrated()
{
    return (uint8_t *)readString(LightSensorArray_Uncalibrated, 16);
}

uint8_t *LightSensorArray::getWhiteLimit()
{
    return (uint8_t *)readString(LightSensorArray_White_Limit, 8);
}

uint8_t *LightSensorArray::getBlackLimit()
{
    return (uint8_t *)readString(LightSensorArray_Black_Limit, 8);
}

uint8_t *LightSensorArray::getWhiteCalibration()
{
    return (uint8_t *)readString(LightSensorArray_White_Calibration, 8);
}

uint8_t *LightSensorArray::getBlackCalibration()
{
    return (uint8_t *)readString(LightSensorArray_Black_Calibration, 8);
}

/**
 This bool interfaces with mindsensors LineLeader attached to EVShield
 */
LineLeader::LineLeader(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

uint8_t LineLeader::issueCommand(char command)
{
    return writeByte(LineLeader_Command, (uint8_t)command);
}

bool LineLeader::calibrateWhite()
{
    return issueCommand('W');
}

bool LineLeader::calibrateBlack()
{
    return issueCommand('B');
}

bool LineLeader::sleep()
{
    return issueCommand('D');
}

bool LineLeader::wakeUp()
{
    return issueCommand('P');
}

bool LineLeader::invertLineColorToWhite()
{
    return issueCommand('I');
}

bool LineLeader::resetColorInversion()
{
    return issueCommand('R');
}

bool LineLeader::takeSnapshot()
{
    return issueCommand('S');
}

bool LineLeader::configureUS()
{
    return issueCommand('A');
}

bool LineLeader::configureEurope()
{
    return issueCommand('E');
}

bool LineLeader::configureUniversal()
{
    return issueCommand('U');
}

uint8_t LineLeader::getSetPoint()
{
    return readByte(LineLeader_SetPoint);
}

bool LineLeader::setSetPoint(uint8_t spoint)
{
    return writeByte(LineLeader_SetPoint, (uint8_t)spoint);
}

uint8_t LineLeader::getKp(uint8_t kp)
{
    return readByte(LineLeader_Kp);
}

bool LineLeader::setKp(uint8_t kp)
{
    return writeByte(LineLeader_Kp, (uint8_t)kp);
}

uint8_t LineLeader::getKi(uint8_t ki)
{
    return readByte(LineLeader_Ki);
}

bool LineLeader::setKi(uint8_t ki)
{
    return writeByte(LineLeader_Ki, (uint8_t)ki);
}

uint8_t LineLeader::getKd(uint8_t kd)
{
    return readByte(LineLeader_Kd);
}

bool LineLeader::setKd(uint8_t kd)
{
    return writeByte(LineLeader_Kd, (uint8_t)kd);
}

uint8_t LineLeader::getKpFactor(uint8_t kpfact)
{
    return readByte(LineLeader_Kp);
}

bool LineLeader::setKpFactor(uint8_t kpfact)
{
    return writeByte(LineLeader_Kp_Factor, (uint8_t)kpfact);
}

uint8_t LineLeader::getKiFactor(uint8_t kifact)
{
    return readByte(LineLeader_Ki);
}

bool LineLeader::setKiFactor(uint8_t kifact)
{
    return writeByte(LineLeader_Ki_Factor, (uint8_t)kifact);
}

uint8_t LineLeader::getKdFactor(uint8_t kdfact)
{
    return readByte(LineLeader_Kd);
}

bool LineLeader::setKdFactor(uint8_t kdfact)
{
    return writeByte(LineLeader_Kd_Factor, (uint8_t)kdfact);
}

int LineLeader::getSteering()
{
    return readByte(LineLeader_Steering);
}

unsigned char LineLeader::getAverage()
{
    return readByte(LineLeader_Average);
}

unsigned char LineLeader::getResult()
{
    return readByte(LineLeader_Result);
}

uint8_t *LineLeader::getRawCalibrated()
{
    return (uint8_t *)readString(LineLeader_Raw_Calibrated, 8);
}

uint8_t *LineLeader::getRawUncalibrated()
{
    return (uint8_t *)readString(LineLeader_Raw_Uncalibrated, 16);
}

uint8_t *LineLeader::getWhiteLimit()
{
    return (uint8_t *)readString(LineLeader_White_Limit, 8);
}

uint8_t *LineLeader::getBlackLimit()
{
    return (uint8_t *)readString(LineLeader_Black_Limit, 8);
}

uint8_t *LineLeader::getWhiteCalibration()
{
    return (uint8_t *)readString(LineLeader_White_Calibration, 8);
}

uint8_t *LineLeader::getBlackCalibration()
{
    return (uint8_t *)readString(LineLeader_Black_Calibration, 8);
}

/**
 This bool interfaces with mindsensors MagicWand attached to EVShield
 */
MagicWand::MagicWand(uint8_t i2c_address) : EVShieldI2C(i2c_address)
{
}

void MagicWand::lightWand(uint8_t byteToWrite)
{
    writeByte(byteToWrite, byteToWrite);
}

/**
 This bool interfaces with mindsensors NumericPad attached to EVShield
 */
/*
byte Group1[] = {0x5C, 0x0B, 0x20};
byte Group2[] = {0x2B, 1, 1, 0, 0, 1, 1, 0xFF, 2};
byte Group3[] = {0x41, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15};
byte Group4[] = {0x50, 10, 15, 10, 15, 10, 15, 10, 15, 10, 15, 10};
byte Group5[] = {0x7D, 156, 101, 140, 0x0C};
*/
byte Group1[] = {0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0xFF, 0x02};
byte Group2[] = {0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F};
byte Group3[] = {0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F};
byte Group4[] = {0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F};
byte Group5[] = {0x0b, 0x20, 0x0C};
byte Group6[] = {0x9C, 0x65, 0x8C};
byte waitPeriod = 1;
byte Group7[] = {0x0B};

char keyMap[] = {'4', '1', '7', '*', '5', '2', '8', '0', '3', '6', '9', '#'};

uint8_t KEY_STATUS_REG = 0x00;

NumericPad::NumericPad(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

void NumericPad::InitializeKeypad()
{
    // the following sequence of bytes initialize the sensor's
    // configuration for performance.

    // This function must be called at the beginning of every power cycle
    // (call it at the beginning of your program, after you initialize the port).

    // Do not change the values below
    // Or the order in which these values are written.
    writeRegisters(0x41, 9, Group2);
    writeRegisters(0x4A, 8, Group3);
    writeRegisters(0x52, 8, Group4);
    writeRegisters(0x5C, 3, Group5);
    writeRegisters(0X2B, 8, Group1);
    writeRegisters(0x7B, 1, Group7);
    writeRegisters(0x7D, 3, Group6);
}

bool NumericPad::GetKeyPress(int waitPeriod /* seconds */, byte &keyHolder)
{
    uint8_t *regValue;
    int Touch;
    int Previous_Touch;
    int bit_test;
    int i, j;
    int w, cw;
    w = waitPeriod * 1000;

    cw = 0; // cumulative wait
    while (true)
    {

        Touch = readInteger(KEY_STATUS_REG);
        int b;
        b = 0x01 << 11;

        if (Previous_Touch != Touch)
        {
            Previous_Touch = Touch;

            for (j = 0, i = 0; j < 12; j++)
            {
                if (Touch & b)
                {
                    keyHolder = keyMap[j];
                    return true;
                }
                b = b >> 1;
            }
        }
        delay(150);
        cw += 150;
        if (w != 0 && cw > w)
        {
            return false;
        }
    }
    return false;
}

int NumericPad::GetKeysPressed()
{ //Returns a 12bit number containing the status of all 12 keys.
    uint8_t buf[4];
    int result;
    result = readInteger(KEY_STATUS_REG);
    return result;
}

/**
  This bool interfaces with PFMate attached to EVShield 
	*/
PFMate::PFMate(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}
uint8_t PFMate::issueCommand(char command)
{
    return writeByte(PF_Commmand, (uint8_t)command);
}

bool PFMate::sendSignal()
{
    return issueCommand('G');
}

bool PFMate::setChannel(uint8_t channel)
{
    return writeByte(PF_Channel, (uint8_t)channel);
}

bool PFMate::setControl(uint8_t control)
{
    return writeByte(PF_Control, (uint8_t)control);
}

bool PFMate::setOperationA(uint8_t operation)
{
    return writeByte(PF_Operation_A, (uint8_t)operation);
}

bool PFMate::setOperationB(uint8_t operation)
{
    return writeByte(PF_Operation_B, (uint8_t)operation);
}

bool PFMate::setSpeedA(uint8_t speed)
{
    return writeByte(PF_Speed_A, (uint8_t)speed);
}

bool PFMate::setSpeedB(uint8_t speed)
{
    return writeByte(PF_Speed_B, (uint8_t)speed);
}

void PFMate::controlMotor(
    uint8_t channel,   // PF_Channel_1, 2, 3, or 4
    uint8_t control,   // PF_Control_Both, A, or B
    uint8_t operation, // PF_Operation_Forward, Reverse, Float, or Brake
    uint8_t speed)     // [1, 7] or PF_Speed_Full, Medium, or Slow

{
    setChannel(channel);
    if (control == PF_Control_Both)
    {
        setOperationA(operation);
        setSpeedA(speed);
        setOperationB(operation);
        setSpeedB(speed);
    }

    else if (control == PF_Control_A)
    {
        setOperationA(operation);
        setSpeedA(speed);
    }

    else if (control == PF_Control_B)
    {
        setOperationB(operation);
        setSpeedB(speed);
    }
    sendSignal();
}

/**
  This class interfaces with PSP-Nx attached to EVShield 
	*/
PSPNx::PSPNx(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

uint8_t PSPNx::issueCommand(char command)
{
    return writeByte(PSPNx_Command, (uint8_t)command);
}

bool PSPNx::energize()
{
    return issueCommand('R');
}

bool PSPNx::deEnergize()
{
    return issueCommand('S');
}

bool PSPNx::setDigitalMode()
{
    return issueCommand('A');
}

bool PSPNx::setAnalogMode()
{
    return issueCommand('s');
}

int8_t PSPNx::getXLJoy()
{
    int16_t a;
    int8_t b;
    a = readByte(PSPNx_XLeftJoystick);
    b = (((a - 128) * 25) >> 5) & 0xFF;
    return b;
}
int8_t PSPNx::getYLJoy()
{
    int16_t a;
    int8_t b;
    a = readByte(PSPNx_YLeftJoystick);
    b = (((a - 128) * 25) >> 5) & 0xFF;
    return b;
}
int8_t PSPNx::getXRJoy()
{
    int16_t a;
    int8_t b;
    a = readByte(PSPNx_XRightJoystick);
    b = (((a - 128) * 25) >> 5) & 0xFF;
    return b;
}
int8_t PSPNx::getYRJoy()
{
    int16_t a;
    int8_t b;
    a = readByte(PSPNx_YRightJoystick);
    b = (((a - 128) * 25) >> 5) & 0xFF;
    return b;
}

void PSPNx::getButtons(int8_t *buttons1, int8_t *buttons2)
{
    int8_t buf1, buf2;
    buf1 = readByte(PSPNx_ButtonSet1);
    buf2 = readByte(PSPNx_ButtonSet2);
    for (int i = 0; i < 8; i++)
    {
        buttons1[i] = buf1 >> i & 0x01;
        buttons2[i] = buf2 >> i & 0x01;
    }
}

/**
  This class interfaces with PiLight sensor attached to EVShield 
  */
PiLight::PiLight(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

void PiLight::readPiLight(color &currcolor)
{
    char *b;
    char str[200];
    b = readString(PILIGHT_RED, 3);
    currcolor.r = readIntFromBuffer((uint8_t *)&b[0]);
    currcolor.g = readIntFromBuffer((uint8_t *)&b[1]);
    currcolor.b = readIntFromBuffer((uint8_t *)&b[2]);
}

void PiLight::setTimeout1(uint8_t timeoutValue)
{
    uint8_t null = 0;
    writeByteToBuffer(_buffer + 0, null);
    writeByteToBuffer(_buffer + 1, null);
    writeByteToBuffer(_buffer + 2, null);
    writeByteToBuffer(_buffer + 3, timeoutValue);
    writeRegisters(PILIGHT_RED, 4, _buffer);
}

void PiLight::createPiLight(uint8_t red, uint8_t green, uint8_t blue)
{
    writeByteToBuffer(_buffer + 0, red);
    writeByteToBuffer(_buffer + 1, green);
    writeByteToBuffer(_buffer + 2, blue);
    writeRegisters(PILIGHT_RED, 3, _buffer);
}

/**
  This class interfaces with RTC attached to EVShield 
	*/
RTC::RTC(uint8_t i2c_address)
    : EVShieldI2C(i2c_address)
{
}

uint8_t RTC::BCDToInteger(uint8_t b)
{
    uint8_t i;
    i = (b & 0x0F) + (((b >> 4) & 0x0F) * 10);
    return i;
}

uint8_t RTC::getSeconds()
{
    return BCDToInteger(readByte(RTC_Seconds));
}

uint8_t RTC::getMinutes()
{
    return BCDToInteger(readByte(RTC_Minutes));
}

uint8_t RTC::getHours()
{
    return BCDToInteger(readByte(RTC_Hours));
}

uint8_t RTC::getDayWeek()
{
    return BCDToInteger(readByte(RTC_Day_of_Week));
}

uint8_t RTC::getDayMonth()
{
    return BCDToInteger(readByte(RTC_Day_of_Month));
}

uint8_t RTC::getMonth()
{
    return BCDToInteger(readByte(RTC_Month));
}

uint8_t RTC::getYear()
{
    return BCDToInteger(readByte(RTC_Year));
}

bool SumoEyes::init(EVShield *shield, BankPort bp)
{
    EVShieldAGS::init(shield, bp);
}

/**
  This class interfaces with EVTSumoEyes sensor attached to EVShield 
	*/
bool SumoEyes::setType(int8_t type)
{
    if (mp_shield == NULL)
        return false;
    switch (m_bp)
    {
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

bool SumoEyes::setShortRange()
{
    return setType(Type_LIGHT_AMBIENT);
}

bool SumoEyes::setLongRange()
{
    return setType(Type_LIGHT_REFLECTED);
}

bool SumoEyes::isNear(int reference, int delta, int comet)
{
    if ((comet > (reference - delta)) && (comet < (reference + delta)))
    {
        return true;
    }
    return false;
}

SE_Zone SumoEyes::detectObstacleZone()
{
    int se_value;

    se_value = readRaw();

    if (isNear(830, 10, se_value))
        return SE_Front;
    if (isNear(580, 10, se_value))
        return SE_Left;
    if (isNear(487, 10, se_value))
        return SE_Right;

    return (SE_None);
}

char *SumoEyes::OBZoneToString(SE_Zone ob)
{
    switch (ob)
    {
    case SE_None:
        return (char *)"NONE";
        break;
    case SE_Front:
        return (char *)"FRONT";
        break;
    case SE_Left:
        return (char *)"LEFT";
        break;
    case SE_Right:
        return (char *)"RIGHT";
        break;
    }
}