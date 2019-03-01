//EVShield interface library for NXT Sensors

#include "EVLIB.h"

/**
  This bool interfaces with LEGO Touch sensor attached to EVShield.
	*/
bool NXTTouch::init(EVShield *shield, BankPort bp)
{
	EVShieldAGS::init(shield, bp);
	EVShieldAGS::setType(Type_ANALOG);
}

bool NXTTouch::isPressed()
{
	int a;
	a = readRaw();

	if (a < 300)
		return true;
	else
		return false;
}

/**
  This bool interfaces with LEGO Light sensor attached to EVShield.
	*/
bool NXTLight::init(EVShield *shield, BankPort bp)
{
	EVShieldAGS::init(shield, bp);
}

bool NXTLight::setReflected()
{
	//return setType( Type_ANALOG | Type_DATABIT0_HIGH );
	return setType(Type_LIGHT_REFLECTED);
}

bool NXTLight::setAmbient()
{
	return setType(Type_LIGHT_AMBIENT);
}

/**
  This bool interfaces with LEGO Color sensor attached to EVShield.
	*/
NXTColor::NXTColor()
{
	mp_shield = NULL;
}

NXTColor::NXTColor(EVShield *shield, BankPort bp)
{
	mp_shield = shield;
	m_bp = bp;
	switch (m_bp)
	{
	case BAS1:
	case BBS1:
		m_offset = 0;
		break;
	case BAS2:
	case BBS2:
		m_offset = 52;
		break;
	}
	//setType(Type_COLORFULL);
}

bool NXTColor::init(EVShield *shield, BankPort bp)
{
	mp_shield = shield;
	m_bp = bp;
	switch (m_bp)
	{
	case BAS1:
	case BBS1:
		m_offset = 0;
		break;
	case BAS2:
	case BBS2:
		m_offset = 52;
		break;
	}
	//setType(Type_COLORFULL);
	return true;
}

bool NXTColor::setType(uint8_t type)
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

uint8_t NXTColor::readValue()
{
	char str[100];
	uint8_t val;
	switch (m_bp)
	{
	case BAS1:
	case BAS2:
		val = (mp_shield->bank_a.readByte((0x70 + m_offset)));
		return (val * 100) / 255;
	case BBS1:
	case BBS2:
		val = (mp_shield->bank_b.readByte((0x70 + m_offset)));
		return (val * 100) / 255;
	}
}

uint8_t NXTColor::readColor()
{
	char str[100];
	uint8_t val;
	/*
    Serial.println("NXTColor::readValue: ");
    for (int i=0x70; i< 0x70+10;i++) {
        Serial.print(" "); Serial.print(mp_shield->bank_a.readByte(i));
    }
    Serial.println("");
    */
	switch (m_bp)
	{
	case BAS1:
	case BAS2:
		return (mp_shield->bank_a.readByte((0x70 + m_offset)));
	case BBS1:
	case BBS2:
		return (mp_shield->bank_b.readByte((0x70 + m_offset)));
	}
}

/**
  This bool interfaces with NXTCam attached to EVShield.
	*/

NXTCam::NXTCam(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
}

uint8_t NXTCam::issueCommand(char command)
{
	return writeByte(Cam_Command, (uint8_t)command);
}

bool NXTCam::sortSize()
{
	return issueCommand('A');
}

bool NXTCam::selectObjectMode()
{
	return issueCommand('B');
}

bool NXTCam::writeImageRegisters()
{
	return issueCommand('C');
}

bool NXTCam::disableTracking()
{
	return issueCommand('D');
}

bool NXTCam::enableTracking()
{
	return issueCommand('E');
}

bool NXTCam::getColorMap()
{
	return issueCommand('G');
}

bool NXTCam::illuminationOn()
{
	return issueCommand('I');
}

bool NXTCam::readImageRegisters()
{
	return issueCommand('H');
}

bool NXTCam::selectLineMode()
{
	return issueCommand('L');
}

bool NXTCam::pingCam()
{
	return issueCommand('P');
}

bool NXTCam::resetCam()
{
	return issueCommand('R');
}

bool NXTCam::sendColorMap()
{
	return issueCommand('S');
}

bool NXTCam::illuminationOff()
{
	return issueCommand('T');
}

bool NXTCam::sortColor()
{
	return issueCommand('U');
}

bool NXTCam::camFirmware()
{
	return issueCommand('V');
}

bool NXTCam::sortNone()
{
	return issueCommand('X');
}

int NXTCam::getNumberObjects()
{
	return readByte(Cam_Number_Objects);
}

void NXTCam::getBlobs(int *nblobs, uint8_t *color, uint8_t *left, uint8_t *top, uint8_t *right, uint8_t *bottom)
{
	*nblobs = readByte(Cam_Number_Objects);
	for (int i = 0; i < *nblobs; i++)
	{
		uint8_t *buf;
		buf = (uint8_t *)readString(Start_Reg + (i * 5), 5);
		color[i] = buf[0] & 0x00FF;
		left[i] = buf[1] & 0x00FF;
		top[i] = buf[2] & 0x00FF;
		right[i] = buf[3] & 0x00FF;
		bottom[i] = buf[4] & 0x00FF;
	}
}

CurrentMeter::CurrentMeter(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
}

/**
  This bool interfaces with NXTCurrentMeter attached to EVShield.
	*/
uint8_t CurrentMeter::issueCommand(char command)
{
	return writeByte(IM_Command, (uint8_t)'d');
}

int CurrentMeter::getACurrent()
{
	return readInteger(ABSOLUTE_I);
}

int CurrentMeter::getRCurrent()
{
	return readInteger(RELATIVE_I);
}

int CurrentMeter::getReference()
{
	return readInteger(REFERENCE_I);
}

int CurrentMeter::setReferenceI()
{
	return issueCommand('d');
}

/**
   This class interfaces with NXTMMX attached to EVShield 
  */
NXTMMX::NXTMMX(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
	// for some reason the reset command doesn't work here
	//reset();	// start all encoder positions off at 0
}

uint8_t NXTMMX::getBatteryVoltage()
{
	return readByte(MMX_VOLTAGE) * 39;
}

uint8_t NXTMMX::issueCommand(char command)
{
	return writeByte(MMX_COMMAND, (uint8_t)command);
}

// Set/get the encoder target for the motor (ie. a position to go
// to and stop at)
bool NXTMMX::setEncoderTarget(uint8_t which_motor, long target)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_SETPT_M1 : MMX_SETPT_M2;
	return writeLong(reg, target);
}
long NXTMMX::getEncoderTarget(uint8_t which_motor)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_SETPT_M1 : MMX_SETPT_M2;
	return (long)readLong(reg);
}

// Set/get the speed of the motor
// (I believe this is in the range [-100, +100])
bool NXTMMX::setSpeed(uint8_t which_motor, int speed)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_SPEED_M1 : MMX_SPEED_M2;
	return writeByte(reg, (uint8_t)(int8_t)speed);
}
int8_t NXTMMX::getSpeed(uint8_t which_motor)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_SPEED_M1 : MMX_SPEED_M2;
	return (int8_t)readByte(reg);
}

// This is the time, in seconds, for the motor to run
bool NXTMMX::getTimeToRun(uint8_t which_motor, int seconds)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_TIME_M1 : MMX_TIME_M2;
	return writeByte(reg, seconds);
}
uint8_t NXTMMX::getTimeToRun(uint8_t which_motor)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_TIME_M1 : MMX_TIME_M2;
	return readByte(reg);
}

// Command Register 'B' is currently unused, but reserved for future expansion
// If you set it, you must set it to zero.
bool NXTMMX::setCommandRegB(uint8_t which_motor, uint8_t value)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_CMD_B_M1 : MMX_CMD_B_M2;
	return writeByte(reg, value);
}
uint8_t NXTMMX::getCommandRegB(uint8_t which_motor)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_CMD_B_M1 : MMX_CMD_B_M2;
	return readByte(reg);
}

// See User's Guide for what command register A does
bool NXTMMX::setCommandRegA(uint8_t which_motor, uint8_t value)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_CMD_A_M1 : MMX_CMD_A_M2;
	return writeByte(reg, value);
}
uint8_t NXTMMX::getCommandRegA(uint8_t which_motor)
{
	uint8_t reg = (which_motor == MMX_Motor_1) ? MMX_CMD_A_M1 : MMX_CMD_A_M2;
	return readByte(reg);
}

// Get the current encoder position
int32_t NXTMMX::getEncoderPosition(uint8_t which_motor)
{
	uint8_t location = (which_motor == MMX_Motor_1) ? MMX_POSITION_M1 : MMX_POSITION_M2;
	return (int32_t)readLong(location);
}

// See User's Guide for documentation on the status byte
uint8_t NXTMMX::getMotorStatusByte(uint8_t which_motor)
{
	uint8_t location = (which_motor == MMX_Motor_1) ? MMX_STATUS_M1 : MMX_STATUS_M2;
	return readByte(location);
}

// (I couldn't find an explanation for this in the User's Guide)
uint8_t NXTMMX::getMotorTasksRunningByte(uint8_t which_motor)
{
	uint8_t location = (which_motor == MMX_Motor_1) ? MMX_TASKS_M1 : MMX_TASKS_M2;
	return readByte(location);
}

// Set the PID that controls how we stop as we approach the
// angle we're set to stop at
bool NXTMMX::setEncoderPID(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
	writeIntToBuffer(_buffer + 0, Kp);
	writeIntToBuffer(_buffer + 2, Ki);
	writeIntToBuffer(_buffer + 4, Kd);
	return writeRegisters(MMX_ENCODER_PID, 6, _buffer);
}

// Sets the PID that controls how well that motor maintains its speed
bool NXTMMX::setSpeedPID(uint16_t Kp, uint16_t Ki, uint16_t Kd)
{
	writeIntToBuffer(_buffer + 0, Kp);
	writeIntToBuffer(_buffer + 2, Ki);
	writeIntToBuffer(_buffer + 4, Kd);
	return writeRegisters(MMX_SPEED_PID, 6, _buffer);
}

// See user's guide for details.
bool NXTMMX::setPassCount(uint8_t pass_count)
{
	return writeByte(MMX_PASS_COUNT, pass_count);
}

// Sets tolerance which adjust accuracy while positioning.
// See user's guide for more details.
bool NXTMMX::setTolerance(uint8_t tolerance)
{
	return writeByte(MMX_TOLERANCE, tolerance);
}

// Special I2C commands

// Resets all encoder values and motor parameters.  Leaves PIDs untouched.
bool NXTMMX::reset()
{
	return issueCommand('R');
}

// Tells the motors to start at the same time.
bool NXTMMX::startMotorsInSync()
{
	return issueCommand('S');
}

// Reset the encoder for motor 1 or motor 2
bool NXTMMX::resetEncoder(uint8_t which_motor)
{
	char code = (which_motor == MMX_Motor_1) ? 'r' : 's';
	return issueCommand(code);
}

// This function sets the speed, the number of seconds, and
// the control (a.k.a. command register A)
bool NXTMMX::setSpeedTimeAndControl(
		uint8_t which_motors, // MMX_Motor_ 1, 2, or Both
		int speed,						// in range [-100, +100]
		uint8_t duration,			// in seconds
		uint8_t control)			// bit flags for control purposes
{
	if (which_motors == MMX_Motor_Both)
	{
		control &= ~MMX_CONTROL_GO; // Clear the 'go right now' flag
		bool m1 = setSpeedTimeAndControl(MMX_Motor_1, speed, duration, control);
		bool m2 = setSpeedTimeAndControl(MMX_Motor_2, speed, duration, control);
		startMotorsInSync();
		return m1 && m2;
	}

	_buffer[0] = (uint8_t)(int8_t)speed;
	_buffer[1] = duration;
	_buffer[2] = 0;				// command register B
	_buffer[3] = control; // command register A

	uint8_t reg = (which_motors == MMX_Motor_1) ? MMX_SPEED_M1 : MMX_SPEED_M2;
	return writeRegisters(reg, 4);
}

void setEncoderSpeedTimeAndControlInBuffer(
		uint8_t *buffer,	// pointer to the buffer
		long encoder,			// encoder value
		int speed,				// speed, in range [-100, +100]
		uint8_t duration, // in seconds
		uint8_t control)	// control flags
{
	writeLongToBuffer(buffer + 0, (uint32_t)(int32_t)encoder);
	buffer[4] = (uint8_t)(int8_t)speed;
	buffer[5] = duration;
	buffer[6] = 0;			 // command register B
	buffer[7] = control; // command register A
}

// This function sets the speed, the number of seconds, and
// the control (a.k.a. command register A)
bool NXTMMX::setEncoderSpeedTimeAndControl(
		uint8_t which_motors, // MMX_Motor_ 1, 2, or Both
		long encoder,					// encoder/tachometer position
		int speed,						// speed, in range [-100, +100]
		uint8_t duration,			// in seconds
		uint8_t control)			// control flags
{
	if (which_motors == MMX_Motor_Both)
	{
		// The motor control registers are back to back, and both can be written in one command
		control &= ~MMX_CONTROL_GO; // Clear the 'go right now' flag
		setEncoderSpeedTimeAndControlInBuffer(_buffer + 0, encoder, speed, duration, control);
		setEncoderSpeedTimeAndControlInBuffer(_buffer + 8, encoder, speed, duration, control);
		bool success = writeRegisters(MMX_SETPT_M1, 16);
		startMotorsInSync();
		return success;
	}

	// Or, just issue the command for one motor
	setEncoderSpeedTimeAndControlInBuffer(_buffer, encoder, speed, duration, control);
	uint8_t reg = (which_motors == MMX_Motor_1) ? MMX_SETPT_M1 : MMX_SETPT_M2;
	return writeRegisters(reg, 8);
}

// True when a motor has completed a timed move
bool NXTMMX::isTimeDone(uint8_t which_motors)
{
	if (which_motors == MMX_Motor_Both)
	{
		return isTimeDone(MMX_Motor_1) && isTimeDone(MMX_Motor_2);
	}
	uint8_t status = getMotorStatusByte(which_motors);
	return (status & MMX_CONTROL_TIME) == 0;
}

// waited until a timed command finishes
void NXTMMX::waitUntilTimeDone(uint8_t which_motors)
{
	delay(50); // this delay is required for the status byte to be available for reading.
	while (!isTimeDone(which_motors))
		delay(50);
}

// True when a command based on using the motor encoder completes
bool NXTMMX::isTachoDone(uint8_t which_motors)
{
	if (which_motors == MMX_Motor_Both)
	{
		return isTachoDone(MMX_Motor_1) && isTachoDone(MMX_Motor_2);
	}
	uint8_t status = getMotorStatusByte(which_motors);
	return (status & MMX_CONTROL_TACHO) == 0;
}

// waited until a turn-by-degrees command ends
void NXTMMX::waitUntilTachoDone(uint8_t which_motors)
{
	delay(50); // this delay is required for the status byte to be available for reading.
	while (!isTachoDone(which_motors))
		delay(50);
}

// Utility functions for motor control

// Take a speed and direction and give just a speed
inline int calcFinalSpeed(int initialSpeed, uint8_t direction)
{
	if (direction == MMX_Direction_Forward)
		return initialSpeed;
	return -initialSpeed;
}

// Calculate the bits that control what happens when this action finishes
inline uint8_t calcNextActionBits(uint8_t next_action)
{
	if (next_action == MMX_Next_Action_Brake)
		return MMX_CONTROL_BRK;
	else if (next_action == MMX_Next_Action_BrakeHold)
		return MMX_CONTROL_BRK | MMX_CONTROL_ON;
}

void NXTMMX::runUnlimited(
		uint8_t which_motors, // MMX_Motor_ 1, 2, or Both
		uint8_t direction,		// MMX_Direction_ Forward or Reverse
		int speed)						// in range [-100, +100]
{
	uint8_t ctrl = MMX_CONTROL_SPEED | MMX_CONTROL_GO;
	int sp = calcFinalSpeed(speed, direction);
	setSpeedTimeAndControl(which_motors, sp, 0, ctrl);
}

// runs the motors for a given number of seconds
void NXTMMX::runSeconds(
		uint8_t which_motors,				 // MMX_Motor_ 1, 2, or Both
		uint8_t direction,					 // MMX_Direction_ Forward or Reverse
		int speed,									 // [-100, +100]
		uint8_t duration,						 // in seconds
		uint8_t wait_for_completion, // MMX_Completion_ Wait_For or Dont_Wait
		uint8_t next_action)				 // MMX_Next_Action_ Brake, BrakeHold or Float
{
	uint8_t ctrl = MMX_CONTROL_SPEED | MMX_CONTROL_TIME | MMX_CONTROL_GO;
	ctrl |= calcNextActionBits(next_action);
	int sp = calcFinalSpeed(speed, direction);
	setSpeedTimeAndControl(which_motors, sp, duration, ctrl);

	if (wait_for_completion == MMX_Completion_Wait_For)
	{
		waitUntilTimeDone(which_motors);
	}
}

// runs the motors until the tachometer reaches a certain position
void NXTMMX::runTachometer(
		uint8_t which_motors,				 // MMX_Motor_ 1, 2, or Both
		uint8_t direction,					 // MMX_Direction_ Forward or Reverse
		int speed,									 // [-100, +100]
		long tachometer,						 // in degrees
		uint8_t relative,						 // MMX_Move_ Relative or Absolute
		uint8_t wait_for_completion, // MMX_Completion_ Wait_For or Dont_Wait
		uint8_t next_action)				 // MMX_Next_Action_ Brake, BrakeHold or Float
{
	uint8_t ctrl = MMX_CONTROL_SPEED | MMX_CONTROL_TACHO | MMX_CONTROL_GO;
	ctrl |= calcNextActionBits(next_action);
	int final_speed = calcFinalSpeed(speed, direction);

	// The tachometer can be absolute or relative.
	// If it is absolute, we ignore the direction parameter.
	long final_tach = tachometer;

	if (relative == MMX_Move_Relative)
	{
		ctrl |= MMX_CONTROL_RELATIVE;

		// a (relative) forward command is always a positive tachometer reading
		final_tach = abs(tachometer);
		if (final_speed < 0)
		{
			// and a (relative) reverse command is always negative
			final_tach = -final_tach;
		}
	}

	setEncoderSpeedTimeAndControl(which_motors, final_tach, final_speed, 0, ctrl);

	if (wait_for_completion == MMX_Completion_Wait_For)
	{
		waitUntilTachoDone(which_motors);
	}
}

// Turns the motors the specified number of degrees
void NXTMMX::runDegrees(
		uint8_t which_motors,				 // MMX_Motor_ 1, 2, or Both
		uint8_t direction,					 // MMX_Direction_ Forward or Reverse
		int speed,									 // [-100, +100]
		long degrees,								 // in degrees
		uint8_t wait_for_completion, // MMX_Completion_ Wait_For or Dont_Wait
		uint8_t next_action)				 // MMX_Next_Action_ Brake, BrakeHold or Float
{
	runTachometer(which_motors, direction, speed, degrees,
								MMX_Move_Relative, wait_for_completion, next_action);
}

// runs the motor(s) the specified number of rotations
void NXTMMX::runRotations(
		uint8_t which_motors,				 // MMX_Motor_ 1, 2, or Both
		uint8_t direction,					 // MMX_Direction_ Forward or Reverse
		int speed,									 // [-100, +100]
		long rotations,							 // number of full rotations of the motor
		uint8_t wait_for_completion, // MMX_Completion_ Wait_For or Dont_Wait
		uint8_t next_action)				 // MMX_Next_Action_ Brake, BrakeHold or Float
{
	runTachometer(which_motors, direction, speed, 360 * rotations,
								MMX_Move_Relative, wait_for_completion, next_action);
}

// The stop command will only stop the motor(s) by making them float/coast
// or brake.  Even if you specify MMX_Next_Action_BrakeHold, the motor
// will only brake, not hold.
bool NXTMMX::stop(uint8_t which_motors, uint8_t next_action)
{
	if (which_motors >= MMX_Motor_1 && which_motors <= MMX_Motor_Both)
	{
		// The magic variables become clear in the user's guide
		uint8_t base_code = (next_action != MMX_Next_Action_Float) ? 'A' - 1 : 'a' - 1;

		return issueCommand(base_code + which_motors);
	}

	setWriteErrorCode(5); // bad parameters
	return false;
}

/**
   This class interfaces with NXTServo attached to EVShield 
  */
NXTServo::NXTServo(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
}

uint8_t NXTServo::getBatteryVoltage()
{
	return readByte(Servo_Voltage);
}

uint8_t NXTServo::issueCommand(char command)
{
	return writeByte(Servo_Command, (uint8_t)command);
}

bool NXTServo::storeInitial(uint8_t number)
{
	issueCommand('I');
	return issueCommand(number);
}
bool NXTServo::reset()
{
	return issueCommand('S');
}
bool NXTServo::haltMacro()
{
	return issueCommand('H');
}

bool NXTServo::resumeMacro()
{
	return issueCommand('R');
}

bool NXTServo::gotoEEPROM(uint8_t position)
{
	issueCommand('G');
	issueCommand(position);
}

bool NXTServo::editMacro()
{
	issueCommand('E');
	return issueCommand('m');
}

bool NXTServo::pauseMacro()
{
	return issueCommand('P');
}
bool NXTServo::setSpeed(uint8_t number, uint8_t speed)
{
	return writeByte((uint8_t)number, (uint8_t)speed);
}

bool NXTServo::setPosition(uint8_t number, uint8_t position)
{
	return writeByte((uint8_t)number, (uint8_t)position);
}

void NXTServo::runServo(uint8_t number,		// Servo_1, 2, 3, ..., 8
												uint8_t position, // [500,2500] Servo_Default
												uint8_t speed)		// [0, inf] or Speed_Full
{
	if (number == Servo_1)
	{
		setPosition(Servo_Position_1, position);
		setSpeed(Servo_Speed_1, speed);
	}
	else if (number == Servo_2)
	{
		setPosition(Servo_Position_2, position);
		setSpeed(Servo_Speed_2, speed);
	}
	else if (number == Servo_3)
	{
		setPosition(Servo_Position_3, position);
		setSpeed(Servo_Speed_3, speed);
	}
	else if (number == Servo_4)
	{
		setPosition(Servo_Position_4, position);
		setSpeed(Servo_Speed_4, speed);
	}
	else if (number == Servo_5)
	{
		setPosition(Servo_Position_5, position);
		setSpeed(Servo_Speed_5, speed);
	}
	else if (number == Servo_6)
	{
		setPosition(Servo_Position_6, position);
		setSpeed(Servo_Speed_6, speed);
	}
	else if (number == Servo_7)
	{
		setPosition(Servo_Position_7, position);
		setSpeed(Servo_Speed_7, speed);
	}
	else if (number == Servo_8)
	{
		setPosition(Servo_Position_8, position);
		setSpeed(Servo_Speed_8, speed);
	}
}

/**
   This class interfaces with NXTThermometer attached to EVShield 
  */
NXTThermometer::NXTThermometer(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
}
void NXTThermometer::setMode(void)
{
	writeByte(0x01, 0x60);
}

float NXTThermometer::getTemperature()
{
	int temp = readInteger(0x00);

	byte b1 = temp & 0xff;
	byte b2 = 0xff & (temp >> 8);
	temp = (b1 << 4) | (b2 >> 4);
	return 0.0625 * (float)temp;
}

/**
  This class interfaces with NXTVoltMeter attached to EVShield 
	*/
VoltMeter::VoltMeter(uint8_t i2c_address)
		: EVShieldI2C(i2c_address)
{
}

uint8_t VoltMeter::issueCommand(char command)
{
	return writeByte(VM_Command, (uint8_t)command);
}

int VoltMeter::getAVoltage()
{
	return readInteger(ABSOLUTE_V);
}

int VoltMeter::getRVoltage()
{
	return readInteger(RELATIVE_V);
}

int VoltMeter::getReference()
{
	return readInteger(REFERENCE_V);
}

int VoltMeter::setReferenceV()
{
	return issueCommand('d');
}

//EV3Ultrasonic
bool EV3Ultrasonic::init(EVShield *shield, BankPort bp)
{
	EVShieldUART::init(shield, bp);
	EVShieldUART::setType(Type_EV3);
}

float EV3Ultrasonic::getDist()
{
	uint16_t l, m;
	float result;
	result = readValue();
	return (result / 10);
}

uint8_t EV3Ultrasonic::detect()
{
	uint16_t l, m;
	uint8_t result;
	readAndPrint(0x81 + m_offset, 10);
	result = readValue();
	return (result);
}

//EV3Color
bool EV3Color::init(EVShield *shield, BankPort bp)
{
	EVShieldUART::init(shield, bp);
	EVShieldUART::setType(Type_EV3);
}

float EV3Color::getVal()
{
	uint16_t l, m;
	float result;
	result = readValue();
	return (result);
}

//EV3Gyro
uint16_t ref = 0;

bool EV3Gyro::init(EVShield *shield, BankPort bp)
{
	EVShieldUART::init(shield, bp);
	EVShieldUART::setType(Type_EV3);
}

int EV3Gyro::getAngle()
{
	return readValue();
}

int EV3Gyro::getRefAngle()
{
	uint16_t refAngle = (readValue() - ref);
	return refAngle;
}

int EV3Gyro::setRef()
{
	ref = readValue();
}

//EV3Infrared
bool EV3Infrared::init(EVShield *shield, BankPort bp)
{
	EVShieldUART::init(shield, bp);
	EVShieldUART::setType(Type_EV3);
}

uint16_t EV3Infrared::readProximity()
{
	return readValue();
}

int8_t EV3Infrared::readChannelHeading(uint8_t channel)
{
	if (channel < 0 || channel > 3)
		return -1;
	return readLocationByte((0x81 + (channel * 2) + m_offset));
}

uint8_t EV3Infrared::readChannelProximity(uint8_t channel)
{
	if (channel < 0 || channel > 3)
		return -1;
	return readLocationByte((0x82 + (channel * 2) + m_offset));
}

uint8_t EV3Infrared::readChannelButton(uint8_t channel)
{
	if (channel < 0 || channel > 3)
		return -1;
	return readLocationByte((0x82 + (channel) + m_offset));
}

//EV3Touch
bool EV3Touch::init(EVShield *shield, BankPort bp)
{
	EVShieldUART::init(shield, bp);
	EVShieldUART::setType(Type_EV3_SWITCH);
}

bool EV3Touch::isPressed()
{
	return (readLocationByte(0x83 + m_offset));
}

int EV3Touch::getBumpCount()
{
	return (readLocationByte(0x84 + m_offset));
}

bool EV3Touch::resetBumpCount()
{
	return (writeLocation(0x84 + m_offset, 0));
}
