// EVShield.h

#ifndef EVShield_H
#define EVShield_H

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// SHDefines Library
#if defined(__AVR__)
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__) 
      #define MODEL_EVSHIELD_D
  #else
      #define MODEL_EVSHIELD_M
  #endif

#elif defined(__PIC32MX__)

  #if defined(_BOARD_UNO_) 
      #define MODEL_EVSHIELD_D
  #elif defined(_BOARD_MEGA_)
      #define MODEL_EVSHIELD_M
  #endif

#endif
#if defined(ARDUINO_ARC32_TOOLS)
  #define MODEL_EVSHIELD_D
#endif


/**
  \enum BankPort for the sensor bank ports
*/
typedef enum {
  BAS1 = 0x01,  /*!<  Bank A Sensor Port 1 */
  BAS2 = 0x02,  /*!<  Bank A Sensor Port 2 */
  BBS1 = 0x03,  /*!<  Bank B Sensor Port 1 */
  BBS2 = 0x04   /*!<  Bank B Sensor Port 2 */
} BankPort;

//! Protocols supported by EVShield.
/**
  \enum Protocols Protocol enums - to initialize your EVShield with appropriate protocol.
*/
typedef enum {
  HardwareI2C  = 0,  /*!< It's best to use hardware i2c as it is faster, (but it does not work with Ultrasonic Sensor). */
  SoftwareI2C,  /*!< Software I2C is slower, and designed to work with Ultrasonic sensor.  */
} Protocols;

#if defined(MODEL_EVSHIELD_D)
  // Arduino Duemilanove, Uno
  #define SCL_BAS1  A5
  #define SDA_BAS1  A4
  #define SCL_BAS2  2
  #define SDA_BAS2  A0
  #define SCL_BBS1  4
  #define SDA_BBS1  A1
  #define SCL_BBS2  7
  #define SDA_BBS2  A2

  // deepak
  #define BTN_RIGHT  4
  #define BTN_LEFT  1
  // deepak end

  #define BTN_GO  2
  #define LED_RED  8
  #define LED_GREEN  A3
  #define LED_BLUE  12

#else
  // Arduino mega, 2560
  #define SCL_BAS1  21
  #define SDA_BAS1  20
  #define SCL_BAS2  19
  #define SDA_BAS2  A13
  #define SCL_BBS1  17
  #define SDA_BBS1  A14
  #define SCL_BBS2  18
  #define SDA_BBS2  A15

  #define BTN_LEFT  16
  #define BTN_GO  15
  #define BTN_RIGHT 14
  #define LED_RED  16
  #define LED_GREEN  15
  #define LED_BLUE  14
  
#endif

// delay used to tweek signals
#define I2C_DELAY_USEC 30

// R/W direction bit to OR with address for start or restart
#define I2C_READ 1
#define I2C_WRITE 0

// Motor control related constants.
#define CONTROL_SPEED      0x01
#define CONTROL_RAMP       0x02
#define CONTROL_RELATIVE   0x04
#define CONTROL_TACHO      0x08
#define CONTROL_BRK        0x10
#define CONTROL_ON         0x20
#define CONTROL_TIME       0x40
#define CONTROL_GO         0x80

#define STATUS_SPEED       0x01
#define STATUS_RAMP        0x02
#define STATUS_MOVING      0x04
#define STATUS_TACHO       0x08
#define STATUS_BREAK       0x10
#define STATUS_OVERLOAD    0x20
#define STATUS_TIME        0x40
#define STATUS_STALL       0x80

#define COMMAND     0x41
#define VOLTAGE     0x6E

#define SETPT_M1     0x42
#define SPEED_M1     0x46
#define TIME_M1      0x47
#define CMD_B_M1     0x48
#define CMD_A_M1     0x49

#define SETPT_M2     0x4A
#define SPEED_M2     0x4E
#define TIME_M2      0x4F
#define CMD_B_M2     0x50
#define CMD_A_M2     0x51

/*
 * Read registers.
 */
#define POSITION_M1  0x52
#define POSITION_M2  0x56
#define STATUS_M1    0x5A
#define STATUS_M2    0x5B
#define TASKS_M1     0x5C
#define TASKS_M2     0x5D

#define ENCODER_PID  0x5E
#define SPEED_PID  0x64
#define PASS_COUNT  0x6A
#define TOLERANCE  0x6B

#define S1_MODE  0x6F
#define S1_EV3_MODE  0x6F
#define S1_ANALOG   0x70

#define S2_MODE  0xA3
#define S2_EV3_MODE  0x6F
#define S2_ANALOG   0xA4

#define BTN_PRESS     0xDA
#define RGB_LED     0xD7
#define CENTER_RGB_LED     0xDE

#define PS_TS_X  0xE3
#define PS_TS_Y  0xE5
#define PS_TS_RAWX  0xE7
#define PS_TS_RAWY  0xE9
#define PS_TS_CALIBRATION_DATA_READY 0x70
#define PS_TS_CALIBRATION_DATA 0x71
#define PS_TS_SAVE 0x77
#define PS_TS_LOAD 0x6C
#define PS_TS_UNLOCK 0x45

/* constants to be used by user programs */
/**
 * \enum Motor Motor selection related constants
 */
typedef enum {
  Motor_1     = 0x01,   /*!< Choose Motor 1 for selected operation */
  Motor_2     = 0x02,   /*!< Choose Motor 2 for selected operation  */
  Motor_Both  = 0x03    /*!< Choose Both Motors for selected operation */
} Motor;

/*
 * \enum Next_Action Next action related constants
 */
typedef enum {
  Next_Action_Float  = 0x00, /*!< stop and let the motor coast. */
  Next_Action_Brake = 0x01, /*!< apply brakes, and resist change to tachometer, but if tach position is forcibly changed, do not restore position */
  Next_Action_BrakeHold = 0x02 /*!< apply brakes, and restore externally forced change to tachometer */
} Next_Action;

/**
 * \enum Direction Motor direction related constants.
 */
typedef enum {
  Reverse = 0x00,   /*!< Run motor in reverse direction */
  Forward = 0x01   /*!< Run motor in forward direction */
} Direction;

/*
 * \enum Move Tachometer movement related constants
 */
typedef enum {
  Move_Absolute = 0x00,   /*!< Move the tach to absolute value provided */
  Move_Relative = 0x01   /*!< Move the tach relative to previous position */
} Move;

/*
 * \enum Completion_Wait Whether to wait for motor to finish it's current task or not
 */
typedef enum {
  Completion_Dont_Wait    = 0x00,  /*!< Don't wait for motor to finish, program will continue with next function */
  Completion_Wait_For     = 0x01  /*!< Wait for motor to finish, program will wait until current function finishes it's operation */
} Completion_Wait;

/*
 * Speed constants, these are just convenience constants,
 * You can use any value between 0 and 100.
 */
#define Speed_Full 90
#define Speed_Medium 60
#define Speed_Slow 25

/*
 * EVShield has two banks, and each of them has different I2C address.
 * each bank has 2 motors and 2 sensors.
 *
 */
/*!
  \def Bank_A
	I2C address of bank A
*/
#define Bank_A 0x34
/*!
  \def Bank_B
	I2C address of bank B
*/
#define Bank_B 0x36

/*
 *  Sensor type primitives
 *
 */

 /*!
  \def Type_NONE
	In this type the sensor port is not defined and used.
*/
#define Type_NONE   0x00

#define Type_SWITCH    0x01
 
/*!
  \def Type_ANALOG
	In this type the sensor port is not powered (for sensors like touch sensor).
*/
#define Type_ANALOG   0x02

/*!
  \def Type_LIGHT_REFLECTED
	Used for detection of Refelected Light from Light sensors, 
*/
#define Type_LIGHT_REFLECTED      0x03
/*!
  \def Type_LIGHT_AMBIENT
	Used for detection of ambient Light using the light sensors, 
*/
#define Type_LIGHT_AMBIENT      0x04
/*!
  \def Type_I2C
	In this type the sensor connected should be an i2c device.
*/
#define Type_I2C                 0x09

/*!
  \def Type_COLORFULL 
	In this type the sensor connected is NXT color sensor in color mode
*/
#define Type_COLORFULL            13
/*!
  \def Type_COLORRED   
	In this type the sensor connected is NXT color sensor in RED color mode
*/
#define Type_COLORRED             14
/*!
  \def Type_COLORGREEN     
	In this type the sensor connected is NXT color sensor in GREEN color mode
*/
#define Type_COLORGREEN           15
/*!
  \def Type_COLORBLUE 
	In this type the sensor connected is NXT color sensor in BLUE color mode
*/
#define Type_COLORBLUE            16
/*!
  \def Type_COLORNONE
	In this type the sensor connected is NXT color sensor in ambient light mode
*/
#define Type_COLORNONE            17
/*!
  \def Type_EV3_SWITCH 
	In this type the sensor connected is EV3 touch sensor
*/
#define Type_EV3_SWITCH           18
/*!
  \def Type_EV3
	In this type the sensor connected is EV3 UART sensor
*/
#define Type_EV3                  19

/*
 * Sensor defines.
 */
/*!
  \def S1
	This is used internally to address Sensor Port 1.
	Do not use this in sketches.
	Sketches should use BankPort enums.
*/
#define S1   1
/*!
  \def S2
	This is used internally to address Sensor Port 2.
	Do not use this in sketches.
	Sketches should use BankPort enums.
*/
#define S2   2

#if defined(__AVR__)
  #include <avr/io.h>
  #include <avr/interrupt.h>
#endif

/** parse the two bytes in the buffer into an integer */
inline uint16_t readIntFromBuffer(uint8_t* buf)
{
	return buf[0] | (buf[1] << 8);
}

/** parse the four bytes in the buffer into an integer of type long */
inline uint32_t readLongFromBuffer(uint8_t* buf)
{
    /* typecasts added to make it compatible with 1.6.8 */
	return (uint32_t)buf[0] |
           (((uint32_t)buf[1]) << 8) |
           (((uint32_t)buf[2]) << 16) |
           (((uint32_t)buf[3]) << 24);
}

/** write the data as a byte to the supplied buffer */
inline void writeByteToBuffer(uint8_t* buf, uint8_t data)
{
	buf[0] = data;
}

inline void writeByteToBuffer(uint8_t* buf, int8_t data)
{
	writeByteToBuffer(buf, (uint8_t)data);
}

/** write the two byte integer to the supplied buffer */
inline void writeIntToBuffer(uint8_t* buf, uint16_t data)
{
	buf[0] = data & 0xFF;
	buf[1] = (data >> 8) & 0xFF;
}

inline void writeIntToBuffer(uint8_t* buf, int16_t data)
{
	writeIntToBuffer(buf, (uint16_t)data);
}

/** write the four byte integer of type long to the supplied buffer */
inline void writeLongToBuffer(uint8_t* buf, uint32_t data)
{
	buf[0] = data & 0xFF;
	buf[1] = (data >>  8) & 0xFF;
	buf[2] = (data >> 16) & 0xFF;
	buf[3] = (data >> 24) & 0xFF;
}

inline void writeLongToBuffer(uint8_t* buf, int32_t data)
{
	writeLongToBuffer(buf, (uint32_t)data);
}

/**
  @brief This class implements hardware I2C protocol used by EVShield/NXShield on an Arduino
	*/
class BaseI2CDevice
{
	// Note that this class is a base class, but not an abstract base class
	// Feel free to instantiate BaseI2CDevice.
	
public:
	/** constructor for the BaseI2C Device class; requires the i2c address of the device */
	BaseI2CDevice(uint8_t i2c_address);

	/** initialize hardware i2c using the Wire.h library */
	void initProtocol();
	
	/** read specified number of bytes from the start register.
	 @param start_register location to start reading from
	 @param bytes_to_read Number of bytes to read (max 16 for LEGO compatible devices)
	 @param buffer (optional) buffer to read the data into
	 @param buffer_length (optional) length of the buffer if it was provided
	 @param clear_buffer (optional) to clear the buffer or not before using.
	 @return pointer to data buffer that was read. If buffer was not provided, this is internal pointer.
	*/
	uint8_t* 	readRegisters	(uint8_t start_register, uint8_t bytes_to_read,
								uint8_t* buffer = 0, uint8_t buffer_length = 0, bool clear_buffer = false);

	/** Read a byte from specified location
	 @param location address to read at
	 @return  a byte value read from the location
	*/
	uint8_t  	readByte	(uint8_t location);

	/** Read an integer from specified location. Integer comprises of 2 bytes.
	 @param location address to read at
	 @return  an integer value read from the location
	*/
	int16_t  	readInteger	(uint8_t location);

	/** Read a long from specified location. Long comprises of 4 bytes.
	 @param location address to read at
	 @return  a long value read from the location
	*/
	uint32_t  	readLong	(uint8_t location);

	/** Read a string from specified location
	 @param location address to read at
	 @param bytes_to_read  number of bytes to read
	 @param buffer optional, a buffer to read the data into.
	 @param buffer_length optional, length of the buffer supplied.
	 @return  a char array read from the location
	*/
	char* 		readString	(uint8_t  location, uint8_t  bytes_to_read,
							 uint8_t* buffer = 0, uint8_t  buffer_length = 0);


	/** write data bytes to the i2c device starting from the start register
	@param start_register location to write at.
	@param bytes_to_write Number of bytes to write
	@param buffer (optional) data buffer, if not supplied, data from internal buffer is used.
	*/
	bool 		writeRegisters	(uint8_t start_register, uint8_t bytes_to_write,
								uint8_t* buffer = 0);

	/** write one byte to the specified register location
	@param location location to write to.
	@param data the data to write.
	*/
	bool 		writeByte	(uint8_t location, uint8_t data);
	
	/** write two bytes (int) to the specified register location
	@param location location to write to.
	@param data the data to write.
	*/
	bool 		writeInteger(uint8_t location, uint16_t data);
	
	/** write four bytes (long) to the specified register location 
	@param location location to write to.
	@param data the data to write.
	*/
	bool 		writeLong	(uint8_t location, uint32_t data);

	/** validate if a device is attached to the i2c bus with the specified i2c address */
	bool checkAddress();
	
	/** set the i2c address for this device 
	@param i2c_address new device address.
	*/
	bool setAddress(uint8_t i2c_address);
	
	/** returns the current address for this instance of BaseI2CDevice */
	uint8_t getAddress();

	/** returns the error code for an error with the Wire.h library on the i2c bus */
	uint8_t		getWriteErrorCode();

	/** return the firware version id of the device */
	char*		getFirmwareVersion();
	
	/** return the name of the vendor for the device */
	char*		getVendorID();
	
	/** get the name of the device */
	char*		getDeviceID();
	
	/** returns the features on the device, not supported by all devices */
	char*		getFeatureSet();

	/** Buffer used for data that is returned from I2C commands
	*/
	static uint8_t* _buffer;

	static bool b_initialized;

protected:
	/** write the internal error code
	*/
	void		setWriteErrorCode(uint8_t code);

private:
	uint8_t _device_address;	// I2C address of the I2C device
	uint8_t _write_error_code;	// Error code from last write
};

/**
  @brief This class implements software i2c interface used by EVShield/NXShield on Arduino
	*/
class SoftI2cMaster {

	bool initialized;
private:
	uint8_t sclPin_;
	uint8_t sdaPin_;
	uint8_t deviceAddr;
	uint8_t _error_code;	// Error code 

public:
	/** internal buffer */
	uint8_t* _so_buffer;
	
	/** issue a start condition for i2c address with read/write bit */
	uint8_t start(uint8_t addressRW);

	/** issue a stop condition */
	void stop(void);
	
	/** issue stop condition, pull down scl, and start again */
	uint8_t restart(uint8_t addressRW);
	
	/** write byte and return true for Ack or false for Nak */
	uint8_t write(uint8_t b);

	/** read a byte and send Ack if last is false else Nak to terminate read */
	uint8_t read(uint8_t last);

	/** class constructor supplies the device address */
	SoftI2cMaster(uint8_t devAddr);
	
	/** init bus custom scl and sda pins are optional */
	void initProtocol(uint8_t sclPin = (uint8_t)NULL, uint8_t sdaPin = (uint8_t)NULL);
	
	/** read number of bytes from start register and return values; optional buffer */
	uint8_t* readRegisters(uint8_t startRegister, uint8_t bytes, uint8_t* buf = NULL);
	
	/** write number of bytes from buffer */
	bool writeRegistersWithLocation(int bytes, uint8_t* buf);
	
	/** write bytes starting at the specified register location */
  bool     writeRegisters  (uint8_t location, uint8_t bytes_to_write,
                uint8_t* buffer = 0);
  
	/** write one byte starting at the specified register location */
	bool     writeByte  (uint8_t location, uint8_t data);
	
	/** write integer starting at the specified register location */
  bool     writeInteger(uint8_t location, uint16_t data);
	
	/** write integer type long starting at the specified register location */
  bool     writeLong  (uint8_t location, uint32_t data);

	/** read specified number of bytes starting at the startRegister */
	char* readString(uint8_t startRegister, uint8_t bytes, uint8_t* buf = NULL, uint8_t len=0);
	
	/** read one byte starting at the location */
	uint8_t readByte	(uint8_t location);
	
	/** read two bytes and parse as an integer starting at the location */
	int16_t readInteger	(uint8_t location);
	
	/** read and parse as integer type long at the location */
	uint32_t readLong	(uint8_t location);

	/** get the version of the firmware */
	char*		getFirmwareVersion();
	
	/** get the name of the vendor */
	char*		getVendorID();
	
	/** get the name of the device */
	char*		getDeviceID();
	
	/** Get error of last i2c operation */
	uint8_t getWriteErrorCode();

	bool checkAddress();

	/** set the i2c address for this device 
	@param address new device address.
	*/
	bool setAddress(uint8_t address);
};

/**
  This class implements I2C interfaces used by EVShield.
	*/
class EVShieldI2C : public BaseI2CDevice, public SoftI2cMaster
{
public:
	/** Pointer to the EVShield
	*/
  void * mp_shield;
	/** Pointer to internal i2c buffer
	*/
	uint8_t *_i2c_buffer;

public:
	/** Class constructor for the EVShieldI2C; derived from both BaseI2CDevice and SoftI2cMaster; i2c address must be passed as a parameter */
  EVShieldI2C(uint8_t i2c_address);
  
	/** global variable of the i2c protocol used */
	uint8_t m_protocol;

	/** initialized this i2c address with a pointer to the EVShield and the bankport it is connected to */
	void init(void * shield, BankPort bp);
  
	/** Read a byte from specified location
	 @param location address to read at
	 @return  a byte value read from the location
	*/
  uint8_t  readByte  (uint8_t location);
  
	/** Read an integer from specified location. Integer comprises of 2 bytes.
	 @param location address to read at
	 @return  an integer value read from the location
	*/
	uint16_t readInteger  (uint8_t location);
	
	/** Read a long from specified location. Long comprises of 4 bytes.
	 @param location address to read at
	 @return  a long value read from the location
	*/
    uint32_t readLong  (uint8_t location);
	
	/** read the specified number of bytes from the buffer starting from the specified start register 
	 @param start_register location to start reading from
	 @param bytes Number of bytes to read
	 @param buf buffer to read the data into
	 @return the character array that was read.
	*/
	uint8_t*  readRegisters  (uint8_t  start_register, uint8_t  bytes, uint8_t* buf);

	/** Read a string from specified location
	 @param location address to read at
	 @param bytes_to_read  number of bytes to read
	 @param buffer optional, a buffer to read the data into.
	 @param buffer_length optional, length of the buffer supplied.
	 @return  a char array read from the location
	*/
  char*    readString  (uint8_t  location, uint8_t  bytes_to_read,
               uint8_t* buffer = 0, uint8_t  buffer_length = 0);

	/** write data bytes to the i2c device starting from the start register
	@param start_register location to write at.
	@param bytes_to_write Number of bytes to write
	@param buffer (optional) data buffer, if not supplied, data from internal buffer is used.
	*/
  bool     writeRegisters  (uint8_t start_register, uint8_t bytes_to_write,
                uint8_t* buffer = 0);

	/** write one byte to the specified register location
	@param location location to write to.
	@param data the data to write.
	*/
  bool     writeByte  (uint8_t location, uint8_t data);
  
	/** write two bytes (int) to the specified register location
	@param location location to write to.
	@param data the data to write.
	*/
	bool     writeInteger(uint8_t location, uint16_t data);
  
	/** write four bytes (long) to the specified register location 
	@param location location to write to.
	@param data the data to write.
	*/
	bool     writeLong  (uint8_t location, uint32_t data);

	/** get the firmware version of the device */
	char*		getFirmwareVersion();
	
	/** get the name of the vendor of the device */
	char*		getVendorID();
	
	/** get the name of the device */
	char*		getDeviceID();
	
	/** get the features the device is capable of; only supported by some devices */
	char*		getFeatureSet();

	/** get the error code of last i2c operation */
	uint8_t getErrorCode();

	bool checkAddress();

	/** set the i2c address for this device 
	@param address new device address.
	*/
    bool setAddress(uint8_t address);

};

/**
  @brief This class defines methods for the EVShield Bank(s).
  */
class EVShieldBank : public EVShieldI2C
{
public:
  /** Constructor for bank a of the EVShield device */
  EVShieldBank(uint8_t i2c_address = Bank_A);
  
  /** Get the battery voltage (milli-volts) for this bank of the EVShield
    @return voltage value in milli-volts 
    The voltage reported by this function is actual voltage at VIN pin on Arduino
    This will be lower than your supply voltage due to drops at various points in the circuit.
    The drop will be different based on where the power source is connected.
    (i.e. source through EVShield Green connector Vs Arduino black adapater Vs Arduino USB.)
    */
  int  evshieldGetBatteryVoltage();

  /** nxshieldGetBatteryVoltage() is provided for backword compatibility with nxshield programs.
   */
  int  nxshieldGetBatteryVoltage();

  /** 
  Issue a command to this bank of the EVShield
  @param command Refer to user guide for list of commands.
  */
  uint8_t  EVShieldIssueCommand(char command);

  //
  //  Motor Operation APIs.
  //
  /** Set the target encoder position for the motor
    @param which_motor    Provide which motor to operate on
    @param target         Encode value to achieve
  */
  bool     motorSetEncoderTarget(Motor which_motor, long target);
  
  /** 
  Get the target encoder position for the motor
    @param which_motor    Provide which motor to operate on
    @return long encoder value that the motor is trying to achieve.
  */
  long     motorGetEncoderTarget(Motor which_motor);
  
  /** 
  Set the speed of the motor
    @param which_motor    Provide which motor to operate on
    @param speed          The speed value between 0 and 100
  */
  bool     motorSetSpeed(Motor which_motor, int speed);
  
  /** 
  Get the speed of the motor
    @param which_motor    Provide which motor to operate on
    @return  the speed value set to the motor
  */
  int8_t   motorGetSpeed(Motor which_motor);
  
  /** 
  Set the time in seconds for which the motor should run for
    @param which_motor    Provide which motor to operate on
    @param seconds          The time duration the motor should run
  */
  bool     motorSetTimeToRun(Motor which_motor, int seconds);
  
  /** 
  Get the time in seconds that the motor is running for
    @param which_motor    Provide which motor to operate on
    @return  time the motor has been running since last start.
  */
  uint8_t  motorGetTimeToRun(Motor which_motor);
  
  /**
    Set the Command Register B
    There are two command registers, A and B.

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1

    @param which_motor    Provide which motor to operate on
    @param value       The command register value to set
  */
  bool     motorSetCommandRegB(Motor which_motor, uint8_t value);
  /**
    Get the command register B

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1
    @param which_motor    Provide which motor to operate on
    @return the last set command register value.
  */
  uint8_t  motorGetCommandRegB(Motor which_motor);
  /**
    Set the Command Register A
    There are two command registers, A and B.

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1
    @param which_motor    Provide which motor to operate on
    @param value       The command register value to set
    */
  bool     motorSetCommandRegA(Motor which_motor, uint8_t value);
  /**
    Get the command register A

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1
    @param which_motor    Provide which motor to operate on
    @return the last set command register value.
  */
  uint8_t  motorGetCommandRegA(Motor which_motor);
  
  /** 
  Get the current encoder position of the motor in degrees
    @param which_motor    Provide which motor to operate on
    @return              current encoder value
  */
  int32_t  motorGetEncoderPosition(Motor which_motor);
  
  /** 
  Get the current status of the motor
    @param which_motor    Provide which motor to operate on
    @return  The current status of the motor.
    This is a byte with various bits set based on motor's state.
    Refer to User Guide for details of bits.
  */  
  uint8_t  motorGetStatusByte(Motor which_motor);
  
  /** 
  Get the tasks that are running on the specific motor
    @param which_motor    Provide which motor to operate on
    @return  The task byte that's currently running for this motor.
    (Currently only one task is supported.)
  */
  uint8_t  motorGetTasksRunningByte(Motor which_motor);
  
  /** 
  Set the PID control factors for the encoders
  All motors on this bank will use the same PID values.
    @param Kp The proportionate factor of the PID.
    @param Ki The integreal factor of the PID.
    @param Kd The differential factor of the PID.
  */
  bool     motorSetEncoderPID(uint16_t Kp, uint16_t Ki, uint16_t Kd);
  
  /** 
  Set the PID control factors for the speed of the motors
  All motors on this bank will use the same PID values.
    @param Kp The proportionate factor of the PID.
    @param Ki The integreal factor of the PID.
    @param Kd The differential factor of the PID.
  */
  bool     motorSetSpeedPID(uint16_t Kp, uint16_t Ki, uint16_t Kd);
  
  bool centerLedSetRGB(uint8_t R, uint8_t G, uint8_t B);

  // Set the RGBLED that shows RGB color

  bool    ledSetRGB(uint8_t R, uint8_t G, uint8_t B);
  
  /** 
  Set how many times the PID controller is allowed to oscillate at the set point
  Depending on your situation of load and power characteristics, your PID algorithm
  may oscillate indefinitly trying to achieve it's target.
  To prevent that from happening there is a limit set.
    @param pass_count the maximum number of times the PID is allowed to cross it's target.
  */
  bool     motorSetPassCount(uint8_t pass_count);
  
  /** 
  Set how far away from the set point the PID controller is allowed to oscillate (amplitude)
  Depending on your situation of load and power characteristics, your PID algorithm
  may oscillate above or below the target.
    @param tolerance the maximum amplititude allowed.
  */
  bool     motorSetTolerance(uint8_t tolerance);
  
  /** 
  Reset all the set values for the motors
  Applies to all motors on this bank.
  */
  bool     motorReset();
  
  /** 
  Start both motors at the same time to follow the set conditions
  This will execute the commands specified in the command register on both motors at once.
  */
  bool     motorStartBothInSync();
  
  /** 
  Reset the current encoder position to zero for the motor
    @param which_motor    Provide which motor to operate on
  */
  bool     motorResetEncoder(Motor which_motor);
  
  /**
  Set the speed, duration to run, and control for the motor through register A (or B)

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1
    @param which_motors    Provide which motor(s) to operate on
    @param speed          Speed value between 0 and 100
    @param duration       time to run in seconds
    @param control        command register value
  */
  bool     motorSetSpeedTimeAndControl(Motor which_motors, int speed,
                                      uint8_t duration, uint8_t control);

  /** 
 This function sets the speed, the number of seconds, and
 the control (a.k.a. command register A)

		For more information on what register does, please refer to 'Motor Command Register Explained' section of EVShield-Advanced-Development-Guide.pdf from following url:
		http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1
    @param which_motors    Provide which motor(s) to operate on
    @param encoder        Target encoder position to achieve
    @param speed          Speed value between 0 and 100
    @param duration       time to run in seconds
    @param control        command register value
  */
  bool     motorSetEncoderSpeedTimeAndControl(Motor which_motors,
                                      long encoder, int speed,
                                      uint8_t duration, uint8_t control);
  
  /**
  Validate if the motor has finished running for the set time duration
    @param which_motors    Provide which motor(s) to operate on
    @return                0 when motor(s) has completed a timed move properly,
    If the return value is non-zero, either motor has not finished yet or has encountered an error condition.
  */
  uint8_t     motorIsTimeDone(Motor which_motors);
  
  /**
  Wait until the motor has finished running for its set respective time duration
    @param which_motors    Provide which motor(s) to operate on
    @return                function waits until when motor(s) has stopped, returns 0 if the set goal was achieved.
    If the return value is non-zero, you should check for error condition such as stall.
  */
  uint8_t     motorWaitUntilTimeDone(Motor which_motors);
  
  /**
  Validate if the motor has reached its set target tachometer position
    @param which_motors    Provide which motor(s) to operate on
    @return                0 when motor(s) has completed a encoder based move properly,
    If the return value is non-zero, either motor has not finished yet or has encountered an error condition.
  */
  uint8_t     motorIsTachoDone(Motor which_motors);
  
  /** 
  Wait until the motor has reached its set target tachometer position
    @param which_motors    Provide which motor(s) to operate on
    @return                function waits until when motor(s) has stopped, returns 0 if the set goal was achieved.
    If the return value is non-zero, you should check for error condition such as stall.
  */
  uint8_t     motorWaitUntilTachoDone(Motor which_motors);
  
  /**
  Run the motor endlessly at the desired speed in the desired direction
   @param which_motors     specifiy the motor(s) to operate on
   @param direction        specifiy the direction to run the motor
   @param speed            the speed value (between 0 and 100)
   @return  Starts the motors and function returns immediately
  */
  void     motorRun(Motor which_motors, Direction direction,
                                      int speed);
                                      
  /** Run the motor for a set duration at a set speed and do the next action
   @param which_motors     specifiy the motor(s) to operate on
   @param direction        specifiy the direction to run the motor
   @param speed            the speed value (between 0 and 100)
   @param duration         in seconds
   @param wait_for_completion    whether this API should wait for completion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t     motorRunSeconds(Motor which_motors, Direction direction,
                                      int speed, uint8_t duration,
                                      Completion_Wait wait_for_completion,
                                      Next_Action next_action);
                                      
  /**
  run until the tachometer target has been reached and do next action
   @param which_motors     specifiy the motor(s) to operate on
   @param direction        specifiy the direction to run the motor
   @param speed            the speed value (between 0 and 100)
   @param tachometer       the target for the encoder value to achieve.
   @param relative         is the tachometer relative or absolute.
   @param wait_for_completion    whether this API should wait for completion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t     motorRunTachometer(Motor which_motors, Direction direction,
                                      int speed, long tachometer,
                                      Move relative,
                                      Completion_Wait wait_for_completion,
                                      Next_Action next_action);
                                      
  /**
  Run the motor for a set number of degrees and proceed to the next action
   @param which_motors     specifiy the motor(s) to operate on
   @param direction        specifiy the direction to run the motor
   @param speed            the speed value (between 0 and 100)
   @param degrees          The degrees the motor should turn through
   @param wait_for_completion    whether this API should wait for completion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t     motorRunDegrees(Motor which_motors, Direction direction,
                                      int  speed, long degrees,
                                      Completion_Wait wait_for_completion,
                                      Next_Action next_action);
                                      
  /**
  Run the motor for a set number of complete rotations and proceed to the next action
   @param which_motors     specifiy the motor(s) to operate on
   @param direction        specifiy the direction to run the motor
   @param speed            the speed value (between 0 and 100)
   @param rotations        The rotations the motor should rotate through
   @param wait_for_completion    whether this API should wait for completion or not
   @param next_action      for these motor being operated on
   @return        0 if the operation was finished satisfactorily,
            in case return value is non-zero you should check for the bits for error conditions.
  */
  uint8_t     motorRunRotations(Motor which_motors, Direction direction,
                                      int speed, long rotations,
                                      Completion_Wait wait_for_completion,
                                      Next_Action next_action);
                                      
  /**
  stop the motor and do the next action
   @param which_motors     specifiy the motor(s) to operate on
   @param next_action      for these motor being operated on
  */
  bool     motorStop(Motor which_motors, Next_Action next_action);


  //
  // EVShield sensor functions.
  //
public:
  /**
  Set the sensor type of the sensor on this bank
  @param  which_sensor the sensor to set the type to.
  @param  sensor_type     type value of the sensor,
  refer to Advanced User Guide for available values of sensor types.
  */
  bool     sensorSetType(uint8_t which_sensor, uint8_t sensor_type);
  
  /**
  Read the raw analog value from the sensor and return as an int
  @param  which_sensor the sensor to read the raw value from
  @return   raw value from the sensor
  */
  int      sensorReadRaw(uint8_t which_sensor);

};

/**
  @brief EVShield has two banks. Bank B has few differences from Bank A.
  This class defines overriding methods for the EVShield Bank B.
  */
class EVShieldBankB : public EVShieldBank
{
private:

public:
  /** constructor for bank be of the EVShield; optional custom i2c address can be supplied */
  EVShieldBankB(uint8_t i2c_address_b = Bank_B);
  /**
  Read the raw analog value from the sensor and return as an int
  @param  which_sensor the sensor to read the raw value from
  @return   raw value from the sensor
  */
  int      sensorReadRaw(uint8_t which_sensor);

  /**
  Set the sensor Type of the sensor on this bank
  @param  which_sensor the sensor to set the type to.
  @param  sensor_type     type value of the sensor,
  refer to Advanced User Guide for available values of sensor types.
  */
  bool     sensorSetType(uint8_t which_sensor, uint8_t sensor_type);
};


/**
  @brief This class defines methods to access EVShield features
*/
class EVShield
{
public:
  /**
  Global variable representing the i2c protocol to use; whether software or hardware
  */
  uint8_t m_protocol;
  /** Variable for the bank_a of EVShield
  */
  EVShieldBank   bank_a;
  /** Variable for the bank_b of EVShield
  */
  EVShieldBankB  bank_b;

  /** class constructor for EVShield; optional custom i2c addresses may be supplied for both banks */
  EVShield(uint8_t i2c_address_a = Bank_A,
           uint8_t i2c_address_b = Bank_B);
  
  /**
  the initialization of the EVShield; 
	This function initializes the LED related timers, and communication protocols.
  @param protocol optional, specify the i2c protocol to use for the EVShield and highspeed i2c port
  */
  void init(Protocols protocol=HardwareI2C);

  /**
  the initialization of the EVShield LED timers.
  */
	void initLEDTimers();

  /**
    the initialization of the EVShield I2C timer.
  */
	void I2CTimer();
    
  /**
  the initialization of EVShield communication protocols.
  @param protocol optional, specify the i2c protocol to use for the EVShield and highspeed i2c port
  */
	void initProtocols(Protocols protocol=HardwareI2C);

  
  /**
  Get the button state of the specific button on EVShield.<br>
  When using the Wi-Fi Arduino Interface for PiStorms, there is only a GO button.
  The PiStorms does not have a left or right button like the EVShield. In this case,
  the F1 software button will be BTN_LEFT, and F2 will be BTN_RIGHT. If a program
  asks you to press the left button, instead tap the stylus in the F1 area on screen.
  @param btn      Button to get state for (BTN_GO, BTN_LEFT, BTN_RIGHT)
  @return true or false for specified button on the EVShield 
  */
  bool getButtonState(uint8_t btn);
  
  /** 
  Wait inside function until specified button is pressed on EVShield (BTN_GO, BTN_LEFT, BTN_RIGHT)
  @param btn      Button to get state for (BTN_GO, BTN_LEFT, BTN_RIGHT)
  @param led_pattern   0 for LED off.
  1 to brighten/lighten LED with breathing pattern (default).
  2 to brighten/lighten LED with heart beat pattern.
  */
  void waitForButtonPress(uint8_t btn, uint8_t led_pattern=1);
  

  /**
  Set the colors of LED on the EVShield;
  The values of red, green, blue are between 0 to 255.
  @param red      Intensity for red color (between 0 and 255)
  @param green      Intensity for green color (between 0 and 255)
  @param blue      Intensity for blue color (between 0 and 255)
  */
  void ledSetRGB(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0);

  /**
    The LED is brightened and dimmed in a breathing pattern.
    Call this function repeatedly to make the pattern.
  */
  void ledBreathingPattern();

  /**
    The LED is brightened and dimmed in a HeartBeat pattern.
    Call this function repeatedly to make the pattern.
  */
  void ledHeartBeatPattern();
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  Read the touchscreen press and write the coordinates to the output parameters.
  @param x x-value of touchscreen press is written to this variable
  @param y y-value of touchscreen press is written to this variable
  */
  void getTouchscreenValues(uint16_t *x, uint16_t *y);
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  Reads the x-coordinate of the touchscreen press
  */
  uint16_t TS_X();
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  Reads the y-coordinate of the touchscreen press
  */
  uint16_t TS_Y();
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  Detect touchscreen presses and prevents false positives.
  */
  bool isTouched();
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  returns true if the specified area of the screen is being touched
  */
  bool checkButton(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
  
  /**
  Wi-Fi Arduino Interface for PiStorms only!<br>
  returns 0 if none of the software buttons are touched, or 1-4 if one of them is.
  */
  uint8_t getFunctionButton();

private:
  /** touchscreen calibration values */
  uint16_t x1, y1, x2, y2, x3, y3, x4, y4;
  
  /** read the raw x-coordinate of the touchscreen press */
  uint16_t RAW_X();
  
  /** read the raw x-coordinate of the touchscreen press */
  uint16_t RAW_Y();
  
  bool useOldTouchscreen = false;
  
  /** get raw touchscreen values, do some math using the calibration values, and write to the output parameters
    @param x x-value of touchscreen press is written to this variable
    @param y y-value of touchscreen press is written to this variable
  */
  void getReading(uint16_t *x, uint16_t *y);
};

/** This function formats an integer in binary format.
  @param i 8 bit integer value
  @param s returned string of the binary representation
  */
extern bool format_bin(uint8_t i, char *s);

/**
	EVShield Analog Sensor class.
  */
class EVShieldAGS
{
public:
	/** pointer to the EVShield class instantiation used */
	EVShield * mp_shield;
	
	/** bank port the analog device is connected to */
	BankPort m_bp;
	
	/** null constructor for the EVShieldAGS class; need to init later */
  EVShieldAGS();
	
	/** class constructor with pointed to EVShield and the bankport as a parameter; init is not needed */
  EVShieldAGS(EVShield * shield, BankPort bp);
	
	/** set the type of the device on this port of the EVShield */
  bool  setType(uint8_t type);
	
	/** read the raw analog value from the device and return as an integer */
  int   readRaw();
	
	/** initialize the analog device with a pointed to the EVShield and the bank port it is connected to */
	bool init(EVShield * shield, BankPort bp);

};

/**
  @brief EVShield UART Sensor class.
  also provides support for the EV3 Touch Sensor
 */
class EVShieldUART
{
    public:
        /** pointer to the EVShield class instantiation used */
        EVShield * mp_shield;

        /** bank port the analog device is connected to */
        BankPort m_bp;

        /** the data for uart sensors is stored in the bank, and 
          there is a offset based on port */
        int m_offset;

        /** null constructor for the EVShieldUART class; need to init later */
        EVShieldUART();

        /** class constructor with pointer to EVShield and the bankport as a parameter; init is not needed */
        EVShieldUART(EVShield * shield, BankPort bp);

        /** get the mode of the sensor */
        uint8_t	getMode( );

        /** When the device is initially connected (or type is changed) it takes a while for the sensor to negotiate UART communication with host and be ready to provide readings. This funciton will return True if the sensor is ready, False if it is not ready*/
        bool isDeviceReady();

        /** set the type of the device on this port of the EVShield */
        bool  setType(uint8_t type);

        /**  write a byte at the given location (selects appropriate bank) */
        bool writeLocation(uint8_t loc, uint8_t data);

        /** read integer value from specificed location */
        int16_t readLocationInt(uint8_t loc);

        /** read the value from the device at given location and return as an integer */
        uint8_t readLocationByte(uint8_t loc);

        /** initialize the analog device with a pointed to the EVShield and the bank port it is connected to */
        bool init(EVShield * shield, BankPort bp);

        /** set mode of the sensor */
        uint8_t  setMode(char newMode);

        /** read sensor reading */
        uint16_t	readValue();

        /**  internal function to examine the buffer */
        bool	readAndPrint(uint8_t loc, uint8_t len);
};

#endif
