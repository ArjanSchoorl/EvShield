/*
 * EVShield interface library for NXT & EV3 Sensors
*/

#ifndef EVLIB_H
#define EVLIB_H

#include "EVShield.h"

//NXTCam
#define Cam_Command 0x41
#define Cam_Number_Objects 0x42
#define Start_Reg 0x43

//NXTCurrentMeter
#define IM_Command 0x41
#define ABSOLUTE_I 0X43
#define RELATIVE_I 0x45
#define REFERENCE_I 0X47

//NXTMMX
// THE FOLLOWING CONSTANTS ARE ALL FROM THE NXC SOURCE CODE
#define MMX_CONTROL_SPEED 0x01
#define MMX_CONTROL_RAMP 0x02
#define MMX_CONTROL_RELATIVE 0x04
#define MMX_CONTROL_TACHO 0x08
#define MMX_CONTROL_BRK 0x10
#define MMX_CONTROL_ON 0x20
#define MMX_CONTROL_TIME 0x40
#define MMX_CONTROL_GO 0x80

#define MMX_COMMAND 0x41
#define MMX_VOLTAGE 0x41

#define MMX_SETPT_M1 0x42
#define MMX_SPEED_M1 0x46
#define MMX_TIME_M1 0x47
#define MMX_CMD_B_M1 0x48
#define MMX_CMD_A_M1 0x49

#define MMX_SETPT_M2 0x4A
#define MMX_SPEED_M2 0x4E
#define MMX_TIME_M2 0x4F
#define MMX_CMD_B_M2 0x50
#define MMX_CMD_A_M2 0x51

/*
 * Read registers.
 */
#define MMX_POSITION_M1 0x62
#define MMX_POSITION_M2 0x66
#define MMX_STATUS_M1 0x72
#define MMX_STATUS_M2 0x73
#define MMX_TASKS_M1 0x76
#define MMX_TASKS_M2 0x77

#define MMX_ENCODER_PID 0x7A
#define MMX_SPEED_PID 0x80
#define MMX_PASS_COUNT 0x86
#define MMX_TOLERANCE 0x87

/* constants to be used by user programs */
/*
 * Motor selection related constants
 */
#define MMX_Motor_1 0x01
#define MMX_Motor_2 0x02
#define MMX_Motor_Both 0x03

/*
 * Next action related constants
 */
// stop and let the motor coast.
#define MMX_Next_Action_Float 0x00
// apply brakes, and resist change to tachometer
#define MMX_Next_Action_Brake 0x01
// apply brakes, and restore externally forced change to tachometer
#define MMX_Next_Action_BrakeHold 0x02

/*
 * Direction related constants
 */
#define MMX_Direction_Forward 0x01
#define MMX_Direction_Reverse 0x00

/*
 * Tachometer related constants
 */
#define MMX_Move_Relative 0x01
#define MMX_Move_Absolute 0x00

#define MMX_Completion_Wait_For 0x01
#define MMX_Completion_Dont_Wait 0x00

/*
 * Speed constants, these are just convenience constants,
 * You can use any value between 0 and 100.
 */
#define MMX_Speed_Full 90
#define MMX_Speed_Medium 60
#define MMX_Speed_Slow 25

// NXTServo
#define Servo_Command 0x41
#define Servo_Voltage 0x41

#define Servo_1 1
#define Servo_2 2
#define Servo_3 3
#define Servo_4 4
#define Servo_5 5
#define Servo_6 6
#define Servo_7 7
#define Servo_8 8

#define Servo_Position_Default 1500
#define Servo_Speed_Full 0

#define Servo_Position_1 0x5A
#define Servo_Position_2 0x5B
#define Servo_Position_3 0x5C
#define Servo_Position_4 0x5D
#define Servo_Position_5 0x5E
#define Servo_Position_6 0x5F
#define Servo_Position_7 0x60
#define Servo_Position_8 0x61

#define Servo_Speed_1 0x52
#define Servo_Speed_2 0x53
#define Servo_Speed_3 0x54
#define Servo_Speed_4 0x55
#define Servo_Speed_5 0x56
#define Servo_Speed_6 0x57
#define Servo_Speed_7 0x58
#define Servo_Speed_8 0x59

//NXTThermometer
#define NXTThermometer_Command 0x41
#define NXTThermometer_Ambient_Temperature_C 0x42
#define NXTThermometer_Target_Temperature_C 0x44
#define NXTThermometer_Ambient_Temperature_F 0x46
#define NXTThermometer_Target_Temperature_F 0x48

//NXTVoltMeter
#define VM_Command 0x41
#define ABSOLUTE_V 0X43
#define RELATIVE_V 0x45
#define REFERENCE_V 0X47

//SensorMux
#define ESA_Command 0x41

//NXTCam
#define Cam_Command 0x41
#define Cam_Number_Objects 0x42
#define Start_Reg 0x43

/**
  This class interfaces with LEGO NXT Touch sensor attached to EVShield 
	*/
class NXTTouch : public EVShieldAGS
{
public:
  /** check if the touch sensor is pressed */
  bool init(EVShield *shield, BankPort bp);
  bool isPressed();
};

/** 
  This class interfaces with LEGO Light sensor attached to EVShield.
	*/
class NXTLight : public EVShieldAGS
{
public:
  /** initialize the NXTLight sensor with a pointer to the EVShield and the bank port it is connected to */
  bool init(EVShield *shield, BankPort bp);

  /** set the NXTLight sensor mode to reflected light mode (internal LED will be turned on) */
  bool setReflected();

  /** set the NXTLight sensor mode to ambient light mode (internal LED will be turned off) */
  bool setAmbient();
};

/**
  This class interfaces with Lego Color sensor attached to EVShield.
 */
class NXTColor
{
public:
  /** pointer to the EVShield class instantiation used */
  EVShield *mp_shield;

  /** bank port the device is connected to */
  BankPort m_bp;

  /** the data for uart sensors is stored in the bank, and 
          there is a offset based on port */
  int m_offset;

  /** null constructor for the class; need to init later */
  NXTColor();

  /** class constructor with pointer to EVShield and the bankport as a parameter; if object is instantiated using this constructor, init is not needed */
  NXTColor(EVShield *shield, BankPort bp);

  /** set the type of the device on this port of the EVShield */
  bool setType(uint8_t type);

  /** read the raw analog value from the device and return as an integer */
  //int   readRaw();

  /** initialize the analog device with a pointer to the EVShield and the bank port it is connected to */
  bool init(EVShield *shield, BankPort bp);

  uint8_t readValue();

  uint8_t readColor();
};

/**
   This class interfaces with NXTCam attached to EVShield 
  */
class NXTCam : public EVShieldI2C
{
public:
  /** class constructor for NXTCam with optional custom i2c address parameter */
  NXTCam(uint8_t i2c_address = 0x02);

  /** issue character command byte to the command register of the NXTCam 
   (command such as enable tracking, stop tracking, etc).  Refer to User Guide for available commands.
*/
  uint8_t issueCommand(char command);

  /** sort the blobs from the NXTCam byt blobs */
  bool sortSize();

  /** configure the tracking mode of the device to object tracking */
  bool selectObjectMode();

  /** write to the image registers of the CCD */
  bool writeImageRegisters();

  /** disable tracking for the NXTCam */
  bool disableTracking();

  /** enable tracking for the NXTCam */
  bool enableTracking();

  /** get the current colormap of the NXTCam */
  bool getColorMap();

  /** not impmemented yet */
  bool illuminationOn();

  /** read the image registers of the CCD */
  bool readImageRegisters();

  /** put the NXTCam in line tracking mode */
  bool selectLineMode();

  /** ping the NXTCam to test connection */
  bool pingCam();

  /** reset the NXTCam */
  bool resetCam();

  /** send ColorMap to the NXTCam */
  bool sendColorMap();

  /** not impmemented yet */
  bool illuminationOff();

  /** sort the blobs by color */
  bool sortColor();

  /** clear any selected sort configuration */
  bool sortNone();

  /** get the firmware of the NXTCam */
  bool camFirmware();

  /** get the total number of objected tracked by the NXTCam */
  int getNumberObjects();

  /**
This function gets the blob information of all the blobs that NXTCam is tracking.
There could be up to 8 blobs being tracked by NXTCam.
All the parameters of this function are return values.
This function will return color and coordinate information for all the blobs.
  @param nblobs      In this variable, number of blobs NXTCam sees are returned.
The blobs NXTCam sees could be of different color or same color.
For e.g. if you have 3 colors defined, say red (color 1), blue (color 2) and green (color3), and you have 3 red objects and 2 blue objects and 1 green object in front of NXTCam, nblobs will be 6.
color[]: a array of colors for the blobs
in above example, Color array will have 6 valid elements - three bytes will say 1 (for color1), and 2 bytes will say (for color 2), and 1 byte will say 3 (for color 3).
Based on sorting chosen, they will be sorted by size or color (if the sorting was color, first 3 bytes will be 1, next two will be 2 and next one will be 3).
  @param color      a array of colors for the blobs (array has 8 elements)
  @param left       the coordinate values of the blobs (array of 8 elements)
  @param top        the coordinate values of the blobs (array of 8 elements)
  @param right      the coordinate values of the blobs (array of 8 elements)
  @param bottom     the coordinate values of the blobs (array of 8 elements)
@return look at the nblobs first to see how many blobs are being tracked, and then read the respective color and coordinate information.
*/
  void getBlobs(int *nblobs, uint8_t *color, uint8_t *left, uint8_t *top, uint8_t *right, uint8_t *bottom);
};

/**
   This class interfaces with NXTCurrentMeter attached to EVShield 
  */
class CurrentMeter : public EVShieldI2C
{
public:
  /** Constructor for the class; may supply an optional custom i2c address 	*/
  CurrentMeter(uint8_t i2c_address = 0x28);
  /** Write a command byte at the command register of the device */
  uint8_t issueCommand(char command);
  /** Get the Absolute Current  
	 *  @return Absolute Current value*/
  int getACurrent();
  /** Get the Relative Current  
	 *  @return Relative Current value*/
  int getRCurrent();
  /** Get the Reference Current  
	 *  @return Reference Current value*/
  int getReference();
  /** Set the Reference Current to Absolute Current */
  int setReferenceI();
};

/**
   This class interfaces with NXTMMX attached to EVShield 
  */
class NXTMMX : public EVShieldI2C
{
public:
  /** constructor for NXTMMX
	*/
  NXTMMX(uint8_t i2c_address = 0x06);

  /** get the battery voltage for the MMX */
  uint8_t getBatteryVoltage();

  /** issue a command to this bank of the NXTMMX */
  uint8_t issueCommand(char command);

  /** set the target encoder position for the motor */
  bool setEncoderTarget(uint8_t which_motor, long target);

  /** get the target encoder position for the motor */
  long getEncoderTarget(uint8_t which_motor);

  /** set the speed of the motor */
  bool setSpeed(uint8_t which_motor, int speed);

  /** get the speed of the motor */
  int8_t getSpeed(uint8_t which_motor);

  /** set the time in seconds for which the motor should run for */
  bool getTimeToRun(uint8_t which_motor, int seconds);

  /** get the time in seconds that the motor is running for */
  uint8_t getTimeToRun(uint8_t which_motor);

  bool setCommandRegB(uint8_t which_motor, uint8_t value);
  uint8_t getCommandRegB(uint8_t which_motor);
  bool setCommandRegA(uint8_t which_motor, uint8_t value);
  uint8_t getCommandRegA(uint8_t which_motor);

  /** get the current encoder position of the motor in degrees */
  int32_t getEncoderPosition(uint8_t which_motor);

  /** get the current status of the motor */
  uint8_t getMotorStatusByte(uint8_t which_motor);

  /** get the tasks that are running on the specific motor */
  uint8_t getMotorTasksRunningByte(uint8_t which_motor);

  /** set the PID control for the encoders */
  bool setEncoderPID(uint16_t Kp, uint16_t Ki, uint16_t Kd);

  /** set the PID control for the speed of the motors */
  bool setSpeedPID(uint16_t Kp, uint16_t Ki, uint16_t Kd);

  /** set how many times the PID controller is allowed to oscillate about the set point */
  bool setPassCount(uint8_t pass_count);

  /** set how far away from the set point the PID controller is allowed to oscillate (amplitude) */
  bool setTolerance(uint8_t tolerance);

  /** reset all the set values for the motors */
  bool reset();

  /** start both motors at the same time to follow the set conditions */
  bool startMotorsInSync();

  /** reset the current encoder position to zero for the motor */
  bool resetEncoder(uint8_t which_motor);

  /** set the speed, duration to run, and control for the motor */
  bool setSpeedTimeAndControl(uint8_t which_motors, int speed, uint8_t duration, uint8_t control);

  /** set the ratget encoder position, speed, duration to run, and control for the motor */
  bool setEncoderSpeedTimeAndControl(uint8_t which_motors, long encoder, int speed, uint8_t duration, uint8_t control);

  /** validate if the motor has finished running for the set time duration */
  bool isTimeDone(uint8_t which_motors);

  /** wait until the motor has finished running for its set respective time duration */
  void waitUntilTimeDone(uint8_t which_motors);

  /** validate if the motor has reached its set target tachometer position */
  bool isTachoDone(uint8_t which_motors);

  /** wait until the motor has reached its set target tachometer position */
  void waitUntilTachoDone(uint8_t which_motors);

  /** run the motor endlessly at the desired speed in the desired direction */
  void runUnlimited(uint8_t which_motors, uint8_t direction, int speed);

  /** run the motor for a set duration at a set speed and do the next action */
  void runSeconds(uint8_t which_motors, uint8_t direction, int speed, uint8_t duration, uint8_t wait_for_completion, uint8_t next_action);

  /** run until the tachometer target has been reached and do next action */
  void runTachometer(uint8_t which_motors, uint8_t direction, int speed, long tachometer, uint8_t relative, uint8_t wait_for_completion, uint8_t next_action);

  /** run the motor for a set number of degrees and proceed to the next action */
  void runDegrees(uint8_t which_motors, uint8_t direction, int speed, long degrees, uint8_t wait_for_completion, uint8_t next_action);

  /** run the motor for a set number of complete rotations and proceed to the next action */
  void runRotations(uint8_t which_motors, uint8_t direction, int speed, long rotations, uint8_t wait_for_completion, uint8_t next_action);

  /** stop the motor and do an action */
  bool stop(uint8_t which_motors, uint8_t next_action);
};

/**
   This class interfaces with NXTServo attached to EVShield 
  */
class NXTServo : public EVShieldI2C
{
public:
  /** class constructor for the NXTServo with optional custom i2c address parameter */
  NXTServo(uint8_t i2c_address = 0xb0);

  /** issue a character command byte to the command register of the NXTServo */
  uint8_t issueCommand(char command);

  /** get the battery voltage supplied to the NXTServo */
  uint8_t getBatteryVoltage();

  /** store current settings of the given servo to initial default setting and remember when powered on */
  bool storeInitial(uint8_t number);

  /** reset all servos to default */
  bool reset();

  /** stop the onboard macro on the NXTServo */
  bool haltMacro();

  /** resume the onboard macro on the NXTServo */
  bool resumeMacro();

  /** Go to given EEPROM position (This command re-initializes the macro environment) */
  bool gotoEEPROM(uint8_t position);

  /** edit the onboard macro */
  bool editMacro();

  /** temporarily pause the running macro */
  bool pauseMacro();

  /** set the speed of a specified servo */
  bool setSpeed(uint8_t number, uint8_t speed);

  /** set the position of a specified servo */
  bool setPosition(uint8_t number, uint8_t position);

  /** run the specified to the specified position at the specified speed */
  void runServo(uint8_t number, uint8_t position, uint8_t speed);
};

/**
  This class interfaces with NXTThermometer attached to EVShield 
	*/
class NXTThermometer : public EVShieldI2C
{
public:
  /** device constructor for NXTThermometer; custom i2c address is an optional parameter */
  NXTThermometer(uint8_t i2c_address = 0x98);

  /** issue a byte command to the command register of the device */
  void setMode(void);

  /** get the Ambient Temperature  from the NXTThermometer in Celsius*/
  float getTemperature();
};

/**
  This class interfaces with NXTVoltMeter attached to EVShield 
	*/
class VoltMeter : public EVShieldI2C
{
public:
  /** Constructor for the class; may supply an optional custom i2c address 	*/
  VoltMeter(uint8_t i2c_address = 0x26);
  /** Write a command byte at the command register of the device */
  uint8_t issueCommand(char command);
  /** Get the Absolute Voltage  
	 *  @return Absolute Voltage value*/
  int getAVoltage();
  /** Get the Relative Voltage  
	 *  @return Relative Voltage value*/
  int getRVoltage();
  /** Get the Reference Voltage  
	 *  @return Reference Voltage value*/
  int getReference();
  /** Set the Reference Voltage to current Absolute Voltage */
  int setReferenceV();
};

/**
 * \enum MODE_Sonar, Modes supported by EV3 Sonar (Ultrasonic Sensor).
 */
typedef enum
{
  MODE_Sonar_CM = 0x00,       /*!< Choose for measurements in centimeters */
  MODE_Sonar_Inches = 0x01,   /*!< Choose measurements in inches */
  MODE_Sonar_Presence = 0x02, /*!< Choose to Listen for other ultrasonic devices */
} MODE_Sonar;

/**
  @brief This class interfaces with LEGO EV3 Ultrasonic sensor attached to EVShield 
	*/
class EV3Ultrasonic : public EVShieldUART
{
public:
  /** initialize the device and tell where it is connected */
  bool init(EVShield *shield, BankPort bp);

  /** get the distance to obstacle (in cm or inches based on the mode.)
        use setMode() to change the mode as you need
    */
  float getDist();

  /** detect other ultrasonic devices */
  uint8_t detect();
};

/**
 * \enum MODE_Color, Modes supported by EV3 Color (Color Sensor).
 */
typedef enum
{
  MODE_Color_ReflectedLight = 0x00, /*!< Choose for measuring reflected light */
  MODE_Color_AmbientLight = 0x01,   /*!< Choose for measuring ambient light */
  MODE_Color_DetectColor = 0x02,    /*!< Choose for measuring color*/
} MODE_Color;

/**
  @brief This class interfaces with LEGO EV3 Color sensor attached to EVShield 
	*/
class EV3Color : public EVShieldUART
{
public:
  /** initialize the device and tell where it is connected */
  bool init(EVShield *shield, BankPort bp);

  /** get the color value */
  float getVal();
};

/**
 * \enum MODE_Sonar, Modes supported by EV3 Sonar (Ultrasonic Sensor).
 */
typedef enum
{
  MODE_Gyro_Angle = 0x00, /*!< for Angle measurements */
  MODE_Gyro_Rate = 0x01   /*!< for rate of change measurement */

} MODE_Gyro;

/**
  @brief This class interfaces with LEGO EV3 Gyro sensor attached to EVShield 
	*/
class EV3Gyro : public EVShieldUART
{
public:
  bool init(EVShield *shield, BankPort bp);

  /** get the angle of the Gyro sensor */
  int getAngle();

  /** reset the angle to zero */
  int getRefAngle();

  /** reset the angle to zero */
  int setRef();
};

/**
 * \enum MODE_Infrared, Modes supported by EV3 Infrared Sensor.
 */
typedef enum
{
  MODE_Infrared_Proximity = 0x00, /*!< for measuring Proximity with EV3 Infrared sensor */
  MODE_Infrared_Beacon = 0x01,    /*!< for measuring in Mode Beacon (returns 8 bytes values - 2 per channel) */
  MODE_Infrared_Remote = 0x02     /*!< for Remote mode  */

} MODE_Infrared;

/**
  @brief This class interfaces with LEGO EV3 IR sensor attached to EVShield 
	*/
class EV3Infrared : public EVShieldUART
{
public:
  /** check if the touch sensor is pressed */
  bool init(EVShield *shield, BankPort bp);

  /** for mode MODE_Infrared_Proximity */
  uint16_t readProximity();

  /** in mode: MODE_Infrared_Beacon  and MODE_InfraRed_Proximity */
  int8_t readChannelHeading(uint8_t channel);

  /** in mode: MODE_InfraRed_Proximity */
  uint8_t readChannelProximity(uint8_t channel);

  /** in mode MODE_Infrared_Remote */
  uint8_t readChannelButton(uint8_t channel);
};

/**
  @brief This class interfaces with LEGO EV3 Touch sensor attached to EVShield 
	*/
class EV3Touch : public EVShieldUART
{
public:
  /** initialize the interface and tell the shield where the sensor is connected */
  bool init(EVShield *shield, BankPort bp);

  /** check if the touch sensor is pressed (or bumped) */
  bool isPressed();

  /** You can get bump count for EV3Touch Sensor (an incremental
     pressed value) this function will return the bump count since last reset.
     (The max value of bumpCount is 254, after that it will not increment).

     Programming Tip:
     If you don't want to wait to see if button is pressed, 
     use this bump count,
     store the old bumpCount in a variable and see if the new
     bumpCount is larger than the old value.
     */
  int getBumpCount();

  /** reset the bump count and start the incremental bumps from zero */
  bool resetBumpCount();
};

/**
 * This class interfaces with sensor attached to NXShield 
 */
class EV3SensorMux : public EVShieldI2C
{
public:
  /** Constructor for the class; may supply an optional custom i2c address 	*/
  EV3SensorMux(uint8_t i2c_address = 0x32);
  /** Write a command byte at the command register of the device */
  uint8_t issueCommand(char command);
  /** The EV3 sensors have different modes, you can change the mode of attached sensor with this function. To learn what all modes are available, refer to LEGO's documentation */
  uint8_t setMode(char newMode);
  /** it is possible to read back the mode that was set last time.
    use getMode to read the current mode */
  byte getMode();
  /** Read the value from the sensor attached to EV3SensorMux
    */
  int readValue();
};

#endif
