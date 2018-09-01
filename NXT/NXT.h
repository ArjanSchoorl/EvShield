/*
 * EVShield interface library for NXT Sensors
*/

#ifndef NXT_H
#define NXT_H

#include "EVShield.h"

/**
  This class interfaces with LEGO NXT Touch sensor attached to EVShield 
	*/
class NXTTouch : public EVShieldAGS {
public:
	/** check if the touch sensor is pressed */
    bool init(EVShield * shield, BankPort bp);
    bool isPressed();
};


/** 
  This class interfaces with LEGO Light sensor attached to EVShield.
	*/
class NXTLight : public EVShieldAGS {
public:
	/** initialize the NXTLight sensor with a pointer to the EVShield and the bank port it is connected to */
	bool init(EVShield * shield, BankPort bp);

	/** set the NXTLight sensor mode to reflected light mode (internal LED will be turned on) */
	bool setReflected();
	
	/** set the NXTLight sensor mode to ambient light mode (internal LED will be turned off) */
	bool setAmbient();
};

/**
  This class interfaces with Lego Color sensor attached to EVShield.
 */
class NXTColor {
    public:
        /** pointer to the EVShield class instantiation used */
        EVShield * mp_shield;

        /** bank port the device is connected to */
        BankPort m_bp;

        /** the data for uart sensors is stored in the bank, and 
          there is a offset based on port */
        int m_offset;

        /** null constructor for the class; need to init later */
        NXTColor();

        /** class constructor with pointer to EVShield and the bankport as a parameter; if object is instantiated using this constructor, init is not needed */
        NXTColor(EVShield * shield, BankPort bp);

        /** set the type of the device on this port of the EVShield */
        bool  setType(uint8_t type);

        /** read the raw analog value from the device and return as an integer */
        //int   readRaw();

        /** initialize the analog device with a pointer to the EVShield and the bank port it is connected to */
        bool init(EVShield * shield, BankPort bp);

        uint8_t readValue();
        
        uint8_t readColor();
        
};

#endif
