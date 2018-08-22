/*
 * EVShield interface library
 * Copyright (C) 2015 mindsensors.com
 *
 * This file is part of EVShield interface library.
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

#include "NXT.h"


/**
  This bool interfaces with LEGO Touch sensor attached to EVShield.
	*/
bool NXTTouch::init(EVShield * shield, BankPort bp)
{
    EVShieldAGS::init(shield, bp);
    EVShieldAGS::setType(Type_ANALOG);
}

bool NXTTouch::isPressed()
{
  int a;
	a = readRaw();

	if ( a < 300 ) return true;
	else return false;
}


/**
  This bool interfaces with LEGO Light sensor attached to EVShield.
	*/
bool NXTLight::init(EVShield * shield, BankPort bp)
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

NXTColor::NXTColor(EVShield * shield, BankPort bp)
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
    //setType(Type_COLORFULL);
}

bool NXTColor::init(EVShield * shield, BankPort bp)
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
    //setType(Type_COLORFULL);
    return true;
}

bool NXTColor::setType(uint8_t type)
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

uint8_t NXTColor::readValue()
{
    char str[100];
    uint8_t val;
    switch (m_bp) {
        case BAS1:
        case BAS2:
        val = (mp_shield->bank_a.readByte((0x70 + m_offset)));
        return (val*100)/255;
        case BBS1:
        case BBS2:
        val = (mp_shield->bank_b.readByte((0x70 + m_offset)));
        return (val*100)/255;
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
    switch (m_bp) {
        case BAS1:
        case BAS2:
        return (mp_shield->bank_a.readByte((0x70 + m_offset)));
        case BBS1:
        case BBS2:
        return (mp_shield->bank_b.readByte((0x70 + m_offset)));
    }
}
