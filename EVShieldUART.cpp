
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

  Modification history:
  Date      Author      reason
  12/24/14  Deepak      Initial development
*/

#include "EVShieldUART.h"

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
