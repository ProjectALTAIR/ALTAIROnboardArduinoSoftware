/**************************************************************************/
/*!
    @file     ALTAIR_NEOM8N.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR NEO-M8N GPS receiver, located on the
    mast.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_NEOM8N.h"
#include <Wire.h>

/**************************************************************************/
/*!
 @brief  Get the GPS, and place the data in the _gps TinyGPSPlus data
         member.  Return true if successful.
*/
/**************************************************************************/
bool      ALTAIR_NEOM8N::getGPS(              )
{
    bool retval = false;

    Wire.beginTransmission( NEOM8N_I2CADDRESS );
    Wire.write(             NEOM8N_INITCODE   );
    Wire.endTransmission(                     );
    uint8_t i2cErr = Wire.requestFrom( NEOM8N_I2CADDRESS , NEOM8N_INITBYTES );
    if (i2cErr == 0) return false; // got some TWI error. Return

    uint16_t totalBytes = Wire.read() << 8;
    totalBytes         |= Wire.read();
    if (!totalBytes) return false; // GPS not ready to send data. Return
    Serial.print(F("GPS is ready to transfer ")); Serial.print(totalBytes, DEC); Serial.println(F(" bytes"));

    while (totalBytes) {
        uint8_t bytes2Read = totalBytes;
        if (totalBytes > NEOM8N_MAXBUFFERSIZE) bytes2Read = NEOM8N_MAXBUFFERSIZE;
//         Serial.print(bytes2Read);Serial.print("here5.5");
        Wire.beginTransmission(    NEOM8N_I2CADDRESS );
        Wire.write(                NEOM8N_GETGPSCODE );
        Wire.endTransmission(                        );
        i2cErr = Wire.requestFrom( (uint8_t) NEOM8N_I2CADDRESS , bytes2Read);
        if (i2cErr == 0) return false; // got some TWI error. Return
//         Serial.print("here6 ");
        for (uint8_t i = 0; i < bytes2Read; i++) {
            uint8_t theByte = Wire.read();
            if (theByte == NEOM8N_ERRORBYTE) return false; // got some TWI error. Return
//             retval = _gps.encode(theByte);
//             Serial.write(char(theByte));
            bool isEncoded = _gps.encode(theByte);
            retval |= isEncoded;
        }
        totalBytes -= bytes2Read;
//         delay(50);                    // this delay appears to fix mysterious occasional freezes in this while loop
    }

    return retval;
}
