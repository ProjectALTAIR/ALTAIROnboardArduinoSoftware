/**************************************************************************/
/*!
    @file     ALTAIR_ArduinoMicro.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the Arduino Micro I2C slave board, which reads
    and controls the 4 propulsion system RPM sensors, the 4 propulsion 
    system current sensors, and the 8 propulsion system temp sensors.

    Justin Albert  jalbert@uvic.ca     began on 18 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_ArduinoMicro.h"
#include <Wire.h>

/**************************************************************************/
/*!
 @brief  Constructor.
*/
/**************************************************************************/
ALTAIR_ArduinoMicro::ALTAIR_ArduinoMicro(                   ) :
               _dataLastObtainedAtMillis(               0   ) 
{
}

/**************************************************************************/
/*!
 @brief  Get the 16 packed data bytes over I2C from the physical Arduino 
         Micro, and store that data in this ALTAIR_ArduinoMicro object.
*/
/**************************************************************************/
void ALTAIR_ArduinoMicro::getDataAfterInterval(    long interval  )
{
  unsigned long currentMillis = millis();
  if (currentMillis - _dataLastObtainedAtMillis > interval) { 
    _dataLastObtainedAtMillis = currentMillis;

    Wire.requestFrom( ARDUINOMICRO_I2CADDRESS , ARDUINOMICRO_DATABYTES );
    for (int i = 0; i < 4; ++i) _packedRPM[i]     = Wire.read();
    for (int i = 0; i < 4; ++i) _packedCurrent[i] = Wire.read();
    for (int i = 0; i < 8; ++i) _packedTemp[i]    = Wire.read();
  }
}
