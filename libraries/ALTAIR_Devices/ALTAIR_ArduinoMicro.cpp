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
    /* Read in as unsigned shorts (2 bytes per value) */
    //for (int i = 0; i < TEMP_SENSOR_COUNT; ++i)     {_packedTemp[i]     = Wire.read(); _packedTemp[i]    |= (Wire.read() <<8);}
    //for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i)  {_packedCurrent[i]  = Wire.read(); _packedCurrent[i] |= (Wire.read() <<8);}
    //for (int i = 0; i < RPM_SENSOR_COUNT; ++i)      {_packedRPM[i]      = Wire.read(); _packedRPM[i]     |= (Wire.read() <<8);}
 
    /* Read in as bytes (1 byte per value) */
    for (int i = 0; i < TEMP_SENSOR_COUNT; ++i)     _packedTemp_byte[i]    = Wire.read();
    for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i)  _packedCurrent_byte[i] = Wire.read();
    for (int i = 0; i < RPM_SENSOR_COUNT; ++i)      _packedRPS_byte[i]     = Wire.read();

    /* Print the received unsigned shorts */
    //for (int i = 0; i < TEMP_SENSOR_COUNT; ++i) { Serial.print("Packed Temperature["); Serial.print(i); Serial.print("]    = "); Serial.println(_packedTemp[i]) ; }
    //for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i) { Serial.print("Packed Current["); Serial.print(i); Serial.print("]    = "); Serial.println(_packedCurrent[i]) ; }
    //for (int i = 0; i < RPM_SENSOR_COUNT; ++i) { Serial.print("Packed RPM["); Serial.print(i); Serial.print("]    = "); Serial.println(_packedRPM[i]) ; }

    /* Print the received bytes as int values */
    for (int i = 0; i < TEMP_SENSOR_COUNT; ++i) { Serial.print("Packed Temperature["); Serial.print(i); Serial.print("]    = "); Serial.println(int(_packedTemp_byte[i])) ; }
    for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i) { Serial.print("Packed Current["); Serial.print(i); Serial.print("]    = "); Serial.println(int(_packedCurrent_byte[i])) ; }
    for (int i = 0; i < RPM_SENSOR_COUNT; ++i) { Serial.print("Packed RPS["); Serial.print(i); Serial.print("]    = "); Serial.println(int(_packedRPS_byte[i])) ; }

    /* Convert unsigned shorts to useful values */
    //for (int i = 0; i < TEMP_SENSOR_COUNT; ++i) Temp[i] = static_cast<float>(_packedTemp[i])/2  - 273.15;        // Rescaling in conversion in °C
    //for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i) Current[i] = static_cast<float>(_packedCurrent[i])/1000;    // Rescaling from mikroA to milliA
    //for (int i = 0; i < RPM_SENSOR_COUNT; ++i) RPM[i] = int(_packedRPM[i]) ;                                    // No Rescaling needed, already in RPM

    /* Convert bytes to useful values */
    for (int i = 0; i < TEMP_SENSOR_COUNT; ++i)  if(_packedTemp_byte[i]<=127){Temp[i] = _packedTemp_byte[i];}else{Temp[i]=_packedTemp_byte[i]-256;};        // Remapping onto positive/negative °C
    for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i)  Current[i] = static_cast<float>(_packedCurrent_byte[i])/10;    // Rescaling from deciA to milliA
    for (int i = 0; i < RPM_SENSOR_COUNT; ++i)      RPM[i] = int(_packedRPS_byte[i])*60 ;                       // Rescaling from RPS to RPM

    /* Print the useful sensor values */
    for (int i = 0; i < TEMP_SENSOR_COUNT; ++i) { Serial.print("Temperature["); Serial.print(i); Serial.print("]    = "); Serial.println(Temp[i]) ; }
    for (int i = 0; i < CURRENT_SENSOR_COUNT; ++i) { Serial.print("Current["); Serial.print(i); Serial.print("] = "); Serial.println(Current[i]) ; }
    for (int i = 0; i < RPM_SENSOR_COUNT; ++i) { Serial.print("RPM["); Serial.print(i); Serial.print("]     = "); Serial.println(RPM[i]) ; }

  }
}
