/**************************************************************************/
/*!
    @file     ALTAIR_GenTelInt.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the Generic Telemetry Interface base class for ALTAIR.
    Justin Albert  jalbert@uvic.ca     began on 8 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GenTelInt.h"
#include "ALTAIR_GlobalMotorControl.h"
#include "ALTAIR_GlobalDeviceControl.h"
#include "ALTAIR_GlobalLightControl.h"
#include <TinyGPS++.h>

/**************************************************************************/
/*!
 @brief  Constructor.  This will always be overridden (with the necessary 
         transceiver-specific code) in the derived classes.
*/
/**************************************************************************/
ALTAIR_GenTelInt::ALTAIR_GenTelInt() 
{
}

/**************************************************************************/
/*!
 @brief  Send GPS info.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendGPS(TinyGPSPlus& gps) 
{
    uint8_t hour      = gps.time.hour();
    uint8_t minute    = gps.time.minute();
    uint8_t second    = gps.time.second();
    int32_t latitude  = gps.location.lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t longitude = gps.location.lng() * 1000000;  // Longitude, in millionths of a degree.
    int16_t elevation = gps.altitude.meters();         // Elevation above mean sea level in meters.  NOTE: as this is signed 16 bit, this will *turn over*
                                                       // when above 32.77 km!  Would need to add & transmit an extra byte, if one preferred a higher limit.
                                                       // (Or make it unsigned, which would cause issues for launches from Death Valley or the Dead Sea. :)
                                                       // (One could get clever, and make it unsigned, but add, then later remove, 1000 meters, but I'm not 
                                                       //  that clever.)
    sendStart();
    send(0x0E);

    sendBareGPS(hour, minute, second, latitude, longitude, elevation);

    return send('T');
}

/**************************************************************************/
/*!
 @brief  Send every bit of ALTAIR info that is displayed in AIFCOMSS.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendAllALTAIRInfo( TinyGPSPlus&                gps           ,
                                          ALTAIR_GlobalMotorControl&  motorControl  ,
                                          ALTAIR_GlobalDeviceControl& deviceControl ,
                                          ALTAIR_GlobalLightControl&  lightControl   ) 
{
    uint8_t  hour      = gps.time.hour();
    uint8_t  minute    = gps.time.minute();
    uint8_t  second    = gps.time.second();
    int32_t  latitude  = gps.location.lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t  longitude = gps.location.lng() * 1000000;  // Longitude, in millionths of a degree.
    uint16_t age       = gps.location.age();            // Milliseconds since last GPS update (or default value USHRT_MAX if never received).
    int16_t  elevation = gps.altitude.meters();         // Elevation above mean sea level in meters.  NOTE: above in previous function.
    int8_t   hdop      = gps.hdop.value();              // Horizontal degree of precision.  A number typically between 1 and 50.

    int8_t   rssi      = lastRSSI();

    ALTAIR_OrientSensor* primaryOrientSensor = deviceControl.sitAwareSystem()->orientSensors()->primary();
    int16_t  accelZ    = primaryOrientSensor->accelZ();
    int16_t  accelX    = primaryOrientSensor->accelX();
    int16_t  accelY    = primaryOrientSensor->accelY();
    int16_t  yaw       = primaryOrientSensor->yaw();
    int16_t  pitch     = primaryOrientSensor->pitch();
    int16_t  roll      = primaryOrientSensor->roll();
    int8_t   oSensTemp = primaryOrientSensor->temperature();
    uint8_t  typeInfo  = primaryOrientSensor->typeAndHealth();

    ALTAIR_DataStorageSystem* sdCard         = deviceControl.dataStoreSystem();
    uint16_t occSpace  = sdCard->occupiedSpace();
    uint16_t freeSpace = sdCard->remainingSpace();

    sendStart();
    send(0x12);                                         // Number of bytes of data that will be sent.

    sendBareGPS(hour, minute, second, latitude, longitude, elevation);
    send(  byte(( age      >>  8)      & 0xFF));
    send(  byte(  age                  & 0xFF));
    send(  byte(  hdop                 & 0xFF));

    send(  byte(  rssi                 & 0xFF));



    return send('T');
}

/**************************************************************************/
/*!
 @brief  Send bare GPS info.  (Only use within a data packet wrapper 
         function!)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendBareGPS(uint8_t hour    , uint8_t minute   , uint8_t second   ,
                                   int32_t latitude, int32_t longitude, int16_t elevation)
{
  //send time
           send(byte(      hour        & 0xFF));
           send(byte(    minute        & 0xFF));
           send(byte(    second        & 0xFF));
  //send latitude
           send(byte(( latitude >> 24) & 0xFF));
           send(byte(( latitude >> 16) & 0xFF));
           send(byte(( latitude >>  8) & 0xFF));
           send(byte(  latitude        & 0xFF));
  //send longitude
           send(byte((longitude >> 24) & 0xFF));
           send(byte((longitude >> 16) & 0xFF));
           send(byte((longitude >>  8) & 0xFF));
           send(byte( longitude        & 0xFF));
  //send elevation
           send(byte((elevation >>  8) & 0xFF));
    return send(byte( elevation        & 0xFF));
}


/**************************************************************************/
/*!
 @brief  Read a command sent up from a ground station.
*/
/**************************************************************************/
byte* ALTAIR_GenTelInt::readALTAIRCommand()
{
    static byte term[MAX_TERM_LENGTH]    =    "";
    static int  termIndex                =     0;
    static int  termLength               =     0;
    int         hasBegun                 =     0;
    long        readTry                  =     0;
    if (!isBusy()) {
      Serial.print(F("Reading radio "));
      Serial.println(radioName());
      while (true) {
        while (!available() && readTry < MAX_READ_TRIES) {
          ++readTry;
//           delay(5);
        }
        if (readTry < MAX_READ_TRIES) {
          do {
            Serial.println(F("Reading a byte from radio"));

            byte b = read();
//             term[termIndex++] = b;
    
            if (b == (byte) RX_START_BYTE && hasBegun == 0) {
              hasBegun = 1;
            }
            else if (hasBegun == 1) {
              termLength = (int)b;
              hasBegun = 2;
            }
            else if (hasBegun == 2) {
            //Serial.println(b, HEX);
              term[termIndex++] = b;
            } 
            if (termLength == termIndex && termLength > 0) break;
//             if (termIndex == MAX_TERM_LENGTH) break;
          } while (available());
        
//           if (termIndex == MAX_TERM_LENGTH) break;
          if (termLength == termIndex && termLength > 0) break;
    
        } else {
          Serial.println(F("Radio is not available for reading"));
          break;
        }
      }
      if (termLength == termIndex && termLength > 0) {
        termLength = termIndex = hasBegun = 0;    
        return term;
      }
    } else {
      Serial.print(F("Cannot read commands from the ground station, since the"));
      Serial.print(radioName());
      Serial.println(F(" radio is busy"));
    }
    return (byte*) "";
}
