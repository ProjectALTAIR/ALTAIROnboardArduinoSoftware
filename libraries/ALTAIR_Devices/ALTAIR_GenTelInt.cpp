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
#include "ALTAIR_GPSSensor.h"
#include "ALTAIR_GlobalMotorControl.h"
#include "ALTAIR_GlobalDeviceControl.h"
#include "ALTAIR_GlobalLightControl.h"
#include "ALTAIR_ArduinoMicro.h"
#include <Adafruit_BME280.h>


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
bool ALTAIR_GenTelInt::sendGPS(ALTAIR_GPSSensor* gps) 
{
    uint8_t hour      = gps->hour();
    uint8_t minute    = gps->minute();
    uint8_t second    = gps->second();
    int32_t latitude  = gps->lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t longitude = gps->lon() * 1000000;  // Longitude, in millionths of a degree.
    int16_t elevation = gps->ele();            // Elevation above mean sea level in meters.  NOTE: as this is signed 16 bit, this will *turn over*
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
bool ALTAIR_GenTelInt::sendAllALTAIRInfo( ALTAIR_GlobalMotorControl&  motorControl  ,
                                          ALTAIR_GlobalDeviceControl& deviceControl ,
                                          ALTAIR_GlobalLightControl&  lightControl   ) 
{
    byte     sendString1[50];
    byte     sendString2[50];

    ALTAIR_GPSSensor* gps = deviceControl.sitAwareSystem()->gpsSensors()->primary();

    int32_t  latitude     = gps->lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t  longitude    = gps->lon() * 1000000;  // Longitude, in millionths of a degree.
    uint16_t age          = gps->age();            // Milliseconds since last GPS update (or default value USHRT_MAX if never received).
    int16_t  elevation    = gps->ele();            // Elevation above mean sea level in meters.  NOTE: above in previous function.
    int8_t   hdop         = gps->hdop();           // Horizontal degree of precision.  A number typically between 1 and 50.

    uint16_t outPres   = (deviceControl.sitAwareSystem()->bmeMast()->readPressure()    / 2.0F) ; // in units of 2 Pa (fits nicely into a uint16_t)
    int8_t   outTemp   =  deviceControl.sitAwareSystem()->bmeMast()->readTemperature()         ; // in degrees C
    uint8_t  outHum    =  deviceControl.sitAwareSystem()->bmeMast()->readHumidity()            ; // in %
    uint16_t inPres    = (deviceControl.sitAwareSystem()->bmePayload()->readPressure() / 2.0F) ; // in units of 2 Pa (fits nicely into a uint16_t)
    int8_t   inTemp    =  deviceControl.sitAwareSystem()->bmePayload()->readTemperature()      ; // in degrees C
    uint8_t  inHum     =  deviceControl.sitAwareSystem()->bmePayload()->readHumidity()         ; // in %
// If the connector up to the balloon valve is unconnected, or gets pulled out on the fly (by a cutdown), the internal balloon values below will read as all zeros
    uint16_t balPres   = (deviceControl.sitAwareSystem()->bmeBalloon()->readPressure() / 2.0F) ; // in units of 2 Pa (fits nicely into a uint16_t)
    int8_t   balTemp   =  deviceControl.sitAwareSystem()->bmeBalloon()->readTemperature()      ; // in degrees C
    uint8_t  balHum    =  deviceControl.sitAwareSystem()->bmeBalloon()->readHumidity()         ; // in %

    ALTAIR_OrientSensor* primaryOrientSensor = deviceControl.sitAwareSystem()->orientSensors()->primary();
    primaryOrientSensor->update();
    uint8_t rollUInt8   = primaryOrientSensor->rollUInt8();
    uint8_t pitchUInt8  = primaryOrientSensor->pitchUInt8();
    uint8_t yawUInt8    = primaryOrientSensor->yawUInt8();
    int8_t  oSensTemp   = primaryOrientSensor->temperature();
    uint8_t typeInfo    = primaryOrientSensor->typeAndHealth() + (8 * gps->typeAndHealth()) + (32 * radioType());
    uint8_t accelXUInt8 = primaryOrientSensor->accelXUInt8();
    uint8_t accelYUInt8 = primaryOrientSensor->accelYUInt8();
    uint8_t accelZUInt8 = primaryOrientSensor->accelZUInt8();

    Serial.print("   GPS sensor type & health = "); Serial.println(gps->typeAndHealth(), HEX) ;
    Serial.print("   GPS latitude = ");    Serial.println(gps->lat())   ;
    Serial.print("   GPS longitude = ");   Serial.println(gps->lon())   ;
    Serial.print("   GPS elevation = ");   Serial.println(gps->ele())   ;
    Serial.print("   GPS sensor time = "); Serial.println(gps->time())  ;

    int8_t*  packedRPM = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedRPM();
    int8_t*  packedCur = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedCurrent();

    sendString1[0]  = (unsigned char)  TX_START_BYTE;
    sendString1[1]  = (unsigned char)  (0x2B);           // Number of bytes of data that will be sent (0x2B = 43).

    sendString1[2]  = byte((latitude  >> 24) & 0xFF);
    sendString1[3]  = byte((latitude  >> 16) & 0xFF);
    sendString1[4]  = byte((latitude  >>  8) & 0xFF);
    sendString1[5]  = byte( latitude         & 0xFF);

    sendString1[6]  = byte((longitude >> 24) & 0xFF);
    sendString1[7]  = byte((longitude >> 16) & 0xFF);
    sendString1[8]  = byte((longitude >>  8) & 0xFF);
    sendString1[9]  = byte( longitude        & 0xFF);

    sendString1[10] = byte((elevation >>  8) & 0xFF);
    sendString1[11] = byte( elevation        & 0xFF);

    sendString1[12] = byte((age       >>  8) & 0xFF);    // only send the upper byte of age (i.e., age / 256)
    sendString1[13] = byte( hdop             & 0xFF);

    sendString1[14] =       'T'                     ;

    sendString1[15] = byte((outPres   >>  8) & 0xFF);
    sendString1[16] = byte( outPres          & 0xFF);
    sendString1[17] = byte( outTemp          & 0xFF);
    sendString1[18] = byte( outHum           & 0xFF);
    sendString1[19] = byte((inPres    >>  8) & 0xFF);
    sendString1[20] = byte( inPres           & 0xFF);
    sendString1[21] = byte( inTemp           & 0xFF);
    sendString1[22] = byte( inHum            & 0xFF);
    sendString1[23] = byte((balPres   >>  8) & 0xFF);
    sendString1[24] = byte( balPres          & 0xFF);
    sendString1[25] = byte( balTemp          & 0xFF);
    sendString1[26] = byte( balHum           & 0xFF);

    sendString1[27] = byte( accelZUInt8      & 0xFF);
    sendString1[28] = byte( accelXUInt8      & 0xFF);
    sendString1[29] = byte( accelYUInt8      & 0xFF);

    sendString1[30] =       'T'                     ;

    sendString1[31] = byte(   yawUInt8       & 0xFF);
    sendString1[32] = byte( pitchUInt8       & 0xFF);
    sendString1[33] = byte(  rollUInt8       & 0xFF);
    sendString1[34] = byte( oSensTemp        & 0xFF);
    sendString1[35] = byte( typeInfo         & 0xFF);

    for (int i = 0; i < 4; ++i)     sendString1[36+i]  =  byte(  packedRPM[i]          & 0xFF);
    for (int i = 0; i < 4; ++i)     sendString1[40+i]  =  byte(  packedCur[i]          & 0xFF);

    sendString1[44] =       'T'                     ;

//    if (send(sendString1, 45)) Serial.println(F("Successfully sent sendString1"));
    if ((radioType() != rfm23bp) || (lastSentString2())) {
        send(sendString1, 45);
        if (radioType() == rfm23bp) return true;
    }

// try moving work here (instead of a CPU-cycle-wasting delay)

    int8_t*  packedTem = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedTemp();    // in units of 0.5 degrees C! (e.g 0x2B = 21.5 degrees C)

    int8_t   rssi      = lastRSSI();
    uint8_t  bat1V     = (deviceControl.sitAwareSystem()->genOpsBatt()->readVoltage() * 18.18F) ; // in units of 55 mV (would turn over at 14 V)
    uint8_t  bat2V     = (deviceControl.sitAwareSystem()->propBatt()->readVoltage()   * 18.18F) ; // in units of 55 mV (would turn over at 14 V)

    ALTAIR_DataStorageSystem* sdCard =  deviceControl.dataStoreSystem();
    uint16_t occSpace  =      sdCard->occupiedSpace();

    uint8_t  powerMot1 = (motorControl.propSystem()->portOuterMotor()->powerSetting() * 10.0F) ; // an integer containing 10x the present power setting
    uint8_t  powerMot2 = (motorControl.propSystem()->portInnerMotor()->powerSetting() * 10.0F) ; // an integer containing 10x the present power setting
    uint8_t  powerMot3 = (motorControl.propSystem()->stbdInnerMotor()->powerSetting() * 10.0F) ; // an integer containing 10x the present power setting
    uint8_t  powerMot4 = (motorControl.propSystem()->stbdOuterMotor()->powerSetting() * 10.0F) ; // an integer containing 10x the present power setting

    uint8_t  axlRotSet = (motorControl.propSystem()->axleRotServo()->reportSetting()  * 10.0F) ; // an integer containing 10x the present servo setting
    uint8_t  axlRotAng = (motorControl.propSystem()->axleRotServo()->reportPosition() * 50.0F) ; // in units of 1/50 V (i.e. 20 mV): 5.1 V is max
    uint8_t  bleedVSet = (motorControl.bleedSystem()->reportSetting()                 * 10.0F) ; // an integer containing 10x the present servo setting
    uint8_t  bleedVAng = (motorControl.bleedSystem()->reportPosition()                * 50.0F) ; // in units of 1/50 V (i.e. 20 mV): 5.1 V is max
    uint8_t  cutdwnSet = (motorControl.cutdownSystem()->reportSetting()               * 10.0F) ; // an integer containing 10x the present servo setting
    uint8_t  cutdwnAng = (motorControl.cutdownSystem()->reportPosition()              * 50.0F) ; // in units of 1/50 V (i.e. 20 mV): 5.1 V is max

    uint8_t  lightStat =  lightControl.getLightStatusByte()                                                              ;
    uint16_t pd1ADRead =  lightControl.lightSourceMon()->ads1115ADC2()->readADC_SingleEnded( INTSPHERE_PD1_ADC_CHANNEL ) ;
    uint16_t pd2ADRead =  lightControl.lightSourceMon()->ads1115ADC2()->readADC_SingleEnded( INTSPHERE_PD2_ADC_CHANNEL ) ;
    uint16_t pd3ADRead =  lightControl.lightSourceMon()->ads1115ADC2()->readADC_SingleEnded( INTSPHERE_PD3_ADC_CHANNEL ) ;

    sendString2[0]  = (unsigned char)  TX_START_BYTE;
    sendString2[1]  = (unsigned char)  (0x21);           // Number of bytes of data that will be sent (0x21 = 33).

    for (int i = 0; i < 8; ++i)     sendString2[2+i]  =  byte(  packedTem[i]          & 0xFF);

    sendString2[10] = byte(  rssi            & 0xFF);

    sendString2[11] = byte(  bat1V           & 0xFF);
    sendString2[12] = byte(  bat2V           & 0xFF);

    sendString2[13] =       'T'                     ;

    sendString2[14] = byte(( occSpace >>  8) & 0xFF);
    sendString2[15] = byte(  occSpace        & 0xFF);

    sendString2[16] = byte(  powerMot1       & 0xFF);
    sendString2[17] = byte(  powerMot2       & 0xFF);
    sendString2[18] = byte(  powerMot3       & 0xFF);
    sendString2[19] = byte(  powerMot4       & 0xFF);

    sendString2[20] = byte(  axlRotSet       & 0xFF);
    sendString2[21] = byte(  axlRotAng       & 0xFF);
    sendString2[22] = byte(  bleedVSet       & 0xFF);
    sendString2[23] = byte(  bleedVAng       & 0xFF);
    sendString2[24] = byte(  cutdwnSet       & 0xFF);
    sendString2[25] = byte(  cutdwnAng       & 0xFF);

    sendString2[26] =       'T'                     ;

    sendString2[27] = byte(  lightStat       & 0xFF);
    sendString2[28] = byte(( pd1ADRead >> 8) & 0xFF);
    sendString2[29] = byte(  pd1ADRead       & 0xFF);
    sendString2[30] = byte(( pd2ADRead >> 8) & 0xFF);
    sendString2[31] = byte(  pd2ADRead       & 0xFF);
    sendString2[32] = byte(( pd3ADRead >> 8) & 0xFF);
    sendString2[33] = byte(  pd3ADRead       & 0xFF);

    sendString2[34] =       'T'                     ;

//    if (send(sendString2, 35)) Serial.println(F("Successfully sent sendString2"));
    send(sendString2, 35);

    return true;
}

/**************************************************************************/
/*!
 @brief  Send a command from a ground station up to ALTAIR.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendCommandToALTAIR(byte commandByte1 , 
                                           byte commandByte2  )
{
/*
           send(      (unsigned char)           RX_START_BYTE ) ;
           send(                                0x02          ) ;
           send(                                commandByte1  ) ;
    return send(                                commandByte2  ) ;
*/
    byte   sendString[4];
    sendString[0]  =  (unsigned char)           RX_START_BYTE   ;
    sendString[1]  =  (unsigned char)          (0x02)           ;
    sendString[2]  =                            commandByte1    ;
    sendString[3]  =                            commandByte2    ;
    if (send(sendString, 4)) { Serial.print(radioName()); Serial.println(F("  successfully sent sendString")); }
    return true;
}


/**************************************************************************/
/*!
 @brief  Send bare GPS info.  (Only use within a data packet wrapper 
         function!)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendBareGPS(uint8_t hour    , uint8_t minute   , uint8_t second  ,
                                   int32_t latitude, int32_t longitude, int16_t elevation)
{
  //send time
           sendBareGPSTime(  hour     , minute   , second);
  //send latitude & longitude
           sendBareGPSLatLon(latitude , longitude        );
  //send elevation
    return sendBareGPSEle(   elevation                   );
}

/**************************************************************************/
/*!
 @brief  Send bare GPS time info.  (Only use within a data packet wrapper 
         function!)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendBareGPSTime(uint8_t hour    , uint8_t minute   , uint8_t second)
{
  //send time
           send(byte(      hour        & 0xFF));
           send(byte(    minute        & 0xFF));
    return send(byte(    second        & 0xFF));
}

/**************************************************************************/
/*!
 @brief  Send bare GPS lat + lon info. (Only use within a data packet wrapper 
         function!)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendBareGPSLatLon(int32_t latitude, int32_t longitude)
{
  //send latitude
           send(byte((latitude  >> 24) & 0xFF));
           send(byte((latitude  >> 16) & 0xFF));
           send(byte((latitude  >>  8) & 0xFF));
           send(byte( latitude         & 0xFF));
  //send longitude
           send(byte((longitude >> 24) & 0xFF));
           send(byte((longitude >> 16) & 0xFF));
           send(byte((longitude >>  8) & 0xFF));
    return send(byte( longitude        & 0xFF));
}

/**************************************************************************/
/*!
 @brief  Send bare GPS elevation info. (Only use within a data packet wrapper 
         function!)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendBareGPSEle(int16_t elevation)
{
  //send elevation
           send(byte((elevation >>  8) & 0xFF));
    return send(byte( elevation        & 0xFF));
}

/**************************************************************************/
/*!
 @brief  Read a command sent up to ALTAIR from a ground station, or data
         sent down from ALTAIR to a ground station.
*/
/**************************************************************************/
void ALTAIR_GenTelInt::readALTAIRInfo(  byte command[],  bool isGroundStation )
{
/*
    static byte term[MAX_TERM_LENGTH]    =             "" ;
    static int  termIndex                =              0 ;
    static int  termLength               =              0 ;
*/
    byte        term[MAX_TERM_LENGTH]    =             "" ;
    int         termIndex                =              0 ;
    int         termLength               =              0 ;
    int         hasBegun                 =              0 ;
    long        readTry                  =              0 ;
    byte        startByte                =  RX_START_BYTE ;
    if (isGroundStation)  startByte      =  TX_START_BYTE ;
                command[0]               =              0 ;
                command[1]               =              0 ;
    if (!isBusy()) {
//      Serial.print(F("Reading radio: "));  Serial.println(radioName());
      while (true) {
        while (!available() && readTry < MAX_READ_TRIES) {
          ++readTry;
//           delay(5);
        }
        if (readTry < MAX_READ_TRIES) {
          do {
//            Serial.println(F("Reading a byte from radio"));
//            Serial.println(F("here1"));

            byte b = read();
//             term[termIndex++] = b;
//            if (isGroundStation) Serial.println(b, HEX);

            if (b == startByte && hasBegun == 0) {
              hasBegun = 1;
            }
            else if (hasBegun == 1) {
              termLength = (int)b;
              hasBegun = 2;
            }
            else if (hasBegun == 2) {
//              Serial.println(F("here1.1"));
              term[termIndex++] = b;
//              Serial.println(F("here1.2"));
            } 
//            Serial.println(F("here2"));
            if (termLength == termIndex && termLength > 0) break;
//             if (termIndex == MAX_TERM_LENGTH) break;
          } while (available());
        
//           if (termIndex == MAX_TERM_LENGTH) break;
          if (termLength == termIndex && termLength > 0) break;
    
        } else {
//          Serial.println(F("Radio is not available for reading"));
          break;
        }
      }
      if (termLength == termIndex && termLength > 0) {
        command[0] = term[0];
        command[1] = term[1];
        if (isGroundStation) groundStationPrintRxInfo(term, termLength);
        termLength = termIndex = hasBegun = 0;    
        return;
      }
    } else {
      Serial.print(F("Cannot read commands from the ground station, since the "));
      Serial.print(radioName());
      Serial.println(F(" radio is busy"));
    }
//    Serial.println(F("here3"));
}


/**************************************************************************/
/*!
 @brief  Just print out data sent down from ALTAIR to a ground station.
*/
/**************************************************************************/
void ALTAIR_GenTelInt::printALTAIRInfo(  )
{
    long        readTry                  =              0 ;
//    if (!isBusy()) {
      Serial.print(F("Reading radio "));
      Serial.println(radioName());
      while (true) {
        while (!available() && readTry < MAX_READ_TRIES) {
          ++readTry;
//           delay(5);
        }
        if (readTry < MAX_READ_TRIES) {
          do {
//            Serial.println(F("Reading a byte from radio"));

            byte b = read();

            Serial.print(b, HEX);
    
          } while (available());
        } else {
//          Serial.println(F("Radio is not available for reading"));
//          break;
        }
      }
/*
    } else {
      Serial.print(F("Cannot read commands from the ground station, since the "));
      Serial.print(radioName());
      Serial.println(F(" radio is busy"));
    }
*/
}


/**************************************************************************/
/*!
 @brief  Print out (to Serial) the info received from ALTAIR by a ground 
         station.
*/
/**************************************************************************/
void ALTAIR_GenTelInt::groundStationPrintRxInfo(  byte  term[] ,  int termLength )
{
    Serial.print(F("Number of bytes: "));    Serial.println(termLength);
//    Serial.print(F("  Data: \""));
//    for (int i = 0; i < termLength; i++)
//    {
//      Serial.print((char)term[i]);
//    }
//    Serial.println("\"");
    Serial.print(F("Data (HEX): "));
    for (int i = 0; i < termLength; i++)
    {
      if (term[i] < 0x10) Serial.print('0');
      Serial.print(term[i], HEX); Serial.print(" ");
    }
    Serial.println();  
//    Serial.println("\"");  

 if (termLength > 42) {
/*
      Serial.print(F("Transmitter station GMT time: "));  
      Serial.print(term[0], DEC); Serial.print(":"); 
      if (term[1] < 10) Serial.print("0"); Serial.print(term[1], DEC); Serial.print(":"); 
      if (term[2] < 10) Serial.print("0"); Serial.println(term[2], DEC);

//      if (termLength > 42) {
*/
        long lat = 0;
        long lon = 0;
        int  ele = 0;
        int  age = 0;

        lat  = ((unsigned long) term[0])  << 24;
        lat |= ((unsigned long) term[1])  << 16;
        lat |= ((unsigned long) term[2])  << 8;
        lat |= ((unsigned long) term[3]);
    
        lon  = ((unsigned long) term[4]) << 24;
        lon |= ((unsigned long) term[5]) << 16;
        lon |= ((unsigned long) term[6]) << 8;
        lon |= ((unsigned long) term[7]);

        ele |= ((unsigned int)  term[8]) << 8;
        ele |= ((unsigned int)  term[9]);

        age  = (                term[10]);

        Serial.print(F("Latitude:  ")); Serial.println(lat);
        Serial.print(F("Longitude: ")); Serial.println(lon);
        Serial.print(F("Elevation above SL (in m): ")); Serial.println(ele);
        Serial.print(F("GPS age (in units of 256 milliseconds): ")); Serial.println(age);
//      }
    }
    Serial.flush();
}
