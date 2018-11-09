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
#include "ALTAIR_ArduinoMicro.h"
#include <TinyGPS++.h>
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

    uint8_t  bat1V     = (deviceControl.sitAwareSystem()->genOpsBatt()->readVoltage() * 18.18F) ; // in units of 55 mV (would turn over at 14 V)
    uint8_t  bat2V     = (deviceControl.sitAwareSystem()->propBatt()->readVoltage()   * 18.18F) ; // in units of 55 mV (would turn over at 14 V)

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

    int8_t*  packedRPM = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedRPM();
    int8_t*  packedCur = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedCurrent();
    int8_t*  packedTem = (int8_t*)      deviceControl.sitAwareSystem()->arduinoMicro()->packedTemp();

    sendStart();
    send(0x2B);                                         // Number of bytes of data that will be sent.

    sendBareGPSTime(  hour     , minute   , second); 
    sendBareGPSLatLon(latitude , longitude        ); 
    sendBareGPSEle(   elevation                   );    // 13 bytes total (up to here)

    send('T');

    send(  byte(( age       >>  8)      & 0xFF));
    send(  byte(  age                   & 0xFF));
    send(  byte(  hdop                  & 0xFF));

    send(  byte(  rssi                  & 0xFF));

    send(  byte(  bat1V                 & 0xFF));
    send(  byte(  bat2V                 & 0xFF));

    send('T');

    send(  byte(( outPres   >>  8)      & 0xFF));
    send(  byte(  outPres               & 0xFF));
    send(  byte(  outTemp               & 0xFF));
    send(  byte(  outHum                & 0xFF));
    send(  byte(( inPres    >>  8)      & 0xFF));
    send(  byte(  inPres                & 0xFF));
    send(  byte(  inTemp                & 0xFF));
    send(  byte(  inHum                 & 0xFF));
    send(  byte(( balPres   >>  8)      & 0xFF));
    send(  byte(  balPres               & 0xFF));
    send(  byte(  balTemp               & 0xFF));
    send(  byte(  balHum                & 0xFF));

    send('T');

    for (int i = 0; i < 4; ++i)     send(  byte(  packedRPM[i]          & 0xFF));
    for (int i = 0; i < 4; ++i)     send(  byte(  packedCur[i]          & 0xFF));

    send('T');

//    delay(50);
//    delay(200);

// try moving work here (instead of a CPU-cycle-wasting delay)

    ALTAIR_DataStorageSystem* sdCard =  deviceControl.dataStoreSystem();
    uint16_t occSpace  =      sdCard->occupiedSpace();
    uint16_t freeSpace =      sdCard->remainingSpace();

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
    int16_t  diffPD12  =  lightControl.lightSourceMon()->ads1115ADC2()->readADC_Differential_0_1(                      ) ;

    sendStart();
    send(0x26);                                         // Number of bytes of data that will be sent.

    sendBareGPSTime(  hour     , minute   , second); 

    for (int i = 0; i < 8; ++i)     send(  byte(  packedTem[i]          & 0xFF));   // 11 bytes total (up to here)

    send('T');

    send(  byte(( occSpace  >>  8)      & 0xFF));
    send(  byte(  occSpace              & 0xFF));
    send(  byte(( freeSpace >>  8)      & 0xFF));
    send(  byte(  freeSpace             & 0xFF));

    send(  byte(  powerMot1             & 0xFF));
    send(  byte(  powerMot2             & 0xFF));
    send(  byte(  powerMot3             & 0xFF));
    send(  byte(  powerMot4             & 0xFF));

    send('T');

    send(  byte(  axlRotSet             & 0xFF));    
    send(  byte(  axlRotAng             & 0xFF));    
    send(  byte(  bleedVSet             & 0xFF));    
    send(  byte(  bleedVAng             & 0xFF));    
    send(  byte(  cutdwnSet             & 0xFF));    
    send(  byte(  cutdwnAng             & 0xFF));    

    send('T');

    send(  byte(  lightStat             & 0xFF));
    send(  byte(( pd1ADRead >>  8)      & 0xFF));
    send(  byte(  pd1ADRead             & 0xFF));
    send(  byte(( pd2ADRead >>  8)      & 0xFF));
    send(  byte(  pd2ADRead             & 0xFF));
    send(  byte(( pd3ADRead >>  8)      & 0xFF));
    send(  byte(  pd3ADRead             & 0xFF));
    send(  byte(( diffPD12  >>  8)      & 0xFF));
    send(  byte(  diffPD12              & 0xFF));

    return send('T');
}

/**************************************************************************/
/*!
 @brief  Send a command from a ground station up to ALTAIR.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendCommandToALTAIR(byte commandByte1 , 
                                           byte commandByte2  )
{
           send(      (unsigned char)           RX_START_BYTE ) ;
           send(                                0x02          ) ;
           send(                                commandByte1  ) ;
    return send(                                commandByte2  ) ;
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
    if (!isBusy()) {
//      Serial.print(F("Reading radio "));
//      Serial.println(radioName());
      while (true) {
        while (!available() && readTry < MAX_READ_TRIES) {
          ++readTry;
//           delay(5);
        }
        if (readTry < MAX_READ_TRIES) {
          do {
//            Serial.println(F("Reading a byte from radio"));

            byte b = read();
//             term[termIndex++] = b;
    
            if (b == startByte && hasBegun == 0) {
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
      Serial.print(F("Cannot read commands from the ground station, since the"));
      Serial.print(radioName());
      Serial.println(F(" radio is busy"));
    }
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

    if (termLength > 12) {
      Serial.print(F("Transmitter station GMT time: "));  
        Serial.print(term[0], DEC); Serial.print(":"); 
        if (term[1] < 10) Serial.print("0"); Serial.print(term[1], DEC); Serial.print(":"); 
        if (term[2] < 10) Serial.print("0"); Serial.println(term[2], DEC);

      if (termLength > 42) {
        long lat = 0;
        long lon = 0;
        int  ele = 0;
        int  age = 0;

        lat  = ((unsigned long) term[3])  << 24;
        lat |= ((unsigned long) term[4])  << 16;
        lat |= ((unsigned long) term[5])  << 8;
        lat |= ((unsigned long) term[6]);
    
        lon  = ((unsigned long) term[7]) << 24;
        lon |= ((unsigned long) term[8]) << 16;
        lon |= ((unsigned long) term[9]) << 8;
        lon |= ((unsigned long) term[10]);

        ele |= ((unsigned int)  term[11]) << 8;
        ele |= ((unsigned int)  term[12]);

        age |= ((unsigned int)  term[14]) << 8;
        age |= ((unsigned int)  term[15]);

        Serial.print(F("Latitude:  ")); Serial.println(lat);
        Serial.print(F("Longitude: ")); Serial.println(lon);
        Serial.print(F("Elevation above SL (in m): ")); Serial.println(ele);
        Serial.print(F("GPS age (in milliseconds): ")); Serial.println(age);
      }
    }
    Serial.flush();
}
