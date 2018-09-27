/**************************************************************************/
/*!
    @file     ALTAIR_DataStorageSystem.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR onboard microSD card-based data
    storage system, located on the onboard Adafruit MicroSD card breakout
    board.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 8 Sept. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_DataStorageSystem.h"
#include <TinyGPS++.h>

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_DataStorageSystem::ALTAIR_DataStorageSystem()
{
}

/**************************************************************************/
/*!
 @brief  Initialize the microSD card data storage system.
*/
/**************************************************************************/
void ALTAIR_DataStorageSystem::initialize(                        )
{
  Serial.println(F(  "Initializing SPI bus SD card output ..."   ))   ;
  if (!_SD.begin(     DEFAULT_SDCARD_CSPIN                       )) {
    Serial.println(F("SD card output initialization failed!"     ))   ;
    while(1)                                                          ;
  }
  _theSDCardFile = _SD.open(DEFAULT_SDCARD_FILENAME, FILE_WRITE   )   ;
  if (_theSDCardFile                                              ) {
    Serial.println(F("SD card initialization complete."          ))   ;
  } else                                                            {
    Serial.println(F("SD card file open failed."                 ))   ;
    while(1)                                                          ;
  }
  Serial.println(F("SPI bus and device initialization complete." ))   ;
}

/**************************************************************************/
/*!
 @brief  Return the amount of space presently occupied by _theSDCardFile,
         in Mb.
*/
/**************************************************************************/
uint16_t ALTAIR_DataStorageSystem::occupiedSpace(                 )
{
  double   filesize    = _theSDCardFile.size(                     )   ;  // in bytes
           filesize   /=           1024.                              ;  // in kb
           filesize   /=           1024.                              ;  // in Mb
  return   filesize                                                   ;  // in Mb
}


/**************************************************************************/
/*!
 @brief  Return the amount of remaining free space in the disk volume,
         in Mb.
*/
/**************************************************************************/
uint16_t ALTAIR_DataStorageSystem::remainingSpace(                )
{
  uint32_t volumesize  = _SD.vol()->freeClusterCount(             )   ;
           volumesize *= _SD.vol()->blocksPerCluster(             )/2 ;  // in kb
           volumesize /=           1024.                              ;  // in Mb
  return  (volumesize  -  occupiedSpace(                         ))   ;  // in Mb
}

/**************************************************************************/
/*!
 @brief  Store a GPS timestamp.
*/
/**************************************************************************/
void   ALTAIR_DataStorageSystem::storeTimestamp( TinyGPSPlus& gps )
{
  _theSDCardFile.print("GPS UTC time: ");  
  _theSDCardFile.print(gps.time.hour());  
  _theSDCardFile.print("   Milliseconds since CPU start: ");  
  _theSDCardFile.println(millis());  

}
