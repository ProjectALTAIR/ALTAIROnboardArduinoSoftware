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
#include "ALTAIR_GPSSensor.h"

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
  pinMode(            DEFAULT_SDCARD_CSPIN ,         OUTPUT       )   ;
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         HIGH         )   ;   // try adding this
  delay(              10                                          )   ;
//  if (!_SD.begin(     DEFAULT_SDCARD_CSPIN                       )) {   // try changing this to the line below
//  if (!_SD.begin(     DEFAULT_SDCARD_CSPIN ,  SD_SCK_MHZ(  50  ) )) {        
  if (!_SD.begin(     DEFAULT_SDCARD_CSPIN ,  SD_SCK_MHZ(  2  ) )) {        
    Serial.println(F("SD card output initialization failed!"     ))   ;
    _SD.initErrorPrint()                                              ;
    while(1)                                                          ;
  }
  _theSDCardFile = _SD.open(DEFAULT_SDCARD_FILENAME, FILE_WRITE   )   ;
  if (_theSDCardFile                                              ) {
    Serial.println(F("SD card initialization complete."          ))   ;
  } else                                                            {
    Serial.println(F("SD card file open failed."                 ))   ;
    while(1)                                                          ;
  }
  _theSDCardFile.close(                                           )   ;   // try adding this
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         LOW          )   ;   // try adding this
  byte received_byte = SPI.transfer(                 SD_SPI_BYTE  )   ;   // try adding this
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         HIGH         )   ;   // try adding this
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
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         LOW          )   ;   // try adding this
  byte received_byte = SPI.transfer(                 SD_SPI_BYTE  )   ;   // try adding this
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         HIGH         )   ;   // try adding this
  return  ((uint16_t)     filesize                                )   ;  // in Mb
}


/**************************************************************************/
/*!
 @brief  Return the amount of remaining free space in the disk volume,
         in Mb.
*/
/**************************************************************************/
uint16_t ALTAIR_DataStorageSystem::remainingSpace(                )
{
//  uint16_t volumesize  =              0                               ;
  uint16_t volumesize  =  MY_SD_CARD_SIZE - SD_FILESYS_OVERHEAD       ;  // in Mb
/*
  uint32_t volbigsize  = _SD.vol()->freeClusterCount(             )   ;
           volbigsize *= _SD.vol()->blocksPerCluster(             )/2 ;  // in kb
           volbigsize /=           1024.                              ;  // in Mb
           volumesize  = ((uint16_t)   volbigsize                 )   ;  // in Mb
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         LOW          )   ;   // try adding this
  byte received_byte = SPI.transfer(                 SD_SPI_BYTE  )   ;   // try adding this
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         HIGH         )   ;   // try adding this
*/
  return  ((uint16_t)    (volumesize - occupiedSpace(           )))   ;  // in Mb
}

/**************************************************************************/
/*!
 @brief  Store a GPS timestamp.
*/
/**************************************************************************/
void   ALTAIR_DataStorageSystem::storeTimestamp( ALTAIR_GPSSensor* gps )
{
  _theSDCardFile = _SD.open(DEFAULT_SDCARD_FILENAME, FILE_WRITE   )   ;  // try opening and closing before and after
  _theSDCardFile.print("GPS UTC time: ");  
  _theSDCardFile.print(gps->hour());  
  _theSDCardFile.print("   Milliseconds since CPU start: ");  
  _theSDCardFile.println(millis());  
  _theSDCardFile.close();                                                // i.e., add this file close line too
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         LOW          )   ;   // try adding this
  byte received_byte = SPI.transfer(                 SD_SPI_BYTE  )   ;   // try adding this
  digitalWrite(       DEFAULT_SDCARD_CSPIN ,         HIGH         )   ;   // try adding this
}
