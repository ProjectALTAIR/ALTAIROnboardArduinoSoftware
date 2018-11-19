/**************************************************************************/
/*!
    @file     ALTAIR_DataStorageSystem.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR onboard microSD card-based data
    storage system, located on the onboard Adafruit MicroSD card breakout 
    board.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 8 Sept. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_DataStorageSystem_h
#define   ALTAIR_DataStorageSystem_h

#include "Arduino.h"
#include <SdFat.h>

#define   DEFAULT_SDCARD_CSPIN          24
#define   DEFAULT_SDCARD_FILENAME    "data.txt"
#define   SD_SPI_BYTE                 0x00
#define   MY_SD_CARD_SIZE             8000          // in MB
#define   SD_FILESYS_OVERHEAD          100          // in MB  (an approximate value, for now)

class     ALTAIR_GPSSensor;

class     ALTAIR_DataStorageSystem {
  public:

    ALTAIR_DataStorageSystem(                                 )            ;


    void                initialize(                           )            ;

    uint16_t            occupiedSpace(                        )            ;  // current occupied  space on the disk, in Mb
    uint16_t            remainingSpace(                       )            ;  // current remaining space on the disk, in Mb

    void                storeTimestamp( ALTAIR_GPSSensor* gps )            ;

  protected:

  private:
    SdFat              _SD                                                 ;
    File               _theSDCardFile                                      ;
};
#endif    //   ifndef ALTAIR_DataStorageSystem_h
