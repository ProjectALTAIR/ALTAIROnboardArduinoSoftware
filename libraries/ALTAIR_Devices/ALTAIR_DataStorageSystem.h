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
#include <SD.h>

#define   DEFAULT_SDCARD_CSPIN          24
#define   DEFAULT_SDCARD_FILENAME    "data.txt"

class ALTAIR_DataStorageSystem {
  public:

    ALTAIR_DataStorageSystem(                )                             ;


    void                     initialize(     )                             ;

  protected:

  private:
    File                    _theSDCardFile                                 ;

};
#endif    //   ifndef ALTAIR_DataStorageSystem_h
