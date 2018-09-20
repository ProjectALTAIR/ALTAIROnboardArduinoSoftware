/**************************************************************************/
/*!
    @file     ALTAIR_TelemetrySystem.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR telemetry system, including the three
    telemetry radios: the DNT900 (which operates at 910 MHz, and has its
    half-wave antenna at the front of the payload), the SHX144 (which 
    operates at 144 MHz, and has its quarter-wave antenna at the back of 
    the payload), and the RFM23BP (which operates at 440 MHz, and has its
    half-wave antenna on the topside of the payload).

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 4 Sept. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_TelemetrySystem.h"

/**************************************************************************/
/*!
 @brief  Constructor.  Constructs the three transceiver objects with
         their default arguments (via their respective default 
         constructors).
*/
/**************************************************************************/
ALTAIR_TelemetrySystem::ALTAIR_TelemetrySystem()
{
    _primaryRadio       = &_dnt900  ;
    _firstBackupRadio   = &_shx144  ;
    _secondBackupRadio  = &_rfm23bp ;
}

/**************************************************************************/
/*!
 @brief  Initialize all three transceivers (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_TelemetrySystem::initialize(                                   )
{
  Serial.println(F("Starting DNT900 radio setup..."));
  if (!_dnt900.initialize()) {
    Serial.println(F("DNT900 radio init failed"));
    while(1);
  }
  Serial.println(F("DNT900 radio setup complete."));

  Serial.println(F("Starting SHX1 serial modem radio setup..."));
  if (!_shx144.initialize()) {
    Serial.println(F("SHX1 radio init failed"));
    while(1);
  }
  Serial.println(F("SHX1 serial modem radio setup complete."));

  delay(100);

  Serial.println(F("Initializing SPI bus RFM23BP radio (433 MHz, antenna on top of gondola) ..."));
  if (!_rfm23bp.initialize()) {
    Serial.println(F("RFM23BP radio init failed"));
    while(1);
  }
}

/**************************************************************************/
/*!
 @brief  Switch to backup radio #1.
*/
/**************************************************************************/
void ALTAIR_TelemetrySystem::switchToBackup1(                              )
{
     ALTAIR_GenTelInt*  formerPrimary = primary();
                       _primaryRadio  = backup1();
                   _firstBackupRadio  = formerPrimary;
}

/**************************************************************************/
/*!
 @brief  Switch to backup radio #2.
*/
/**************************************************************************/
void ALTAIR_TelemetrySystem::switchToBackup2(                              )
{
     ALTAIR_GenTelInt*  formerPrimary = primary();
                       _primaryRadio  = backup2();
                  _secondBackupRadio  = backup1();
                   _firstBackupRadio  = formerPrimary;
}


