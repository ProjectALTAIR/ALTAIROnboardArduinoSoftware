/**************************************************************************/
/*!
    @file     ALTAIR_TelemetrySystem.h
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

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_TelemetrySystem_h
#define ALTAIR_TelemetrySystem_h

#include "Arduino.h"
#include "ALTAIR_DNT900.h"
#include "ALTAIR_SHX144.h"
#include "ALTAIR_RFM23BP.h"

class ALTAIR_TelemetrySystem {
  public:

    ALTAIR_TelemetrySystem(                  )                             ;

    ALTAIR_DNT900&           dnt900(         ) { return _dnt900            ; }
    ALTAIR_SHX144&           shx144(         ) { return _shx144            ; }
    ALTAIR_RFM23BP&          rfm23bp(        ) { return _rfm23bp           ; }

    ALTAIR_GenTelInt*        primary(        ) { return _primaryRadio      ; }
    ALTAIR_GenTelInt*        backup1(        ) { return _firstBackupRadio  ; }
    ALTAIR_GenTelInt*        backup2(        ) { return _secondBackupRadio ; }

    void                     initialize(     )                             ;
    void                     switchToBackup1()                             ;
    void                     switchToBackup2()                             ;

  protected:

  private:
    ALTAIR_DNT900            _dnt900                                       ;
    ALTAIR_SHX144            _shx144                                       ;
    ALTAIR_RFM23BP           _rfm23bp                                      ;

    ALTAIR_GenTelInt*        _primaryRadio                                 ;
    ALTAIR_GenTelInt*        _firstBackupRadio                             ;
    ALTAIR_GenTelInt*        _secondBackupRadio                            ;

};
#endif    //   ifndef ALTAIR_TelemetrySystem_h
