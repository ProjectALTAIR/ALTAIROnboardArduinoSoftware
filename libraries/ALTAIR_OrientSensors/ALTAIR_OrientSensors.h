/**************************************************************************/
/*!
    @file     ALTAIR_OrientSensors.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR orientation sensors:

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_OrientSensors_h
#define ALTAIR_OrientSensors_h

#include "Arduino.h"
#include "ALTAIR_BNO055.h"
#include "ALTAIR_UM7.h"
#include "ALTAIR_HMC6343.h"
#include "ALTAIR_HMC5883L.h"

class ALTAIR_OrientSensors {
  public:

    ALTAIR_OrientSensors(                    )                     ;

    ALTAIR_BNO055*          bno055(          ) { return &_bno055   ; }
    ALTAIR_UM7*             um7(             ) { return &_um7      ; }
    ALTAIR_HMC6343*         hmc6343(         ) { return &_hmc6343  ; }
    ALTAIR_HMC5883L*        hmc5883l(        ) { return &_hmc5883l ; }

    ALTAIR_OrientSensor*    primary(         ) { return  _primary  ; }
    ALTAIR_OrientSensor*    backup1(         ) { return  _backup1  ; }
    ALTAIR_OrientSensor*    backup2(         ) { return  _backup2  ; }

    void                    initialize(      )                     ;
    void                    switchToBackup1( )                     ;
    void                    switchToBackup2( )                     ;

  private:
    ALTAIR_BNO055          _bno055                                 ;
    ALTAIR_UM7             _um7                                    ;
    ALTAIR_HMC6343         _hmc6343                                ;
    ALTAIR_HMC5883L        _hmc5883l                               ;

    ALTAIR_OrientSensor*   _primary                                ; 
    ALTAIR_OrientSensor*   _backup1                                ; 
    ALTAIR_OrientSensor*   _backup2                                ; 
};
#endif    //   ifndef ALTAIR_OrientSensors_h
