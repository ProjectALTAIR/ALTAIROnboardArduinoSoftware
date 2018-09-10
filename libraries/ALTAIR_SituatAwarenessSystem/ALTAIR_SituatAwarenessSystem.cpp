/**************************************************************************/
/*!
    @file     ALTAIR_SituatAwarenessSystem.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR "situational awareness" system, which
    includes the two GPS receivers (the NEO-M8N on the mast, and the
    DFRobot G6 in a small plastic housing directly atop the payload); the
    three orientation sensors (the Adafruit BNO055 accel/gyro/mag on the
    mast, the CHRobotics UM7 accel/gyro/mag within the payload, and the
    SparkFun HMC6343 accel/mag on the mast); the three Adafruit BME280
    pressure/temp/humidity sensors; and the many (primarily LM333) other
    temperature sensors located at various places within and around ALTAIR.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 5 Sept. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_SituatAwarenessSystem.h"
#include "ALTAIR_TCA9548A.h"

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_SituatAwarenessSystem::ALTAIR_SituatAwarenessSystem()
{
}

/**************************************************************************/
/*!
 @brief  Initialize all devices.
*/
/**************************************************************************/
void ALTAIR_SituatAwarenessSystem::initialize(             )
{
    _orientSensors.initialize() ;

     Serial.println(F("BME280 pressure/temp/humidity sensors initialization..."));
     bool status1, status2, status3, statusall;    
     status1 = _bmeMast.begin(                            );
     status2 = _bmeBalloon.begin( 0x76                    );
     ALTAIR_TCA9548A::tcaselect(  TCA9548A_BME280PAYLOAD  );
     status3 = _bmePayload.begin(                         );
     ALTAIR_TCA9548A::tcaselect(  TCA9548A_EVERYTHINGELSE );
     statusall = status1 && status3; // && status2;
     if (!statusall) {
         Serial.print(F("Could not find one of the 3 BME280 sensors, check wiring!  BME280 on nav mast: ")); Serial.print(status1); 
         Serial.print(F("   BME280 in the balloon valve: ")); Serial.print(status2); 
         Serial.print(F("   BME280 inside the gondola: ")); Serial.println(status3); 
         while (1);
     }

    _gpsSensors.initialize() ;
}


