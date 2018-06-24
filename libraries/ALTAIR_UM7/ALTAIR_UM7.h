/**************************************************************************/
/*!
    @file     ALTAIR_UM7.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the CH Robotics UM7 orientation sensor
    for ALTAIR (which additionally passes through and utilizes GPS 
    information from the DHRobot GPS receiver).  It is instantiated
    as a singleton, and the member functions can be accessed to 
    obtain orientation and GPS location information (as well as 
    additional information such as acceleration, temperature, and 
    device health).  Essentially all device functionality is 
    available via this interface.
    Justin Albert  jalbert@uvic.ca     began on 21 Nov. 2017

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef      ALTAIR_UM7_h
#define      ALTAIR_UM7_h

#include    "Arduino.h"

struct UM7packet {
             byte      Address;
             byte      PT;             // Packet Type
    unsigned int       Checksum;
             byte      data_length;
             byte      data[75];
};

typedef union {
             byte      array[4];
             float     value;
} ByteToFloat;


class ALTAIR_UM7 {
  public:
    ALTAIR_UM7(                           const  char      serialID     );

    struct   UM7packet getDataPacket();
    struct   UM7packet getHealthPacket();


    static   float     getYaw(            struct UM7packet dataPacket   );
    static   float     getPitch(          struct UM7packet dataPacket   );
    static   float     getRoll(           struct UM7packet dataPacket   );
    static   float     getXAccel(         struct UM7packet dataPacket   );
    static   float     getYAccel(         struct UM7packet dataPacket   );
    static   float     getZAccel(         struct UM7packet dataPacket   );

    static   byte      getnSatsUsed(      struct UM7packet healthPacket );
    static   byte      getnSatsInView(    struct UM7packet healthPacket );
    static   byte      getHDOP(           struct UM7packet healthPacket );
    static   byte      getnSensors(       struct UM7packet healthPacket );

    static   float     convertBytesToFloat(      byte*     data );


             byte      parse_serial_data( const  byte*     rx_data,   byte rx_length, byte requestedAddress, struct UM7packet* packet );

  protected:

  private:
             char     _serialID;
  
};

#endif

