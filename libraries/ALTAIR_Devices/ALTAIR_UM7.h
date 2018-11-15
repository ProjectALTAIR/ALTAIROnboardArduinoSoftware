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
#include    "ALTAIR_OrientSensor.h"

#define      DEFAULT_UM7_SERIALID         3

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


class ALTAIR_UM7 : public ALTAIR_OrientSensor {
  public:
    ALTAIR_UM7(                           const  char       serialID           );
    ALTAIR_UM7(                                                                );  // default constructor => default argument to constructor

    struct   UM7packet getDataPacket(                                          );
    struct   UM7packet getHealthPacket(                                        );
             void      update(                                                 ) {        getDataPacket();                           getHealthPacket(); }


    static   float     getYaw(            struct UM7packet  dataPacket         );
    static   float     getPitch(          struct UM7packet  dataPacket         );
    static   float     getRoll(           struct UM7packet  dataPacket         );
    static   float     getXAccel(         struct UM7packet  dataPacket         );
    static   float     getYAccel(         struct UM7packet  dataPacket         );
    static   float     getZAccel(         struct UM7packet  dataPacket         );

    static   byte      getnSatsUsed(      struct UM7packet  healthPacket       );
    static   byte      getnSatsInView(    struct UM7packet  healthPacket       );
    static   byte      getHDOP(           struct UM7packet  healthPacket       );
    static   byte      getnSensors(       struct UM7packet  healthPacket       );
    static   float     getTemperature(    struct UM7packet  healthPacket       );
    static   byte      getTypeAndHealth(  struct UM7packet  healthPacket       );

    static   float     convertBytesToFloat(      byte*      data               );

             byte      parse_serial_data( const  byte*      rx_data         ,   
                                                 byte       rx_length       , 
                                                 byte       requestedAddress, 
                                          struct UM7packet* packet             );

    void               initialize(                                             );
    void               checkUM7Health(                                         );

    int16_t            accelZ(                                                 ) { return convertFloatToInt16(getZAccel(         _lastGoodDataPacket  )      ); }
    int16_t            accelX(                                                 ) { return convertFloatToInt16(getXAccel(         _lastGoodDataPacket  )      ); }
    int16_t            accelY(                                                 ) { return convertFloatToInt16(getYAccel(         _lastGoodDataPacket  )      ); }
    int16_t            yaw(                                                    ) { return ((_lastGoodDataPacket.data[48] << 8) | _lastGoodDataPacket.data[49]); }
    int16_t            pitch(                                                  ) { return ((_lastGoodDataPacket.data[46] << 8) | _lastGoodDataPacket.data[47]); }
    int16_t            roll(                                                   ) { return ((_lastGoodDataPacket.data[44] << 8) | _lastGoodDataPacket.data[45]); }
    int8_t             temperature(                                            ) { return (int8_t)              getTemperature(  _lastGoodHealthPacket       ); }
    uint8_t            typeAndHealth(                                          ) { return getTypeAndHealth(                      _lastGoodHealthPacket       ); }

  protected:

  private:
             char      _serialID                                                ;
    struct   UM7packet _dataPacket                                              ;
    struct   UM7packet _healthPacket                                            ;
    struct   UM7packet _lastGoodDataPacket                                      ;
    struct   UM7packet _lastGoodHealthPacket                                    ;
};

#endif

