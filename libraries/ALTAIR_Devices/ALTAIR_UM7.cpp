/**************************************************************************/
/*!
    @file     ALTAIR_UM7.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the CH Robotics UM7 orientation sensor
    for ALTAIR (which also passes through and utilizes GPS
    information from the DHRobot GPS receiver).  It is instatiated
    as a singleton, and member functions are accessed to obtain
    orientation and GPS location information.  
    Justin Albert  jalbert@uvic.ca     began on 21 Nov. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_UM7.h"

#define   RX_READ_LENGTH     200
#define   RX_READ_ATTEMPTS   500

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_UM7::ALTAIR_UM7(  const char serialID ) :
    _serialID(                      serialID )
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.  
*/
/**************************************************************************/
ALTAIR_UM7::ALTAIR_UM7(                      ) :
    _serialID(          DEFAULT_UM7_SERIALID )
{
}

/**************************************************************************/
/*!
 @brief  Initialization (starts serial and checks health).  
*/
/**************************************************************************/
void ALTAIR_UM7::initialize()
{
    switch (_serialID) {
      case 0:
        Serial.begin(  115200, SERIAL_8N1 );
        break;
      case 1:
        Serial1.begin( 115200, SERIAL_8N1 );
        break;
      case 2:
        Serial2.begin( 115200, SERIAL_8N1 );
        break;
      case 3:
        Serial3.begin( 115200, SERIAL_8N1 );
        break;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of UM7 orientation sensor!"));
        while(1);
    }
    checkUM7Health();
}

/**************************************************************************/
/*!
 @brief  Check the health of the UM7 sensor at initialization.  
*/
/**************************************************************************/
void ALTAIR_UM7::checkUM7Health()
{
    byte tx_data[7];
    byte rx_data[RX_READ_LENGTH];
    int returnVal, nAttempts = 0;
    bool dataReceived = false;
    struct UM7packet new_packet;
    tx_data[0] = 's';  // Send
    tx_data[1] = 'n';  // New
    tx_data[2] = 'p';  // Packet
    tx_data[3] = 0x00; // packet type byte
    tx_data[4] = 0x55; // address of DREG_HEALTH sensor health info register
    tx_data[5] = 0x01; // checksum high byte
    tx_data[6] = 0xA6; // checksum low byte  
    while (nAttempts < RX_READ_ATTEMPTS) {
      switch (_serialID) {
        case 0:
          if (Serial.available()) {
            Serial.write(  tx_data, 7 );
            Serial.readBytes(  rx_data, RX_READ_LENGTH );
            dataReceived = true;
          } 
          break;
        case 1:
          if (Serial1.available()) {
            Serial1.write( tx_data, 7 );
            Serial1.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 2:
          if (Serial2.available()) {
            Serial2.write( tx_data, 7 );
            Serial2.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 3:
          if (Serial3.available()) {
            Serial3.write( tx_data, 7 );
            Serial3.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        default:
          Serial.println(F("Unallowed serial ID provided in initialization of UM7 orientation sensor!"));
          while(1);
      }
      if (dataReceived) {
        break;
      } else {
        ++nAttempts;
      }
    }
    returnVal  = parse_serial_data(rx_data, RX_READ_LENGTH, tx_data[4], &new_packet);
    if ( returnVal == 0 ) {
      // Extract health info ...
      float sats_used_byte = -999., hdop_byte = -999., sats_in_view_byte = -999., sensors_byte = -999.;
      sats_used_byte    = new_packet.data[0];
      hdop_byte         = new_packet.data[1];
      sats_in_view_byte = new_packet.data[2];
      sensors_byte      = new_packet.data[3];
      // ... and print it out.
      Serial.print("sats_used_byte = "); Serial.print(sats_used_byte); Serial.print("    hdop_byte = "); Serial.print(hdop_byte);
         Serial.print("    sats_in_view_byte = "); Serial.print(sats_in_view_byte); Serial.print("    sensors_byte = "); Serial.println(sensors_byte);
    }
}

/**************************************************************************/
/*!
 @brief  Get the main data packet (containing the orientation and acceleration 
         info).  This can then be followed by the static member fuctions 
         (for example, getYaw(struct UM7packet dataPacket)) that parse the 
         data packet into its individual pieces of info.
*/
/**************************************************************************/
struct UM7packet ALTAIR_UM7::getDataPacket() {

    byte tx_data[20];
    byte rx_data[RX_READ_LENGTH];
    byte returnVal, nAttempts = 0;
    bool dataReceived = false;
    tx_data[0] = 's';  // Send
    tx_data[1] = 'n';  // New
    tx_data[2] = 'p';  // Packet
    tx_data[3] = 0x7C; // get a batch of 16 register words (= 64 bytes)
    tx_data[4] = 0x65; // start with the processed accelerometer info: address of DREG_ACCEL_PROC_X register
    tx_data[5] = 0x02; // checksum high byte
    tx_data[6] = 0x32; // checksum low byte

    while (nAttempts < RX_READ_ATTEMPTS) {
      switch (_serialID) {
        case 0:
          if (Serial.available()) {
            Serial.write(  tx_data, 7 );
            Serial.readBytes(  rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 1:
          if (Serial1.available()) {
            Serial1.write( tx_data, 7 );
            Serial1.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 2:
          if (Serial2.available()) {
            Serial2.write( tx_data, 7 );
            Serial2.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 3:
          if (Serial3.available()) {
            Serial3.write( tx_data, 7 );
            Serial3.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        default:
          Serial.println(F("Unallowed serial ID provided in initialization of UM7 orientation sensor!"));
          while(1);
      }
      if (dataReceived) {
        break;
      } else {
        ++nAttempts;
      }
    }
    returnVal = parse_serial_data(rx_data, RX_READ_LENGTH, tx_data[4], &_dataPacket);
    if ( returnVal != 0 ) { Serial.print(F("A bad data packet has been returned by the UM7 orientation sensor! -- with returnVal: ")); Serial.println(returnVal, HEX); }
    else                  { memcpy(&_lastGoodDataPacket, &_dataPacket, sizeof(_dataPacket)); }

    return _dataPacket;
}

/**************************************************************************/
/*!
 @brief  Get the heath packet (containing the sensor status and health
         info).  This can then be followed by the static member fuctions
         (for example, getHDOP(struct UM7packet healthPacket)) that parse the
         health packet into its individual pieces of info.

*/
/**************************************************************************/
struct UM7packet ALTAIR_UM7::getHealthPacket() {

    byte tx_data[7];
    byte rx_data[RX_READ_LENGTH];
    byte returnVal, nAttempts = 0; 
    bool dataReceived = false;
    tx_data[0] = 's';  // Send
    tx_data[1] = 'n';  // New
    tx_data[2] = 'p';  // Packet
    tx_data[3] = 0x7C; // get a batch of 16 register words (= 64 bytes)
    tx_data[4] = 0x55; // address of DREG_HEALTH sensor health info register
    tx_data[5] = 0x02; // checksum high byte
    tx_data[6] = 0x22; // checksum low byte  

    while (nAttempts < RX_READ_ATTEMPTS) {
      switch (_serialID) {
        case 0:
          if (Serial.available()) {
            Serial.write( tx_data, 7 );
            Serial.readBytes(  rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 1:
          if (Serial1.available()) {
            Serial1.write( tx_data, 7 );
            Serial1.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 2:
          if (Serial2.available()) {
            Serial2.write( tx_data, 7 );
            Serial2.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        case 3:
          if (Serial3.available()) {
            Serial3.write( tx_data, 7 );
            Serial3.readBytes( rx_data, RX_READ_LENGTH );
            dataReceived = true;
          }
          break;
        default:
          Serial.println(F("Unallowed serial ID provided in initialization of UM7 orientation sensor!"));
          while(1);
      }
      if (dataReceived) {
        break;
      } else {
        ++nAttempts;
      }
    }
    returnVal = parse_serial_data(rx_data, RX_READ_LENGTH, tx_data[4], &_healthPacket);
    if ( returnVal != 0 ) { Serial.print(F("A bad health packet has been returned by the UM7 orientation sensor! -- with returnVal: ")); Serial.println(returnVal, HEX); }
    else                  { memcpy(&_lastGoodHealthPacket, &_healthPacket, sizeof(_healthPacket)); }

    return _healthPacket;
}

/**************************************************************************/
/*!
 @brief  Extract the yaw from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getYaw( struct UM7packet dataPacket ) {
    int yawInt = -999;
    yawInt   = (dataPacket.data[48] << 8) |  dataPacket.data[49];
    return yawInt / 91.02222;
}

/**************************************************************************/
/*!
 @brief  Extract the pitch from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getPitch( struct UM7packet dataPacket ) {
    int pitchInt = -999;
    pitchInt   = (dataPacket.data[46] << 8) |  dataPacket.data[47];
    return pitchInt / 91.02222;
}

/**************************************************************************/
/*!
 @brief  Extract the roll from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getRoll( struct UM7packet dataPacket ) {
    int rollInt = -999;
    rollInt   = (dataPacket.data[44] << 8) |  dataPacket.data[45];
    return rollInt / 91.02222;
}

/**************************************************************************/
/*!
 @brief  Extract the acceleration in the x direction from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getXAccel( struct UM7packet dataPacket ) {
    return SENSORS_GRAVITY_EARTH * convertBytesToFloat(dataPacket.data);
}

/**************************************************************************/
/*!
 @brief  Extract the acceleration in the y direction from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getYAccel( struct UM7packet dataPacket ) {
    return SENSORS_GRAVITY_EARTH * convertBytesToFloat(&(dataPacket.data[4]));
}

/**************************************************************************/
/*!
 @brief  Extract the acceleration in the z direction from a data packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getZAccel( struct UM7packet dataPacket ) {
    return SENSORS_GRAVITY_EARTH * convertBytesToFloat(&(dataPacket.data[8]));
}

/**************************************************************************/
/*!
 @brief  Extract the temperature from a health packet.
*/
/**************************************************************************/
float ALTAIR_UM7::getTemperature( struct UM7packet healthPacket ) {
    return convertBytesToFloat(&(healthPacket.data[40]));
}

/**************************************************************************/
/*!
 @brief  Return the type and health, given a health packet.
*/
/**************************************************************************/
byte ALTAIR_UM7::getTypeAndHealth( struct UM7packet healthPacket ) {
    // if no failures to initialize, and if norm of acc (but not necessarily mag) are not too ridiculous:
    if ((healthPacket.data[3] & 0x1E) == 0) {  
       return ((byte) um7_healthy);
    } else {
       return ((byte) um7_unhealthy);
    }
}

/**************************************************************************/
/*!
 @brief  Extract the number of satellites used from a health packet.
*/
/**************************************************************************/
byte ALTAIR_UM7::getnSatsUsed( struct UM7packet healthPacket ) {
    return healthPacket.data[0];
}

/**************************************************************************/
/*!
 @brief  Extract the number of satellites in view from a health packet.
*/
/**************************************************************************/
byte ALTAIR_UM7::getnSatsInView( struct UM7packet healthPacket ) {
    return healthPacket.data[2];
}

/**************************************************************************/
/*!
 @brief  Extract the HDOP from a health packet.
*/
/**************************************************************************/
byte ALTAIR_UM7::getHDOP( struct UM7packet healthPacket ) {
    return healthPacket.data[1];
}

/**************************************************************************/
/*!
 @brief  Extract the number of sensors from a health packet.
*/
/**************************************************************************/
byte ALTAIR_UM7::getnSensors( struct UM7packet healthPacket ) {
    return healthPacket.data[3];
}

/**************************************************************************/
/*!
 @brief  Convert 4 bytes to a float.
*/
/**************************************************************************/
float ALTAIR_UM7::convertBytesToFloat( byte* data ) {
   ByteToFloat converter;
   for (byte i = 0; i < 4; i++){
      converter.array[3-i] = data[i];    //or: converter.array[i] = data[i];  if the endian were reversed 
   }
   return converter.value;
}

/**************************************************************************/
/*!
 @brief  Convert 4 bytes to a float.
*/
/**************************************************************************/



/**************************************************************************/
/*!
 @brief  Do the "hard work" of parsing the data received from the UM7 into 
         a valid packet containing the data, or return a value explaining 
         why the UM7 data received is not valid.
*/
/**************************************************************************/
byte ALTAIR_UM7::parse_serial_data( const byte* rx_data, byte rx_length, byte requestedAddress, struct UM7packet* packet ) {
   byte index;
// Make sure that the data buffer provided is long enough to contain a full packet
// The minimum packet length is 7 bytes
   if ( rx_length < 7 ) {
      return 1;
   }
// Try to find the `snp' start sequence for the packet, and ensure that either
//   a) the packet has the requested address, or 2) that any address will do (if requestedAddress == 0)
   for ( index = 0; index < (rx_length - 2); index++ ) {
// Check for `snp'. If found, immediately exit the loop
      if ( rx_data[index] == 's' && rx_data[index+1] == 'n' && rx_data[index+2] == 'p' &&
           ( requestedAddress == 0 || rx_data[index+4] == requestedAddress ) ) {
         break;
      }
   }
   byte packet_index = index;
// Check to see if the variable `packet_index' is equal to (rx_length - 2). If it is, then the above
// loop executed to completion and never found a packet header.
   if ( packet_index == (rx_length - 2) ) {
      return 2;
   }
// If we get here, a packet header was found. Now check to see if we have enough room
// left in the buffer to contain a full packet. Note that at this point, the variable `packet_index'
// contains the location of the `s' character in the buffer (the first byte in the header)
   if ( (rx_length - packet_index) < 7 ) {
      return 3;
   }
// We've found a packet header, and there is enough space left in the buffer for at least
// the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
// the actual length of this packet
   byte PT = rx_data[packet_index + 3];
// Do some bit-level manipulation to determine if the packet contains data and if it is a batch
// We have to do this because the individual bits in the PT byte specify the contents of the
// packet.
   byte packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
   byte packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
   byte batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)
// Now finally figure out the actual packet length
   byte data_length = 0;
   if ( packet_has_data ) {
      if ( packet_is_batch ) {
// Packet has data and is a batch. This means it contains `batch_length' registers, each
// of which has a length of 4 bytes
         data_length = 4*batch_length;
      } else { // Packet has data but is not a batch. This means it contains one register (4 bytes) 
         data_length = 4;
      }
   } else { // Packet has no data
      data_length = 0;
   }
// At this point, we know exactly how long the packet is. Now we can check to make sure
// we have enough data for the full packet.
   if ( (rx_length - packet_index) < (data_length + 5) ) {
      return 3;
   }
// If we get here, we know that we have a full packet in the buffer. All that remains is to pull
// out the data and make sure the checksum is good.
// Start by extracting all the data
   packet->Address = rx_data[packet_index + 4];
   packet->PT = PT;
// Get the data bytes and compute the checksum all in one step
   packet->data_length = data_length;
   unsigned int computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address;
   for ( index = 0; index < data_length; index++ ) {
// Copy the data into the packet structure's data array
      packet->data[index] = rx_data[packet_index + 5 + index];
// Add the new byte to the checksum
      computed_checksum += packet->data[index];
   }
// Now see if our computed checksum matches the received checksum
// First extract the checksum from the packet
   unsigned int received_checksum = (rx_data[packet_index + 5 + data_length] << 8);
   received_checksum |= rx_data[packet_index + 6 + data_length];
// Now check to see if they don't match
   if ( received_checksum != computed_checksum ) {
      return 4;
   }
// At this point, we've received a full packet with a good checksum. It is already
// fully parsed and copied to the `packet' structure, so return 0 to indicate that a packet was
// processed.
   return 0;
}
