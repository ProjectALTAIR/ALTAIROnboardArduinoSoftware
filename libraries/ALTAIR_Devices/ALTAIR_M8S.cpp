/**************************************************************************/
/*!
    @file     ALTAIR_M8S.cpp
    @author   Colton Broughton (cbroughton@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR Raveon M8S
    radio transceiver, which can operate at a range of frequencies.  
    This class derives from the ALTAIR_GenTelInt generic telemetry 
    interface base class.
    Colton Broughton cbroughton@uvic.ca   began on 2021 November 1

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#include "ALTAIR_M8S.h"

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_M8S::ALTAIR_M8S():
    _rxdPin(M8S_DEFAULT_RXD_PIN), 
    _txdPin(M8S_DEFAULT_TXD_PIN),
    _ctsPin(M8S_DEFAULT_CTS_PIN),
    _rtsPin(M8S_DEFAULT_RTS_PIN),
    _rssiPin(M8S_DEFAULT_RSSI_PIN),
    _baudRate(M8S_DEFAULT_BAUD_RATE),
    _txOnPin(M8S_DEFAULT_TXON_PIN),
    _serialID(M8S_DEFAULT_SERIAL_ID)
{
}

/**************************************************************************/
/*!
 @brief  Constructor.
*/
/**************************************************************************/
ALTAIR_M8S::ALTAIR_M8S(const char rxdPin, 
                    const char txdPin, 
                    const char rssiPin, 
                    const char ctsPin, 
                    const char rtsPin,
                    const char txOnPin,
                    const char baudRate,
                    const char serialID) :
    _rxdPin(rxdPin), 
    _txdPin(txdPin),
    _ctsPin(ctsPin),
    _rtsPin(rtsPin),
    _rssiPin(rssiPin),
    _baudRate(baudRate),
    _txOnPin(txOnPin),
    _serialID(serialID) 
{
}

/**************************************************************************/
/*!
 @brief  reading a single byte from the M8S transceivers RX buffer
*/
/**************************************************************************/
byte ALTAIR_M8S::read() {
    /*
    byte c;
    switch (_serialID) {
      case 0:
        c = Serial.read();
        break;
      case 1:
        c = Serial1.read();
        break;
      case 2:
        c = Serial2.read();
        break;
      case 3:
        c = Serial3.read();
        break;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
        while(1);
    }
    return c;
    */

   byte c;
   c = SerialM8S.read();
   return c;
}

/**************************************************************************/
/*!
 @brief  Reads and displays (through Serial) each byte in the M8S 
            transceiver's RX buffer. ie. prints the current rx buffer
            on the serial monitor. If the buffer is extended before 
            this method finishes, it will display the extended portion 
            as well
*/
/**************************************************************************/
void ALTAIR_M8S::readSerial() {
    long readTry = 0;
    while (true) {
        while (!available() && readTry < MAX_READ_TRIES) {++readTry; delay(5);}
        if (readTry < MAX_READ_TRIES) {
            do {
                char c = read();
                print(c);
            } while (available());
        } 
        else {
            break;
        }
    }
}

/**************************************************************************/
/*!
 @brief  prints a single character to the Serial monitor
*/
/**************************************************************************/
void ALTAIR_M8S::print(char c) {  
    Serial.print(c);
}

/**************************************************************************/
/*!
 @brief  Boolean check if RX buffer is non-empty. If buffer > 0 available 
            returns '1'
*/
/**************************************************************************/
bool ALTAIR_M8S::available() {
    /*
    byte c;
    switch (_serialID) {
      case 0:
        c = Serial.available();
        break;
      case 1:
        c = Serial1.available();
        break;
      case 2:
        c = Serial2.available();
        break;
      case 3:
        c = Serial3.available();
        break;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
        while(1);
    }
    */

    byte c;
    c = SerialM8S.available();

    return (bool)c;
}

/**************************************************************************/
/*!
 @brief  sends a single character to the M8S transceivers TX pin to await
            transmission. In packet mode, this will occur when the packet 
            is 'full' or after a defined delay. 
            Packet size can be set with the 'ATTT' command (0-512 default: 80)
*/
/**************************************************************************/
bool ALTAIR_M8S::send(unsigned char aChar) {
    digitalWrite(_rtsPin, HIGH); // Sending request to send data to the M8S transceiver.
    // the following while loop is checking the buffer capacity. If it is above a certain 
    // amount the CTS pin is negated. (which stops this program from sending more data to 
    // the buffer) and waits for the buffer to be flushed.
    while (digitalRead(_ctsPin)!=0) {switch (_serialID) {case 0 : Serial.flush(); return 0; case 1: Serial1.flush(); return 0; case 2: Serial2.flush(); return 0; case 3: Serial3.flush(); return 0;}}
    switch (_serialID) {     
        case 0:
            Serial.write(aChar);
            Serial.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
            return 1;
        case 1:
            Serial1.write(aChar);
            Serial1.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
            return 1;
        case 2:
            Serial2.write(aChar);
            Serial2.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
            return 1;
        case 3:
            Serial3.write(aChar);
            Serial3.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
            return 1;
        default:
            Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
            return 0;
        }
    digitalWrite(_rtsPin, LOW); // we have sent our data and now we can stop the RTS
}
    

/**************************************************************************/
/*!
 @brief  sends a string of characters to the TX pin.
*/
/**************************************************************************/
bool ALTAIR_M8S::send(const uint8_t* aString) {
    digitalWrite(_rtsPin, HIGH); // Sending request to send data to the M8S transceiver.
    // the following while loop is checking the buffer capacity. If it is above a certain 
    // amount the CTS pin is negated. (which stops this program from sending more data to 
    // the buffer) and waits for the buffer to be flushed.    
    while (digitalRead(_ctsPin)!=0) {switch (_serialID) {case 0 : Serial.flush(); return 0; case 1: Serial1.flush(); return 0; case 2: Serial2.flush(); return 0; case 3: Serial3.flush(); return 0;}}
    switch (_serialID) {
      case 0:
        Serial.write((const char*) aString);
        Serial.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 1:
        Serial1.write((const char*) aString);
        Serial1.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 2:
        Serial2.write((const char*) aString);
        Serial2.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 3:
        Serial3.write((const char*) aString);
        Serial3.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
        return 0;
    }
    digitalWrite(_rtsPin, LOW); // we have sent our data and now we can stop the RTS
}

/**************************************************************************/
/*!
 @brief  sends an array of characters to the TX pin.
*/
/**************************************************************************/
bool ALTAIR_M8S::send(const uint8_t* anArray, const uint8_t arrayLen) {
    digitalWrite(_rtsPin, HIGH); // Sending request to send data to the M8S transceiver.
    // the following while loop is checking the buffer capacity. If it is above a certain 
    // amount the CTS pin is negated. (which stops this program from sending more data to 
    // the buffer) and waits for the buffer to be flushed.    
    while (digitalRead(_ctsPin)!=0) {switch (_serialID) {case 0: Serial.flush(); return 0; case 1: Serial1.flush(); return 0; case 2: Serial2.flush(); return 0; case 3: Serial3.flush(); return 0;}}
    switch (_serialID) {
      case 0:
        Serial.write(anArray, arrayLen); 
        Serial.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 1:
        Serial1.write(anArray, arrayLen);
        Serial1.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 2:
        Serial2.write(anArray, arrayLen);
        Serial2.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      case 3:
        Serial3.write(anArray, arrayLen);
        Serial3.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
        return 1;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
        return 0;
    }
    digitalWrite(_rtsPin, LOW); // we have sent our data and now we can stop the RTS
}

/**************************************************************************/
/*!
 @brief  takes a string and sends each character individually (mimicking 
            sending one character)
*/
/**************************************************************************/
bool ALTAIR_M8S::sendAsIndivChars(const uint8_t* aString) {
    digitalWrite(_rtsPin, HIGH); // Sending request to send data to the M8S transceiver.
    
    // the following while loop is checking the buffer capacity. If it is above a certain 
    // amount the CTS pin is negated. (which stops this program from sending more data to 
    // the buffer) and waits for the buffer to be flushed.
    while (digitalRead(_ctsPin)!=0) {switch (_serialID) {case 0 : Serial.flush(); return 0; case 1: Serial1.flush(); return 0; case 2: Serial2.flush(); return 0; case 3: Serial3.flush(); return 0;}}

    for (int i = 0; aString[i] != 0; ++i) {
        switch (_serialID) {
            case 0:
                Serial.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                Serial.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
                break;
            case 1:
                Serial1.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                Serial1.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
                break;
            case 2:
                Serial2.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                Serial2.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
                break;
            case 3:
                Serial3.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                Serial3.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)
                break;
            default:
                Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
                break;
        }
        delay(50);
    }
    digitalWrite(_rtsPin, LOW); // we have sent our data and now we can stop the RTS
}

/**************************************************************************/
/*!
 @brief  takes a string and sends each character individually (mimicking 
            sending one character) for command mode use ONLY
*/
/**************************************************************************/
bool ALTAIR_M8S::sendCMDAsIndivChars(const uint8_t* aString) {
    for (int i = 0; aString[i] != 0; ++i) {
        switch (_serialID) {
            case 0:
                Serial.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                break;
            case 1:
                Serial1.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                break;
            case 2:
                Serial2.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                break;
            case 3:
                Serial3.write(aString[i]); // writes a string letter by letter to the M8S_SERIAL
                break;
            default:
                Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver!"));
                break;
        }
        delay(50);
    }
}

/**************************************************************************/
/*!
 @brief  Transmit test sends a test phrase of CALL_SIGN_MESSAGE followed by
            END_MESSAGE_STRING which is defined in the parent class.
*/
/**************************************************************************/
void ALTAIR_M8S::transmitTest() {
    for (const char c : CALL_SIGN_STRING)   { send(byte(c)); } // transmit the callsign over radio " VE7XJA STATION ALTAIR "
    for (const char c : END_MESSAGE_STRING) { send(byte(c)); } // transmits the end message over read " OVER "
}

/**************************************************************************/
/*!
 @brief  Puts the M8S Transceiver into command mode. The output will read:
           'Raveon M8S
            
            OK<cr><lf>'
            if it is successful. Once in command mode, you can program it 
            using the AT commands listed in the Data Manual.
*/
/**************************************************************************/
void ALTAIR_M8S::cmdMode() {
    /*
    delay(1000);    // nominally only needs 500ms silence, using 1000ms so it for sure has silence before transmission (BT)
    switch (_serialID) {
        case 0:
            Serial.write('+');
            Serial.write('+');
            Serial.write('+');
            break;
        case 1:
            Serial1.write('+');
            Serial1.write('+');
            Serial1.write('+');
            break;
        case 2:
            Serial2.write('+');
            Serial2.write('+');
            Serial2.write('+');
            break;
        case 3:
            Serial3.write('+');
            Serial3.write('+');
            Serial3.write('+');
            break;
    }
    */

   int current_millis = millis();
   int start_millis = current_millis;
   while(current_millis < start_millis + 1000) { current_millis = millis(); }   // nominally only needs 500ms silence, using 1000ms so it for sure has silence before transmission (BT)

   SerialM8S.write('+');
   SerialM8S.write('+');
   SerialM8S.write('+');

   current_millis = millis();
   start_millis = current_millis;
   while(current_millis < start_millis + 750) { current_millis = millis(); }  // following the command sequence you need minimum 500ms of silence.
   readSerial();   // reading serial rx buffer to ensure command mode was successfully entered.
}


/**************************************************************************/
/*!
 @brief  sends a single character to the M8S transceivers TX pin to program 
         the transceiver. (MUST be in command mode.)
*/
/**************************************************************************/
void ALTAIR_M8S::sendCmd(const uint8_t* aString) {
    sendCMDAsIndivChars(aString);
    /*
    switch (_serialID) {
        case 0:
            Serial.write('\r');
            break;
        case 1:
            Serial1.write('\r');
            break;
        case 2:
            Serial2.write('\r');
            break;
        case 3:
            Serial3.write('\r');
            break;
    }
    */
   SerialM8S.write('\r');
    readSerial();
}
/**************************************************************************/
/*!
 @brief  exits command mode
*/
/**************************************************************************/
void ALTAIR_M8S::cmdModeExit() {
    // Exits command mode by sending the EXIT<CR> command
    delay(100);
    switch (_serialID) {
        case 0:
            Serial.write('E');
            Serial.write('X');
            Serial.write('I');
            Serial.write('T');
            Serial.write('\r');
            break;
        case 1:
            Serial1.write('E');
            Serial1.write('X');
            Serial1.write('I');
            Serial1.write('T');
            Serial1.write('\r');
            break;
        case 2:
            Serial2.write('E');
            Serial2.write('X');
            Serial2.write('I');
            Serial2.write('T');
            Serial2.write('\r');
            break;
        case 3:
            Serial3.write('E');
            Serial3.write('X');
            Serial3.write('I');
            Serial3.write('T');
            Serial3.write('\r');
            break;
    }
    delay(100); 
    readSerial();
}

/**************************************************************************/
/*!
 @brief  Checks to see if the transceiver is transmitting or not. 
            returns 1 if transmitting and 0 otherwise.
*/
/**************************************************************************/
bool ALTAIR_M8S::isBusy() { // the module can still be reading, it only differentiates if it is transmitting or not
    return (digitalRead((int)_txOnPin) == HIGH);
}

/**************************************************************************/
/*!
 @brief  Sets up the board pins to there respective inputs or outputs and 
            begins the correct serial channel based on _serialID.
            Returns 1 when complete.
*/
/**************************************************************************/
bool ALTAIR_M8S::initialize(const char* aString) {
    pinMode(_rxdPin,   INPUT  );
    pinMode(_ctsPin,   INPUT  );
    pinMode(_rtsPin,   OUTPUT );
    pinMode(_txOnPin,  INPUT  );
    pinMode(_txdPin,   OUTPUT );
    pinMode(_rssiPin,  INPUT  );
    
    SerialM8S.begin(_baudRate);
    
    /*
    switch (_serialID) {
        case 0:
            Serial.begin(_baudRate);
            break;
        case 1:
            Serial1.begin(_baudRate);
            break;
        case 2:
            Serial2.begin(_baudRate);
            break;
        case 3:
            Serial3.begin(_baudRate);
            break;
        default:
            Serial.println(F("Unallowed serial ID provided in initialization of M8S radio transceiver! (Initialize)"));
            return false;
    }
    */
    return true;
}

/**************************************************************************/
/*!
 @brief  returns the radio transceiver's name "M8S"
*/
/**************************************************************************/
const char* ALTAIR_M8S::radioName() {
    return M8S_RADIO_NAME;
}

/**************************************************************************/
/*!
 @brief  returns the radio_t type defined in GenTelInts enum.
*/
/**************************************************************************/
radio_t ALTAIR_M8S::radioType() {
    return m8s;
}

/**************************************************************************/
/*!
 @brief  Enters command mode, and prints out the RSSI value, then exits 
            command mode.
*/
/**************************************************************************/
char ALTAIR_M8S::lastRSSI() {
    cmdMode();
    readSerial();
    sendCMDAsIndivChars("ATRS ");
    sendCmd('\r');
    readSerial();
    cmdModeExit();
    readSerial();
    return 1;
}

