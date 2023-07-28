/**************************************************************************/
/*!
    @file     ALTAIR_M8S.h
    @author   Colton Broughton <cbroughton@uvic.ca>
    @license  GPL

    This is the telemetry interface class for the ALTAIR Raveon M8S
    radio transceiver, which can operate at a range of frequencies.  
    This class derives from the ALTAIR_GenTelInt generic telemetry 
    interface base class.

    Colton Broughton  cbroughton@uvic.ca     began on 2021 November 01

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef ALTAIR_M8S_h
#define ALTAIR_M8S_h

#include "ALTAIR_GenTelInt.h"

#define M8S_RADIO_NAME              "M8S"  // the radio name for identifying which transceiver is being used
#define M8S_CMDMD                   "+++"  // command mode sequence for putting the M8S into command mode
#define M8S_EXIT                    "EXIT" // command mode exit sequence for exiting from command mode

/* define's for Arduino Mega
#define M8S_DEFAULT_RXD_PIN            19  // pin 6  on M8S is the RX, ie from m8s to arduino to RX data
#define M8S_DEFAULT_TXD_PIN            18  // pin 5  on M8S is the TX, ie from arduino to m8s to TX data
#define M8S_DEFAULT_CTS_PIN            41  // pin 9  on M8S is the Clear To Send signal.
#define M8S_DEFAULT_RTS_PIN            33  // pin 10 on m8s is the request to send signal.
#define M8S_DEFAULT_RSSI_PIN           51  // pin 11 on M8S is the Receive Signal Strength Indicator 
#define M8S_DEFAULT_TXON_PIN           22  // pin 4  on M8S is the TX On pin that is HIGH when transmitting, LOW otherwise.
#define M8S_DEFAULT_BAUD_RATE        9600  // the default serial baud rate on the M8S
#define M8S_DEFAULT_SERIAL_ID           3  // default wiring has the M8S connected to Serial1 (leaving Serial open for the serial monitor)
*/

// Pin Definitions for Grand Central PRELIMINARY
#define M8S_DEFAULT_RXD_PIN            15  // pin 6  on M8S is the RX, ie from m8s to arduino to RX data
#define M8S_DEFAULT_TXD_PIN            14  // pin 5  on M8S is the TX, ie from arduino to m8s to TX data
#define M8S_DEFAULT_CTS_PIN            23  // pin 9  on M8S is the Clear To Send signal.
#define M8S_DEFAULT_RTS_PIN            22  // pin 10 on m8s is the request to send signal.
#define M8S_DEFAULT_RSSI_PIN           25  // pin 11 on M8S is the Receive Signal Strength Indicator 
#define M8S_DEFAULT_TXON_PIN           24  // pin 4  on M8S is the TX On pin that is HIGH when transmitting, LOW otherwise.

#define M8S_DEFAULT_BAUD_RATE        9600  // the default serial baud rate on the M8S
#define M8S_DEFAULT_SERIAL_ID           3  // default wiring has the M8S connected to Serial1 (leaving Serial open for the serial monitor)


class ALTAIR_M8S : public ALTAIR_GenTelInt {
    public:
        virtual bool send(unsigned char aChar                   );  // for sending a single character.
        virtual bool send(const uint8_t* aString                );  // for sending a string of characters
        virtual bool send(const uint8_t* anArray ,
                          const uint8_t arrayLen                );  // sending an array of characters 'anArray' of length 'arrayLen'
        virtual bool sendAsIndivChars(const uint8_t* aString    );  // sends a string as individual characters
        virtual bool sendCMDAsIndivChars(const uint8_t* aString    );  // sends a string as individual characters

        virtual bool available(                                 );  // if there is data waiting in the serial buffer, ie. the transceiver has received data. 
        virtual bool isBusy(                                    );  // for the M8S transceiver, a busy signal ONLY signifies if the unit is Transmitting and NOT if
                                                                    //      it is receiving.
        virtual bool initialize(  const char*    aString = ""   );  // initialization method for setting pinModes.
        virtual byte read(                                      );  // reading a byte from the RX pin. 
        virtual const char* radioName(                          );  // returning the radio name.
        virtual radio_t radioType(                              );  // what is the radio type? (ENUM must be updated to include "m8s" in GenTelInt)
        virtual char lastRSSI(                                  );  // puts device in command mode, sends ATRS command and reads the output, the RSSI.
        virtual bool lastSentString2()          { return true ; }   // included so this does not become an abstract class.
        void sendCmd(const uint8_t* aString                     );
        void readSerial(                                        );  // method for reading the entirety of awaiting rx buffer.
        void print(char                                         );  // printing a char to the serial monitor.
        void transmitTest(                                      );  // sending the callsign message and end message from genTelInt
        void cmdMode(                                           );  // putting the m8s into command mode. 
        void cmdModeExit(                                       );  // taking the  m8s out of command mode.
        ALTAIR_M8S(                                             );  // empty constructor arguments assumes default constructor values
        ALTAIR_M8S(const char rxdPin, 
                   const char txdPin, 
                   const char rssiPin, 
                   const char ctsPin,
		           const char rtsPin,
                   const char txOnPin,
                   const char baudRate,
                   const char serialID                          );  // non-empty constructor sets specific constructor values.

    private:
        char _rxdPin             ;  // pin 6  on M8S is the RX, ie from m8s to arduino to RX data
        char _txdPin             ;  // pin 5  on M8S is the TX, ie from arduino to m8s to TX data
        char _ctsPin             ;  // pin 9  on M8S is the Clear To Send signal.
	    char _rtsPin		     ;  // pin 10 on m8s is the Request to Send signal.
        char _rssiPin            ;  // pin 11 on M8S is the Receive Signal Strength Indicator 
        char _txOnPin            ;  // pin 4  on M8S HIGH when transmitting, LOW otherwise.
        int  _baudRate           ;  // the default serial baud rate on the M8S
        char _serialID           ;  // the serial number the transceiver RX/TX is connected to. (ie. serial 0,1,2,3 for the mega2560 boards)
    
};
#endif