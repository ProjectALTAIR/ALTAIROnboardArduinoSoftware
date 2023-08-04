//#include <ALTAIR_M8S.h>

#include <wiring_private.h>

/*// Pin Definitions for Grand Central PRELIMINARY
#define M8S_DEFAULT_RXD_PIN            15  // pin 6  on M8S is the RX, ie from m8s to arduino to RX data
#define M8S_DEFAULT_TXD_PIN            14  // pin 5  on M8S is the TX, ie from arduino to m8s to TX data
#define M8S_DEFAULT_CTS_PIN            23  // pin 9  on M8S is the Clear To Send signal.
#define M8S_DEFAULT_RTS_PIN            22  // pin 10 on m8s is the request to send signal.
#define M8S_DEFAULT_RSSI_PIN           25  // pin 11 on M8S is the Receive Signal Strength Indicator 
#define M8S_DEFAULT_TXON_PIN           24  // pin 4  on M8S is the TX On pin that is HIGH when transmitting, LOW otherwise.
*/

char _rxdPin      = 15       ;  // pin 6  on M8S is the RX, ie from m8s to arduino to RX data
char _txdPin      = 14       ;  // pin 5  on M8S is the TX, ie from arduino to m8s to TX data
char _ctsPin      = 23       ;  // pin 9  on M8S is the Clear To Send signal.
char _rtsPin		  = 22       ;  // pin 10 on m8s is the Request to Send signal.
char _rssiPin     = 25       ;  // pin 11 on M8S is the Receive Signal Strength Indicator 
char _txOnPin     = 24       ;  // pin 4  on M8S HIGH when transmitting, LOW otherwise.

/*
  Additional Serial setup for Grand Central
  Per GenTelInt M8S has identifier '3' --> TX3 pin 14 (PB16) / RX3 on pin 15 (PB17)
  Serial_i is on RX/TX_i-1. Therefore for identifier 3, Serial4 is needed. (per Serial1 on RX/TX)
  Uses SERCOM5
    From variants.h:
      // Serial4
      #define PIN_SERIAL4_RX      (15)
      #define PIN_SERIAL4_TX      (14)
      #define PAD_SERIAL4_TX      (UART_TX_PAD_0)
      #define PAD_SERIAL4_RX      (SERCOM_RX_PAD_1)
      #define SERCOM_SERIAL4		  sercom5
*/
  Uart SerialM8S( &SERCOM_SERIAL4, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX ) ;
  // Interrupt handler
  void SERCOM5_0_Handler()
  {
    SerialM8S.IrqHandler();
  } 
  void SERCOM5_1_Handler()
  {
    SerialM8S.IrqHandler();
  } 
  void SERCOM5_2_Handler()
  {
    SerialM8S.IrqHandler();
  } 
  void SERCOM5_3_Handler()
  {
    SerialM8S.IrqHandler();
  } 


//unsigned long previousMillis = 0;
//ALTAIR_M8S theM8S;


uint8_t c;
uint8_t data;


void setup() {
  /*
  Serial.begin(38400);
  Serial.println(F("Starting M8S setup..."));
  if (!theM8S.initialize()) {
    Serial.println(F("M8S init failed"));
    while(1);
  }
  theM8S.cmdMode();
  theM8S.sendCmd("STAT 1");  // "STAT" returns a statistics overview, 
                             // "STAT {1..4}" outputs different statistics from M8S 
                             // "STAT 9" resets statistics
  theM8S.cmdModeExit();
  delay(100);
  Serial.println(F("M8S Setup complete"));
  */

  //pinMode(14, OUTPUT);
  //pinMode(15, INPUT);
    
    pinMode(_rxdPin,   INPUT  );
    pinMode(_ctsPin,   INPUT  );
    pinMode(_rtsPin,   OUTPUT );
    pinMode(_txOnPin,  INPUT  );
    pinMode(_txdPin,   OUTPUT );
    pinMode(_rssiPin,  INPUT  );

  pinMode(40, OUTPUT);

  Serial.begin(9600);
  SerialM8S.begin(9600);

  //pinPeripheral(PIN_SERIAL4_RX, PIO_SERCOM);
  //pinPeripheral(PIN_SERIAL4_TX, PIO_SERCOM);

  delay(5000);
  Serial.println("Starting");

  pinMode(LED_BUILTIN, OUTPUT);

}

bool send(unsigned char aChar) {
    digitalWrite(40, HIGH);
    digitalWrite(_rtsPin, HIGH); // Sending request to send data to the M8S transceiver.
    // the following while loop is checking the buffer capacity. If it is above a certain 
    // amount the CTS pin is negated. (which stops this program from sending more data to 
    // the buffer) and waits for the buffer to be flushed.
    while (digitalRead(_ctsPin)!=0) { SerialM8S.flush();}
    SerialM8S.write(aChar);
    SerialM8S.flush(); // Waits for the transmission of outgoing serial data to complete. (Prior to Arduino 1.0, this instead removed any buffered incoming serial data.)

    delay(5);
    digitalWrite(_rtsPin, LOW); // we have sent our data and now we can stop the RTS
    digitalWrite(40, LOW);
    return 1;
}


void loop() {
  /*
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 5000) {
    previousMillis = currentMillis;
    theM8S.transmitTest();
  }
  theM8S.readSerial();
//  theM8S.lastRSSI();
  */
  Serial.println("here we go");
  for (data = 0; data < 256; data++) {
  
  //SerialM8S.write(data);
  //Serial.println("written");
  //delay(100);
  //while( !SerialM8S.available()){/*Serial.println("unavailable");*/}
  //Serial.println("received");
  //c = SerialM8S.read();
  //Serial.println(c, DEC);

  if (send(data)!=0) {
    Serial.println("written");
    delay(50);
  }
  }
  
  /*int current_millis = millis();
  int start_millis = current_millis;
  while (current_millis < start_millis + 1000) { SerialM8S.write((uint8_t) 1); delay(100);}
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  current_millis = millis();
  start_millis = current_millis;
  while (current_millis < start_millis + 1000) { SerialM8S.write((uint8_t) 1); delay(100);}
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  */
  /*for (data = 0; data < 256; data++) {
    SerialM8S.write(data);
    Serial.println("written");
    delay(100);
  }*/
}


