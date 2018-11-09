// Our Arduino code for operation of the Capella DNT900P-based ground station.

#include <ALTAIR_DNT900.h>
  
const byte     dntHwResetPin                   =      4;
const byte     dntCTSPin                       =      5;
const byte     dntRTSPin                       =      6;
long           previousMillis                  =      0;

ALTAIR_DNT900  theDNT900(1, dntHwResetPin,                 // on Serial1 -- 910 MHz, Capella antenna: 6-element 63 cm Yagi, approx. 9 dBi
                         dntCTSPin, dntRTSPin)         ;

void setup() {

  Serial.begin(38400);

  Serial.println(F("Starting DNT900 radio setup..."));
  if (!theDNT900.initialize()) {
    Serial.println(F("DNT900 radio init failed"));
    while(1);
  }
  Serial.println(F("DNT900 radio setup complete."));

  delay(100);

}

void loop() {
  byte downwardCommand[2];
  delay(100);
  theDNT900.readALTAIRInfo( downwardCommand, true );       // ignore any downward commands, for now
  sendCommandsToALTAIRAtInterval(2000);
}

void sendCommandsToALTAIRAtInterval(long interval)
{
  unsigned long currentMillis = millis();
//  if (digitalRead(shxBusyPin) == LOW && currentMillis - previousMillis3 > interval) {
  if (currentMillis - previousMillis > interval) { 
    previousMillis = currentMillis;

    byte    headerByte1, headerByte2, inputByte1, inputByte2;
    if (Serial.available()) {
       headerByte1 = Serial.read();
       if (headerByte1 == 'C') {
          headerByte2 = Serial.read();
          if (headerByte2 == '2') {
             inputByte1 = Serial.read();
             inputByte2 = Serial.read();
             // send the info via the DNT
             theDNT900.sendCommandToALTAIR(inputByte1, inputByte2);
          }
       }
    }

  }
}

