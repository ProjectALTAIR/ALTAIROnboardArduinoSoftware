// Our Arduino code for operation of the Betelgeuse DNT900 telemetry test box
#include <TinyGPS.h>
#include <SoftwareSerial.h>

TinyGPS        gps;

const byte     dntHwResetPin            =  4;
const byte     dntCTSPin                =  5;
const byte     dntRTSPin                =  6;
// const byte     dntCTSPin                =  42;
// const byte     dntRTSPin                =  43;
const byte     gpsSuccessfullyLockedPin =  7;
const byte     fakeShxProgramRxPin      =  8;
const byte     shxProgramPin            =  9;
const byte     shxBusyPin               = 10;

SoftwareSerial shxProgramSerial(fakeShxProgramRxPin, shxProgramPin);

const long     dntMaxReadTries          =  500000;
const int      maxTermLength            =  255;

byte           term[maxTermLength];
int            termIndex                =  0;
int            termLength               =  0;
  
long           previousMillis           =  0;
long           previousMillis2          =  0;
long           previousMillis3          =  0;
int            count                    =  0;

// EnterProtocolMode DNTCFG
const byte     enterProtocol[]       = {0xFB, 0x07, 0x00, 0x44, 0x4E, 0x54, 0x43, 0x46, 0x47};
  
const byte     UcReset[]             = {0xFB, 0x05, 0x04, 0x00, 0xFF, 0x01, 0x00};
const byte     SwReset[]             = {0xFB, 0x02, 0x02, 0x00};

const byte     makeRemote[]          = {0xFB, 0x05, 0x04, 0x00, 0x00, 0x01, 0x00};
const byte     makeBase[]            = {0xFB, 0x05, 0x04, 0x00, 0x00, 0x01, 0x01};

const byte     setDataRate500[]      = {0xFB, 0x05, 0x04, 0x01, 0x00, 0x01, 0x00};
const byte     setDataRate115p2[]    = {0xFB, 0x05, 0x04, 0x01, 0x00, 0x01, 0x02};
const byte     setDataRate38p4[]     = {0xFB, 0x05, 0x04, 0x01, 0x00, 0x01, 0x03};
const byte     setPower1mW[]         = {0xFB, 0x05, 0x04, 0x18, 0x00, 0x01, 0x00};
const byte     setPower1Watt[]       = {0xFB, 0x05, 0x04, 0x18, 0x00, 0x01, 0x05};

const byte     loadEEPROMdef[]       = {0xFB, 0x05, 0x04, 0xFF, 0xFF, 0x01, 0x00};
const byte     saveToEEPROM[]        = {0xFB, 0x05, 0x04, 0xFF, 0xFF, 0x01, 0x01};
const byte     saveEEPROMandReset[]  = {0xFB, 0x05, 0x04, 0xFF, 0xFF, 0x01, 0x02};


void setup() {
  pinMode(                  dntHwResetPin, OUTPUT);
  pinMode(                      dntCTSPin, INPUT);
  pinMode(                      dntRTSPin, OUTPUT);
  pinMode(       gpsSuccessfullyLockedPin, OUTPUT);
  pinMode(            fakeShxProgramRxPin, INPUT);
  pinMode(                  shxProgramPin, OUTPUT);
  pinMode(                     shxBusyPin, INPUT);
  
  digitalWrite(             dntHwResetPin, LOW);
//  digitalWrite(                 dntRTSPin, LOW);
  digitalWrite(                 dntRTSPin, HIGH);
  digitalWrite(  gpsSuccessfullyLockedPin, LOW);

  Serial1.begin(38400);
  Serial.begin(9600);

  Serial2.begin(38400);

  Serial.println(F("Starting SHX1 serial modem setup..."));
  shxProgramSerial.begin(2400);
  
//  delay(200);

  shxProgramSerial.write(byte('S'));
  shxProgramSerial.write(byte('E'));
  shxProgramSerial.write(byte('T'));
  shxProgramSerial.write(byte('M'));
  shxProgramSerial.write(byte('O'));
  shxProgramSerial.write(byte('D'));
  shxProgramSerial.write(byte('\r'));               // '\r' = ASCII 13 = carriage return

  delay(200);

  shxProgramSerial.end();
  digitalWrite(             shxProgramPin, HIGH);  // The PGM pin is _active LOW_, so one must return it to its default
                                                   //  (i.e.: internal 47k pull-up to 4V) HIGH state in order to move
                                                   //  from programming mode to serial modem mode.
  Serial.println(F("SHX1 serial modem setup complete."));

  Serial3.begin(1200);

// END the hardware reset (i.e., set HwResetPin HIGH).  A Hw reset must occur every time the radio is powered up.
  digitalWrite(             dntHwResetPin, HIGH);
  delay(100);

/*
  sendCmd(loadEEPROMdef);
  delay(100);
  sendCmd(saveToEEPROM);
  hardRadioReset();
*/

//  sendCmd(enterProtocol);
//  delay(100);

 // sendCmd(makeBase);
//  sendCmd(makeRemote);
//  sendCmd(saveEEPROMandReset);

//  sendCmd(setDataRate38p4);
//  delay(100);
//  sendCmd(setPower1mW);
//  sendCmd(setPower1Watt);


//  sendCmd(saveEEPROMandReset);
 //  hardRadioReset();

  // delay(100);

 // sendCmd(enterProtocol);

}

void loop() {


  if (getGPS()) {
    digitalWrite(gpsSuccessfullyLockedPin, LOW);
    Serial.println(F("gotten GPS"));
 //   delay(100);
    digitalWrite(gpsSuccessfullyLockedPin, HIGH);
  }
  else {
//    Serial.println(F("failed to get GPS"));
  }


  // Get the Betelgeuse radio's MAC address
//  testRadio();
//  readRadio();

//  delay(100);
  sendGPSToRadioAtInterval(1000);
//  delay(1000);


//  delay(100);
  sendStationNameToRadioAtInterval(1333);
//  delay(100);

//  readRadio();  


  sendStatusToComputerAtInterval(5000);
 

//  delay(100);

}

bool getGPS()
{
  while (Serial1.available())
  {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}

void testRadio()
{
  if (digitalRead(dntCTSPin) == LOW) {
    Serial.println(F("Requesting register containing MAC address info from DNT"));

//    digitalWrite(dntRTSPin, HIGH);

    Serial2.write(0xFB);
    Serial2.write(0x04);
    Serial2.write(0x03);
  
    // GetRegister MAC address (register 0 in bank 2, span 3 bytes)
    Serial2.write((byte)0x00);
    Serial2.write(0x02);
    Serial2.write(0x03);

//    digitalWrite(dntRTSPin, LOW);
  
  } else {
    Serial.println(F("Cannot get MAC address info from DNT, since the DNT CTS pin is HIGH"));
  }

}

void sendStationNameToRadioAtInterval(long interval)
{
  unsigned long currentMillis = millis();
//  if (digitalRead(shxBusyPin) == LOW && currentMillis - previousMillis3 > interval) {
  if (currentMillis - previousMillis3 > interval) { 
    Serial.println(F("Writing station name to SHX1"));
    previousMillis3 = currentMillis;

/*
    Serial2.write(0xFB);
    Serial2.write(0x0E);
    Serial2.write(0x05);
  
  //broadcast
//    Serial2.write((byte)0x00);
//    Serial2.write((byte)0x00);
//    Serial2.write((byte)0x00);
    Serial2.write((byte)0xFF);
    Serial2.write((byte)0xFF);
    Serial2.write((byte)0xFF);
*/
      
  //send station name
    Serial3.write(byte('B'));
    Serial3.write(byte('E'));
    Serial3.write(byte('T'));
    Serial3.write(byte('E'));
    Serial3.write(byte('L'));
    Serial3.write(byte('G'));
    Serial3.write(byte('E'));
    Serial3.write(byte('U'));
    Serial3.write(byte('S'));
    Serial3.write(byte('E'));

  } else if (Serial3.available()) {
    byte shxTerm[maxTermLength];
    int  shxTermIndex = 0;
    while (Serial3.available() && shxTermIndex < maxTermLength) {
      shxTerm[shxTermIndex++] = Serial3.read();
      delay(5);
    }
    Serial.print("  Number of SHX bytes: ");    Serial.println(shxTermIndex);
    Serial.print("  SHX Data: \"");
    for (int i = 0; i < shxTermIndex; i++)
    {
      Serial.print((char)shxTerm[i]);
    }
    Serial.println("\"");
    Serial.print("  SHX Data (HEX): \"");
    for (int i = 0; i < shxTermIndex; i++)
    {
      Serial.print(shxTerm[i], HEX); Serial.print(" ");
    }
    Serial.println("\"");  
  } else if (digitalRead(shxBusyPin) == HIGH) {
//    Serial.println(F("Cannot send the station name to SHX1, since the SHX1 Busy pin is HIGH, and the SHX1 is not available"));
  }
  
}


void sendGPSToRadioAtInterval(long interval)
{
  long latitude, longitude;
  unsigned long age, date, chars;
  unsigned short sentences, failed;
  byte month, day, hour, minute, second, hundredths;
  int year;

  gps.get_position(&latitude, &longitude, &age);

  if (getGPS()) {
    digitalWrite(gpsSuccessfullyLockedPin, LOW);
    Serial.println(F("gotten GPS at 2"));
    delay(100);
    digitalWrite(gpsSuccessfullyLockedPin, HIGH);
  }

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  if (getGPS()) {
    digitalWrite(gpsSuccessfullyLockedPin, LOW);
    Serial.println(F("gotten GPS at 3"));
    delay(100);
    digitalWrite(gpsSuccessfullyLockedPin, HIGH);
  }

  unsigned long currentMillis = millis();
  if (digitalRead(dntCTSPin) == LOW && currentMillis - previousMillis2 > interval) {
    Serial.println(F("*** Writing GPS to DNT ***"));
    previousMillis2 = currentMillis;

    writeToDNT(0xFA);
    writeToDNT(0x0B);
//    Serial2.write(0x05);
  
  //broadcast
//    Serial2.write((byte)0x00);
//    Serial2.write((byte)0x00);
//    Serial2.write((byte)0x00);
//    Serial2.write((byte)0xFF);
//    Serial2.write((byte)0xFF);
//    Serial2.write((byte)0xFF);
  
  //send time
    writeToDNT(hour);
    writeToDNT(minute);
    writeToDNT(second);
    
  //send latitude
    writeToDNT(byte((latitude >> 24) & 0xFF));
    writeToDNT(byte((latitude >> 16) & 0xFF));
    writeToDNT(byte((latitude >> 8) & 0xFF));
    writeToDNT(byte(latitude & 0xFF));
    
  //send longitude
    writeToDNT(byte((longitude >> 24) & 0xFF));
    writeToDNT(byte((longitude >> 16) & 0xFF));
    writeToDNT(byte((longitude >> 8) & 0xFF));
    writeToDNT(byte(longitude & 0xFF));

 //   Serial2.flush();

  } else if (digitalRead(dntCTSPin) == HIGH) {
    Serial.println(F("Cannot send the GPS to DNT, since the DNT CTS pin is HIGH"));
  }

  
  //send altitude
  signed long gps_altitude = gps.altitude();
      
  //send speed
  unsigned long gps_speed = gps.speed();
    
  //send HDOP
  unsigned long gps_hdop = gps.hdop();
  
}

void readRadio () {
  long dntReadTry = 0;
  int hasBegun = 0;
  termIndex = termLength = 0;
  
  Serial.println(F("Reading radio"));
  digitalWrite(dntRTSPin, LOW);
//  delay(100);
  
  while (true) {
    while (!Serial2.available() && dntReadTry < dntMaxReadTries) {
      ++dntReadTry;
//      Serial2.flush();
//      delay(5);
    }
    if (dntReadTry < dntMaxReadTries) {
      do {
//      Serial.println(F("Reading a byte from radio"));

        byte b = Serial2.read();
    
        if (b == (byte)0xFB && hasBegun == 0) {
          hasBegun = 1;
        }
        else if (hasBegun == 1) {
          termLength = (int)b;
          hasBegun = 2;
        }
        else if (hasBegun == 2) {
        //Serial.println(b, HEX);
          term[termIndex++] = b;
        } 
        if (termLength == termIndex && termLength > 0) break;
      } while (Serial2.available());
      if (termLength == termIndex && termLength > 0) break;

    
    } else {
//      digitalWrite(dntRTSPin, HIGH);
//      Serial2.flush();
      Serial.println(F("DNT is not available for reading"));
      break;
//      sendCmd(UcReset);

//      hardRadioReset();
//      delay(200);
//      sendCmd(enterProtocol);
//      delay(200);
    }

  }
  digitalWrite(dntRTSPin, HIGH);

  if (termLength == termIndex && termLength > 0) {
    switch ((byte)term[0]) {

      //Replies
      case (byte)0x10:
      Serial.println("EnterProtocolModeReply");
      //no other data
      break;
    
      case (byte)0x11:
      Serial.println("ExitProtocolModeReply");
      //no other data
      break;
    
      case (byte)0x12:
      Serial.println("SoftwareResetReply");
      //no other data
      break;
    
      case (byte)0x13:
      GetRegisterReply(term);
      break;
    
      case (byte)0x14:
      Serial.println("SetRegisterReply");
      break;
    
      case (byte)0x15:
      Serial.println("TxDataReply");
      TxDataReply(term);
      break;
    
      case (byte)0x1A:
      Serial.println("GetRemoteRegisterReply");
      break;
    
      case (byte)0x1B:
      Serial.println("SetRemoteRegisterReply");
      break;

      // Events
      case (byte)0x26:
      Serial.println("RxData");
      RxData(term, termLength);
      break;
    
      case (byte)0x27:
      Serial.println("Announce");
      Announce(term);
      break;
    
      case (byte)0x28:
      Serial.println("RxEvent");
      break;

      default:
      Serial.print("UNRECOGNISED MESSAGE: "); Serial.println(term[0], HEX);
    }
  
    Serial2.flush();

    termLength = termIndex = 0;    
  }

}

void GetRegisterReply (byte term[]) {
  Serial.println("GetRegisterReply");
  
  Serial.print("  Reg: ");  Serial.println(term[1], HEX);
  Serial.print("  Bank: "); Serial.println(term[2], HEX);
  Serial.print("  Span: "); Serial.println(term[3], HEX);
  Serial.print("  Val: ");
  for (int i = 4; i < term[3] + 4; i++)
  {
    Serial.print(term[i], HEX);
  }
  Serial.println();
  
  Serial.print("  Val (ASCII): \"");
  for (int j = 4; j < term[3] + 4; j++)
  {
    Serial.print((char)term[j]);
  }
  Serial.println("\"");
}

void TxDataReply (byte term[]) {
  Serial.print("  TxStatus: ");
  switch ((byte)term[1]) {
  
    case (byte)0x00:
    Serial.println("ACK received");
    break;

    case (byte)0x01:
    Serial.println("NO ACK received");
    break;

    case (byte)0x02:
    Serial.println("Not linked (remote)");
    break;

    case (byte)0x03:
    Serial.println("NO ACK due to recipient holding for flow control");
    break;

    default:
    Serial.print("UNRECOGNISED TX STATUS: "); Serial.println(term[1], HEX);
  }
  
  Serial.print("  MAC: ");  Serial.print(term[4], HEX); Serial.print(term[3], HEX); Serial.println(term[2], HEX);
  
  Serial.print("  RSSI: "); Serial.println((char)term[5], DEC);
}

void RxData (byte term[], int termLength) {
  Serial.print("  Address: "); Serial.print(term[3], HEX); Serial.print(term[2], HEX); Serial.println(term[1], HEX);
  Serial.print("  RSSI: ");    Serial.println((char)term[4], DEC);

  Serial.print("  Data: \"");
  for (int i = 5; i < termLength; i++)
  {
    Serial.print((char)term[i]);
  }
  Serial.println("\"");
  
  Serial.print("  Data (HEX): \"");
  for (int i = 5; i < termLength; i++)
  {
    Serial.print(term[i], HEX);
  }
  Serial.println("\"");
}

void Announce (byte term[]) {
  switch ((byte)term[1]) {

// Use Serial.print(F("text")) and Serial.println(F("text"))  rather than  Serial.print("text") and Serial.println("text")
// to save lots of variable RAM!!! -- this is needed for this code to run on an Uno!!!

    case (byte)0xA0:
    Serial.println(F("  Radio has completed startup initialization"));
    break;
    
    case (byte)0xA2:
    Serial.println(F("  Base: a remote has joined the network"));
    Serial.print(F("  Remote MAC: ")); Serial.print(term[4], HEX); Serial.print(term[3], HEX); Serial.println(term[2], HEX);
    Serial.print(F("  Range: ")); Serial.println((unsigned int)term[6], DEC);
    break;
    
    case (byte)0xA3:
    Serial.println(F("  Remote: joined a network, ready for data"));
    Serial.print(F("  Network ID: ")); Serial.println(term[2], HEX);
    Serial.print(F("  Base MAC: ")); Serial.print(term[5], HEX); Serial.print(term[4], HEX); Serial.println(term[3], HEX);
    Serial.print(F("  Range: ")); Serial.println((unsigned int)term[6], DEC);
    break;
    
    case (byte)0xA4:
    Serial.println(F("  Remote: exited network (base is out of range)"));
    Serial.print(F("  Network ID: ")); Serial.println(term[2], HEX);
    break;
    
    case (byte)0xA5:
    Serial.println(F("  Remote: the base has been rebooted"));
    break;
    
    case (byte)0xA7:
    Serial.println(F("  Base: remote has left the network"));
    Serial.print(F("  Remote MAC: ")); Serial.print(term[4], HEX); Serial.print(term[3], HEX); Serial.println(term[2], HEX);
    break;
    
    case (byte)0xA8:
    Serial.println(F("  Base: heartbeat received from router or remote"));
    break;

    //Errors
    case (byte)0xE0:
    Serial.println(F("  Protocol error - invalid message type"));
    break;
    
    case (byte)0xE1:
    Serial.println(F("  Protocol error - invalid argument"));
    break;
    
    case (byte)0xE2:
    Serial.println(F("  Protocol error - general error"));
    break;
    
    case (byte)0xE3:
    Serial.println(F("  Protocol error - parser timeout"));
    break;
    
    case (byte)0xE4:
    Serial.println(F("  Protocol error - register is read-only"));
    break;
    
    case (byte)0xE8:
    Serial.println(F("  UART receive buffer overflow"));
    break;
    
    case (byte)0xE9:
    Serial.println(F("  UART receive overrun"));
    break;
    
    case (byte)0xEA:
    Serial.println(F("  UART framing error"));
    break;
    
    case (byte)0xEE:
    Serial.println(F("  hardware error"));
    break;
    
    default:
    Serial.print(F("UNRECOGNISED ANNOUNCE MESSAGE: ")); Serial.println(term[1], HEX);
  }
}

void sendStatusToComputerAtInterval(long interval) {

  long latitude, longitude;
  unsigned long age, date, chars;
  unsigned short sentences, failed;
  byte month, day, hour, minute, second, hundredths;
  int year;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   

//    testRadio();
//    readRadio();

    gps.get_position(&latitude, &longitude, &age);
    Serial.print(F("Betelgeuse Latitude: "));    Serial.println(latitude);
    Serial.print(F("Betelgeuse Longitude: "));   Serial.println(longitude);
    Serial.print(F("Betelgeuse LatLong Age: ")); Serial.println(age);
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    Serial.print(F("Betelgeuse Year: "));        Serial.println(year);
    Serial.print(F("Betelgeuse Month: "));       Serial.println(month);
    Serial.print(F("Betelgeuse Day: "));         Serial.println(day);
    Serial.print(F("Betelgeuse Hour: "));        Serial.println(hour);
    Serial.print(F("Betelgeuse Minute: "));      Serial.println(minute);
    Serial.print(F("Betelgeuse Second: "));      Serial.println(second);
    Serial.print(F("Betelgeuse Hundredth: "));   Serial.println(hundredths);
    Serial.print(F("Betelgeuse GPSTime Age: ")); Serial.println(age);    
  }


}

void sendCmd(const byte theArray[])
{
  if (digitalRead(dntCTSPin) == LOW) {
    Serial.println(F("Writing to DNT"));
    for (int i = 0; i < theArray[1] + 2; i++)
    {
      Serial2.write(theArray[i]);
    }
  } else {
    Serial.println(F("Cannot send the most recent command, since the DNT CTS pin is HIGH"));
  }
}

void sendCommand () {
  const byte lockTo1milliW[] = {0xFB, 0x05, 0x04, 0x18, 0x00, 0x01, 0x10};
  const byte   setDataRate[] = {0xFB, 0x05, 0x04, 0x01, 0x00, 0x01, 0x03};

  const byte  getXmitPower[] = {0xFB, 0x04, 0x03, 0x18, 0x00, 0x01};

// MAC addr = 0x000000, data = "pro."
  const byte  transmitData[] = {0xFB, 0x08, 0x05, 0x00, 0x00, 0x00, 0x70, 0x72, 0x6F, 0x2E};

// set InitialParentNwkID register to 0x01 (thus, base unit should have a network ID of 0x01)
  const byte   setProtocol[] = {0xFB, 0x05, 0x04, 0x00, 0x04, 0x01, 0x01};

  const byte amBaseOrRemte[] = {0xFB, 0x04, 0x03, 0x00, 0x00, 0x01};

//reg bank span val
  const byte   getRegister[] = {0xFB, 0x04, 0x03, 0x1C, 0x00, 0x10};
  
  if (Serial.available())
  {
    char input = Serial.read();
    
    switch (input)
    {
      //get register
      case (char)'a':
      sendCmd(getRegister);
      break;
      
      case (char)'d':
      sendDiagnostics();
      break;
      
      case (char)'l':
      sendCmd(getXmitPower);
      break;
      
      case (char)'L':
      sendCmd(lockTo1milliW);
      break;
      
      case (char)'D':
      sendCmd(setDataRate);
      break;

      //tx data
      case (char)'t':
      sendCmd(transmitData);
      break;

      //set protocol mode
      case (char)'p':
      sendCmd(setProtocol);
      break;

      //make remote
      case (char)'R':
      sendCmd(makeRemote);
      break;

      //make base
      case (char)'B':
      sendCmd(makeBase);
      break;

      //save to EEPROM
      case (char)'S':
      sendCmd(saveToEEPROM);
      break;

      //Enter protocol from transparent
      case (char)'E':
      sendCmd(enterProtocol);
      break;

      //Am I base or remote?
      case (char)'w':
      sendCmd(amBaseOrRemte);
      break;
      
      case (char)'r':
      // Soft Radio Reset
      sendCmd(UcReset);
      break;

      case (char)'f':
      hardRadioReset();
      break;
            
      default:
//      Serial.print("Length: "); Serial.println(termLength);
//      Serial.print("Index: ");  Serial.println(termIndex);
      break;
    }
  }
  
  Serial.flush();
}

void sendDiagnostics()
{
    const byte   diagnostics[] = {0xFB, 0x05, 0x05, 0x02, 0x10, 0x00};
    //do every x seconds
  
    sendCmd(diagnostics);
    Serial2.write((byte)count++);
    
    Serial.print("Diagnostic # "); Serial.print(count - 1); Serial.println(" sent.");
}

void writeToDNT(byte byteToWrite)
{
    int i = 0;
    while (digitalRead(dntCTSPin) == HIGH && i < dntMaxReadTries) {
        ++i;
    }
    if (i < dntMaxReadTries) {
        Serial2.write(byteToWrite);
    } else {
        Serial.println(F("Unable to write to DNT: CTS pin is high"));
    }
}

void hardRadioReset()
{
    digitalWrite(dntHwResetPin, LOW);
    delay(200);
    digitalWrite(dntHwResetPin, HIGH);
}

