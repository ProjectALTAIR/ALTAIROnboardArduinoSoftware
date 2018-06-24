// Our Arduino code for operation of the Capella DNT900P-based ground station

#include <ALTAIR_DNT900.h>

float          sentSetting[7]           = {    6., 15.,  6., 0. , 0. , 0. , 0.  };
const float    sentSettingSafeMax[7]    = {   15., 16., 15., 2.5, 2.5, 2.5, 2.5 };
const float    sentSettingSafeMin[7]    = {    0.,  0.,  0., 0. , 0. , 0. , 0.  };
const float    sentSettingNominal[7]    = {    6., 15.,  6., 0. , 0. , 0. , 0.  };

const byte     dntHwResetPin            =      4;
const byte     dntCTSPin                =      5;
const byte     dntRTSPin                =      6;

const long     dntMaxReadTries          = 500000;
const int      maxTermLength            =    255;

bool           backupRadiosOn           =   true;
byte           term[maxTermLength];
int            termIndex                =      0;
int            termLength               =      0;
int            hasBegun                 =      0;

byte           recDataPayload[maxTermLength];
int            rdpIndex                 =      0;
int            rdpLength                =      0;
  
long           previousMillis           =      0;
int            count                    =      0;

ALTAIR_DNT900  dnt900(1, dntHwResetPin,           // on Serial1 -- 910 MHz, Capella antenna: 6-element 63 cm Yagi, approx. 9 dBi
                      dntCTSPin, dntRTSPin);

void setup() {

  Serial.begin(9600);

  Serial.println(F("Starting DNT900 radio setup..."));
  if (!dnt900.initialize()) {
    Serial.println(F("DNT900 radio init failed"));
    while(1);
  }
  Serial.println(F("DNT900 radio setup complete."));

  delay(100);

}

void loop() {
  delay(100);
  readRadio();
  sendCutdownUpdateAtInterval(2000);
}

void sendCutdownUpdateAtInterval(long interval)
{
  unsigned long currentMillis = millis();
//  if (digitalRead(shxBusyPin) == LOW && currentMillis - previousMillis3 > interval) {
  if (currentMillis - previousMillis > interval) { 
    previousMillis = currentMillis;

    byte    inputByte;
    boolean thingsHaveChanged = false;
    int     channelToModify   = 0;
    if (Serial.available()) {
//    Serial.flush();
//    inputByte = Serial.read();
//    if (inputByte == 'm') {
//      inputByte = Serial.read();
//      if (inputByte == 's') {
        inputByte = Serial.read();
        thingsHaveChanged = false;
        switch(inputByte) {
          case 'A' :
            channelToModify = 0;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'B' :
            channelToModify = 1;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;       
          case 'C' :
            channelToModify = 2;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;       
          case 'D' :
            channelToModify = 3;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'E' :
            channelToModify = 4;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;              
          case 'F' :
            channelToModify = 5;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'G' :
            channelToModify = 6;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'H' :
            channelToModify = 3;
            sentSetting[channelToModify] += 0.5;
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) {   sentSetting[channelToModify] -= 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 4;
            sentSetting[channelToModify] += 0.5;
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) {   sentSetting[channelToModify] -= 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 5;
            sentSetting[channelToModify] += 0.5;
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) {   sentSetting[channelToModify] -= 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 6;
            sentSetting[channelToModify] += 0.5;
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) {   sentSetting[channelToModify] -= 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'N' :
          case 'n' :
                                                                                        backupRadiosOn    = true;
                                                                                        thingsHaveChanged = true;
            break;
          case 'O' :
          case 'o' :
                                                                                        backupRadiosOn    = false;
                                                                                        thingsHaveChanged = true;
            break;       
          case 'U' :
            channelToModify = 3;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 4;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 5;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 6;
            ++sentSetting[channelToModify];
            if (sentSetting[channelToModify] > sentSettingSafeMax[channelToModify]) { --sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;          
          case 'a' :
            channelToModify = 0;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'b' :
            channelToModify = 1;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;       
          case 'c' :
            channelToModify = 2;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;       
          case 'd' :
            channelToModify = 3;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'e' :
            channelToModify = 4;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;              
          case 'f' :
            channelToModify = 5;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break; 
          case 'g' :
            channelToModify = 6;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;
          case 'h' :
            channelToModify = 3;
            sentSetting[channelToModify] -= 0.5;
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) {   sentSetting[channelToModify] += 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 4;
            sentSetting[channelToModify] -= 0.5;
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) {   sentSetting[channelToModify] += 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 5;
            sentSetting[channelToModify] -= 0.5;
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) {   sentSetting[channelToModify] += 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 6;
            sentSetting[channelToModify] -= 0.5;
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) {   sentSetting[channelToModify] += 0.5; }
            else                                                                    {   thingsHaveChanged = true; }
            break; 
          case 'u' :
            channelToModify = 3;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 4;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 5;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            channelToModify = 6;
            --sentSetting[channelToModify];
            if (sentSetting[channelToModify] < sentSettingSafeMin[channelToModify]) { ++sentSetting[channelToModify]; }
            else                                                                    {   thingsHaveChanged = true; }
            break;          
          case 'x' :
            for (int i = 0; i < 7; ++i)        sentSetting[i] = sentSettingNominal[i];
            thingsHaveChanged = true;
            break;
          default :
            break;
        }
//      }
//    }
    } 

        Serial.print("Sent settings: "); Serial.print(sentSetting[0]); Serial.print(" "); Serial.print(sentSetting[1]);
        Serial.print(" "); Serial.print(sentSetting[2]); Serial.print(" "); Serial.print(sentSetting[3]);
        Serial.print(" "); Serial.print(sentSetting[4]); Serial.print(" "); Serial.print(sentSetting[5]);
        Serial.print(" "); Serial.println(sentSetting[6]);

// send the info via the DNT
   if (thingsHaveChanged) {
     dnt900.send(0xFA);
     dnt900.send(0x01);

     dnt900.send(inputByte);

   }

  }
}



void readRadio () {
  long dntReadTry = 0;
  termIndex = termLength = 0;
  
  Serial.println(F("Reading radio"));
  
  while (true) {
    while (!dnt900.available() && dntReadTry < dntMaxReadTries) {
      ++dntReadTry;
//      delay(5);
    }
    if (dntReadTry < dntMaxReadTries) {
      do {
        Serial.println(F("Reading a byte from radio"));

        byte b = dnt900.read();
//        term[termIndex++] = b;
    
        if (b == (byte)0xFA && hasBegun == 0) {
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

//        if (termIndex == maxTermLength) break;
      } while (dnt900.available());
//      if (termIndex == maxTermLength) break;
      if (termLength == termIndex && termLength > 0) break;
    
    } else {
//      dnt900.flush();
      Serial.println(F("DNT is not available for reading"));
      break;
    }
  }

  if (termLength == termIndex && termLength > 0) {
      Serial.println("RxDataTransparent");
      RxDataTransparent(term, termLength);

      termLength = termIndex = hasBegun = 0;    
  }

}


void RxDataTransparent (byte term[], int termLength) {
  Serial.print("  Number of bytes: ");    Serial.println(termLength);

  Serial.print("  Data: \"");
  for (int i = 0; i < termLength; i++)
  {
    Serial.print((char)term[i]);
  }
  Serial.println("\"");
  
  Serial.print("  Data (HEX): \"");
  for (int i = 0; i < termLength; i++)
  {
    Serial.print(term[i], HEX); Serial.print(" ");
  }
  Serial.println("\"");  

  if (termLength > 10) AltReconstData(0);
}

void RxData (byte term[], int termLength) {
  Serial.print("  Address: "); Serial.print(term[3], HEX); Serial.print(term[2], HEX); Serial.println(term[1], HEX);
  Serial.print("  RSSI: ");    Serial.println((char)term[4], DEC);
  Serial.print("  Number of bytes: ");    Serial.println(termLength);

  Serial.print("  Data: \"");
  for (int i = 5; i < termLength; i++)
  {
    Serial.print((char)term[i]);
  }
  Serial.println("\"");
  
  Serial.print("  Data (HEX): \"");
  for (int i = 5; i < termLength; i++)
  {
    Serial.print(term[i], HEX); Serial.print(" ");
  }
  Serial.println("\"");

  if (termLength > 15) AltReconstData(5);
}


void AltReconstData(int i) {

    Serial.print(F("Transmitter station GMT time: "));  
      Serial.print(term[i], DEC); Serial.print(":"); 
      if (term[i+1] < 10) Serial.print("0"); Serial.print(term[i+1], DEC); Serial.print(":"); 
      if (term[i+2] < 10) Serial.print("0"); Serial.println(term[i+2], DEC);

    long lat = 0;
    long lon = 0;
    
    lat  = ((unsigned long) term[i+3])  << 24;
    lat |= ((unsigned long) term[i+4])  << 16;
    lat |= ((unsigned long) term[i+5])  << 8;
    lat |= ((unsigned long) term[i+6]);
    
    lon  = ((unsigned long) term[i+7]) << 24;
    lon |= ((unsigned long) term[i+8]) << 16;
    lon |= ((unsigned long) term[i+9]) << 8;
    lon |= ((unsigned long) term[i+10]);
    
    Serial.print("Latitude:  "); Serial.println(lat);
    Serial.print("Longitude: "); Serial.println(lon);

}


void sendStatusToComputerAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   

  }

}

