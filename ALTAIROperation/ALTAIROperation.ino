// Our Arduino code for full operation of the ALTAIR payload
#include <TinyGPS++.h>

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
            
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_ADS1015.h>
#include <SFE_HMC6343.h>
#include <ALTAIR_DNT900.h>
#include <ALTAIR_SHX144.h>
#include <ALTAIR_RFM23BP.h>
#include <ALTAIR_UM7.h>
#include <ALTAIR_IntSphereLightSource.h>
#include <ALTAIR_DiffLEDLightSource.h>

#include <utility/imumaths.h>

#define        SEALEVELPRESSURE_HPA       (1013.25)
#define        MAX_BUFFER_SIZE               32

// Pins 6, 7, and 8 correspond to timer 4 pins.  Pins 45 and 46 correspond to timer 5 pins.  Pins 11 and 12 correspond to timer 1 pins.
//   pin 6 = axle rotation servo;          pin 7 = helium valve servo;              pin 8 = cutdown and parafoil steering servo  
//   setting 6 = default upward-pointing;  setting 15 = fully closed helium valve;  setting 7 = cutdown plunger open, 6 = cutdown plunger closed
// Pins 45, 46, 11, 12 are the propulsion motors.  Setting 0 is off, setting 10 is max for these.
//   pin 9 = ; pin 10 = ; pin 11 = ; pin 12 = 
const byte     pwmPin[7]                = {   6 ,  7 ,  8 , 45  , 46  , 11  , 12   };  

float          setting[7]               = {   6., 15.,  6.,  0. ,  0. ,  0. ,  0.  };
const float    settingSafeMax[7]        = {  15., 16., 15.,  2.5,  2.5,  2.5,  2.5 };
const float    settingSafeMin[7]        = {   0.,  0.,  0.,  0. ,  0. ,  0. ,  0.  };
const float    settingNominal[7]        = {   6., 15.,  6.,  0. ,  0. ,  0. ,  0.  }; 
const byte     axleRotAngSensePin       =    A3;
const byte     cutdownAngSensePin       =    A4;
const byte     helValveAngSensePin      =    A5;

const byte     gpsI2CAddress            =  0x42;               // U-blox NEO-M8N GPS     (in Hobbyking GPS/compass)
const byte     compassmagI2CAddress     =  0x1E;               // HMC5883L magnetometer  (in Hobbyking GPS/compass)
const byte     compassmagInitBytes[]    = {0x70, 0x20, 0x00};  // 0x70 = 8 samples averaged per output, default output rate of 15 Hz, no applied bias
                                                               // 0x20 = default gain of 1090 LSB/Gauss
                                                               // 0x00 = continuous measurement mode 
const byte     I2CMUXADDR               =  0x70;

const byte     SDcard_CS                =    24;

static const uint8_t rfm23SendString[]  = " VE7XJA STATION ALTAIR OVER";

static int16_t magDataX,      magDataY,      magDataZ;
static float   magCalX = 1.0, magCalY = 1.0, magCalZ = 1.0;                                                        
float          compassmagHeading        =  -999.;              // heading in degrees east of true North

// const long     dntMaxReadTries          =  500000;
const long     dntMaxReadTries          =   100;
const int      maxTermLength            =   255;
// const int      maxTermLength            =  5000;

uint8_t        i2cBuf[maxTermLength];

bool           backupRadiosOn           =   true;
byte           infoByte                 =    '7';
byte           term[maxTermLength];
int            termIndex                =     0;
int            termLength               =     0;
int            hasBegun                 =     0;
  
long           previousMillis           =     0;
long           previousMillis2          =     0;
long           previousMillis3          =     0;
long           previousMillis4          =     0;
long           previousMillis5          =     0;
long           previousMillis6          =     0;
int            count                    =     0;

// All of the below DEFAULT_ pin locations are defined in the respective class header .h file in the library for each component.
ALTAIR_DNT900               dnt900(               DEFAULT_DNT_SERIALID       ,  // on Serial1 -- 910 MHz, 1/2-wave antenna on front of gondola
                                                  DEFAULT_DNTHWRESETPIN      ,         
                                                  DEFAULT_DNTCTSPIN          , 
                                                  DEFAULT_DNTRTSPIN          );
ALTAIR_SHX144               shx144(               DEFAULT_SHX_SERIALID       ,  // on Serial2 -- 144 MHz, 1/4-wave antenna on back  of gondola
                                                  DEFAULT_FAKESHXPROGRAMRXPIN,  // (and also on SoftwareSerial programming pins DEFAULT_SHXPROGRAMPIN and DEFAULT_FAKESHXPROGRAMRXPIN)
                                                  DEFAULT_SHXPROGRAMPIN      , 
                                                  DEFAULT_SHXBUSYPIN         );  
ALTAIR_RFM23BP              rfm23bp(              DEFAULT_RFM_CHIPSELECTPIN  ,  // on SPI     -- 433 MHz, 1/2-wave antenna on top   of gondola
                                                  DEFAULT_RFM_INTERRUPTPIN   );    
ALTAIR_UM7                  um7(                  DEFAULT_UM7_SERIALID       ); // on Serial3 -- CH Robotics 9-axis orientation fusion sensor, including DHRobot GPS data pass-through

ALTAIR_IntSphereLightSource intSphereLightSource( DEFAULT_BLUE440NMLASERPIN  ,
                                                  DEFAULT_GREEN532NMLASERPIN ,
                                                  DEFAULT_RED635NMLASERPIN   ,
                                                  DEFAULT_RED670NMLASERPIN   );
ALTAIR_DiffLEDLightSource   diffLEDLightSource(   DEFAULT_BLUELEDSPIN        ,
                                                  DEFAULT_GREENLEDSPIN       ,
                                                  DEFAULT_YELLOWLEDSPIN      ,
                                                  DEFAULT_REDLEDSPIN         );                       

Adafruit_BNO055             bno = Adafruit_BNO055(55);       // on I2C      (default addr = 0x28), Adafruit 9-axis orientation fusion sensor
Adafruit_BME280             bmeMast, bmeBalloon, bmePayload; // also on I2C (default addr = 0x77.  bmeMast [default addr] is on nav mast, bmeBalloon [w/ addr 0x76] is inside balloon, bmePayload [I2CMUXed, but default addr] inside gondola)
Adafruit_ADS1115            ads1115ADC1;                     // also on I2C (default addr = 0x48), contains the 2 photodiode inputs
Adafruit_ADS1115            ads1115ADC2(0x49);               // also on I2C         (addr = 0x49), contains 
Adafruit_ADS1115            ads1115ADC3(0x4A);               // also on I2C         (addr = 0x4A), contains 
SFE_HMC6343                 hmc6343;                         // also on I2C         (addr = 0x19), SparkFun magnetometer(+accelerometer)
TinyGPSPlus                 gps;                             // nav mast GPS on I2C (addr = 0x42), and also UM7 with DFRobot GPS input on Serial3 (initialized later)

// SD card output file
File                        theSDcardFile;


void tcaselect(int8_t i) {
  if (i > 7 || i < -1) return;
 
  Wire.beginTransmission(I2CMUXADDR);
  if (i == -1) {
    Wire.write(0);
  } else {
    Wire.write(1 << i);
  }
  Wire.endTransmission();  
}


void setup() {
  for (int i = 0; i < 7; ++i) 
  pinMode(pwmPin[i],       OUTPUT);

// and perhaps try initializing SPI before DNT and SHX ...  and try SPI_HALF_SPEED initialization ... and try RFM23BP first again ... and try just straight ReadWrite
//  pinMode(SDcard_CS,       OUTPUT);  // seems to sometimes cause SD.begin to have problems if SD.begin done immediately following it
  
// normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green laser on, but that heats up the I-drive transistor too much)
  intSphereLightSource.initialize();
  diffLEDLightSource.initialize();
  
  Serial.begin(9600);

  Serial.println(F("Starting DNT900 radio setup..."));
  if (!dnt900.initialize()) {
    Serial.println(F("DNT900 radio init failed"));
    while(1);
  }
  Serial.println(F("DNT900 radio setup complete."));

  Serial.println(F("Starting SHX1 serial modem radio setup..."));
  if (!shx144.initialize()) {
    Serial.println(F("SHX1 radio init failed"));
    while(1);
  }
  Serial.println(F("SHX1 serial modem radio setup complete."));

  delay(100);

  Serial.println(F("Initializing SPI bus RFM23BP radio (433 MHz, antenna on top of gondola) ..."));
  if (!rfm23bp.initialize()) {
    Serial.println(F("RFM23BP radio init failed"));
    while(1);
  }


// try putting a delay here, and/or the other type of SD card initialization

  Serial.println(F("Initializing SPI bus SD card output ..."));
  if (!SD.begin(SDcard_CS)) {
    Serial.println(F("SD card output initialization failed!"));
    while(1);
  }
  theSDcardFile = SD.open("data.txt", FILE_WRITE);
  if (theSDcardFile) {
    Serial.println(F("SD card initialization complete."));
  } else {
    Serial.println(F("SD card file open failed."));
    while(1);
  }



  Serial.println(F("SPI bus and device initialization complete."));

  Serial.println(F("Starting I2C/TWI bus..."));
  Wire.begin();
  Serial.println(F("Initializing and calibrating I2C/TWI compass magnetometer..."));
/*
  const byte     compassmagCalibBytes[]    = { 0x71, 0x60, 0x01 };
  twiWriteBytes(compassmagI2CAddress, 0x00, compassmagCalibBytes, 1);
  delay(50);
  twiWriteBytes(compassmagI2CAddress, 0x01, &compassmagCalibBytes[1], 2);
  delay(100);
  getCompassmagData();
  const float    magScale[]                = { 1160.0, 1160.0, 1080 };
  magCalX = magScale[0]/abs(magDataX);
  magCalY = magScale[1]/abs(magDataY);
  magCalZ = magScale[2]/abs(magDataZ);
  twiWriteBytes(compassmagI2CAddress, 0x00, compassmagInitBytes, 3);
*/
  Wire.beginTransmission(compassmagI2CAddress);
  Wire.write(0x02);
  Wire.endTransmission();
//  I2c.write(compassmagI2CAddress, 0x02, 0x00);
//  I2c.end();


/*  jj */
  Serial.println(F("BME280 pressure/temp/humidity sensors initialization..."));

  bool status1, status2, status3, statusall;    
  status1 = bmeMast.begin();
  status2 = bmeBalloon.begin(0x76);
  tcaselect(0);
  status3 = bmePayload.begin();
  tcaselect(-1);
  statusall = status1 && status3; // && status2;
  if (!statusall) {
      Serial.print(F("Could not find one of the 3 BME280 sensors, check wiring!  BME280 on nav mast: ")); Serial.print(status1); 
      Serial.print(F("   BME280 in the balloon valve: ")); Serial.print(status2); 
      Serial.print(F("   BME280 inside the gondola: ")); Serial.println(status3); 
      while (1);
  }

  Serial.println(F("BNO055 orientation sensor initialization..."));

  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.println(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println(F("HMC6343 magnetometer initialization..."));

  if (!hmc6343.init())
  {
    Serial.println("SparkFun HMC6343 magnetometer initialization failed\n\r"); // Report failure, is the sensor wiring correct?
    while(1);
  } 

  Serial.println(F("Initializing the three ADS1115 4-channel ADC breakout boards ..."));
  ads1115ADC1.begin();
  ads1115ADC2.begin();
  ads1115ADC3.begin();
  
  Serial.println(F("I2C/TWI bus and device initialization complete.  Now initializing PWM outputs ..."));


  // start up the PWM outputs
//  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM32) | _BV(WGM31);
//  TCCR3B = _BV(CS32);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM42) | _BV(WGM41);
  TCCR4B = _BV(CS42);
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) |               _BV(WGM52) | _BV(WGM51);
  TCCR5B = _BV(CS52);                                    
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) |               _BV(WGM12) | _BV(WGM11);
  TCCR1B = _BV(CS12);
  

//  OCR3A = 32 + 2*setting[0];
//  OCR3B = 32 + 2*setting[1];
//  OCR3C = 32 + 2*setting[2];
  OCR4A = 34 + 2*setting[0];
  OCR4B = 34 + 2*setting[1];
  OCR4C = 34 + 2*setting[2];
  OCR5B = 34 + 2*setting[3];
  OCR5A = 34 + 2*setting[4];
  OCR1A = 34 + 2*setting[5];
  OCR1B = 34 + 2*setting[6];

  Serial.println(F("Setup complete."));
 
}

void loop() {

  if (getGPSandHeadingAtInterval(400)) {
//  if (getGPSandHeadingAtInterval(377)) {  // I thought setting interval to 377 might help with the DNT blocking GPS issue -- but it really was reducing dntMaxReadTries that fixed it.
//    digitalWrite(gpsSuccessfullyLockedPin, LOW);
//    Serial.println(F("gotten GPS"));
//    delay(100);
//    digitalWrite(gpsSuccessfullyLockedPin, HIGH);
  }
  else {
//    Serial.println(F("failed to get GPS"));
  }

  getPropMonInfoAtInterval(450);

  sendStatusToDNTRadioAndReadCommandsAtInterval(1000);

//  delay(100);
  if (backupRadiosOn) sendStationNameToSHXandRFMRadiosAtInterval(1333);
//  delay(100);

  sendGPSCompassStatusToComputerAtInterval(5000);

  printNavMastSensorValsAndAdjSettingsAtInterval(2000);

  storeDataOnMicroSDCard();
  

//  delay(100);

}

void storeDataOnMicroSDCard() {

}

void getPropMonInfoAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis6 > interval) { 
    previousMillis6 = currentMillis;

    char packedRPM[4];
    char packedCurrent[4];
    char packedTemp[8];

    Wire.requestFrom(8, 16);
    for (int i = 0; i < 4; ++i) packedRPM[i]     = Wire.read();
    for (int i = 0; i < 4; ++i) packedCurrent[i] = Wire.read();
    for (int i = 0; i < 8; ++i) packedTemp[i]    = Wire.read();
    


  }
  
}

void printNavMastSensorValsAndAdjSettingsAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis5 > interval) { 
    previousMillis5 = currentMillis;

// First, the BME280 temp/pres/hum

    Serial.print(F("Mast Temperature = "));
    Serial.print(bmeMast.readTemperature());
    Serial.println(F(" *C"));

    Serial.print(F("Mast Pressure = "));

    Serial.print(bmeMast.readPressure() / 100.0F);
    Serial.println(F(" hPa"));

    Serial.print(F("Mast Approx. Altitude = "));
    Serial.print(bmeMast.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));

    Serial.print(F("Mast Humidity = "));
    Serial.print(bmeMast.readHumidity());
    Serial.println(F(" %"));
/*
    Serial.print(F("Balloon Temperature = "));
    Serial.print(bmeBalloon.readTemperature());
    Serial.println(F(" *C"));

    Serial.print(F("Balloon Pressure = "));

    Serial.print(bmeBalloon.readPressure() / 100.0F);
    Serial.println(F(" hPa"));

    Serial.print(F("Balloon Approx. Altitude = "));
    Serial.print(bmeBalloon.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(F(" m"));

    Serial.print(F("Balloon Humidity = "));
    Serial.print(bmeBalloon.readHumidity());
    Serial.println(F(" %"));
*/
    Serial.println();

    

// Next, the BNO055 orientation

    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
  
    /* Display the floating point data */
    Serial.print(F("X: "));
    Serial.print(event.orientation.x, 4);
    Serial.print(F("\tY: "));
    Serial.print(event.orientation.y, 4);
    Serial.print(F("\tZ: "));
    Serial.print(event.orientation.z, 4);
    Serial.println(F(""));

// And lastly, the Sparkfun HMC6343 magnetometer

    hmc6343.readHeading();
    printHMC6343HeadingData();
    hmc6343.readAccel();
    printHMC6343AccelData();

// Now, see if there is serial input for settings adjustment

    byte    inputByte;
    boolean thingsHaveChanged = false;
    int     channelToModify   = 0;
    if (Serial.available()) {
        inputByte = Serial.read();
        thingsHaveChanged = true;
        switch(inputByte) {
          case 'A' :
            channelToModify = 0;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'B' :
            channelToModify = 1;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;       
          case 'C' :
            channelToModify = 2;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;       
          case 'D' :
            channelToModify = 3;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'E' :
            channelToModify = 4;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;              
          case 'F' :
            channelToModify = 5;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'G' :
            channelToModify = 6;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'H' :
            channelToModify = 3;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 4;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 5;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 6;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            break;
          case 'N' :
          case 'n' :
            backupRadiosOn = true;
            break;
          case 'O' :
          case 'o' :
            backupRadiosOn = false;
            break;
          case 'U' :
            channelToModify = 3;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 4;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 5;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 6;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;             
          case 'a' :
            channelToModify = 0;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'b' :
            channelToModify = 1;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;       
          case 'c' :
            channelToModify = 2;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;       
          case 'd' :
            channelToModify = 3;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'e' :
            channelToModify = 4;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;              
          case 'f' :
            channelToModify = 5;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break; 
          case 'g' :
            channelToModify = 6;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'h' :
            channelToModify = 3;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 4;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 5;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 6;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            break; 
          case 'u' :
            channelToModify = 3;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 4;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 5;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 6;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'X' :                
          case 'x' :
            for (int i = 0; i < 7; ++i)    setting[i] = settingNominal[i];
            break;
          default :
            thingsHaveChanged = false;
            break;
        }
    } 

        Serial.print("Present settings: "); Serial.print(setting[0]); Serial.print(" "); Serial.print(setting[1]);
        Serial.print(" "); Serial.print(setting[2]); Serial.print(" "); Serial.print(setting[3]);
        Serial.print(" "); Serial.print(setting[4]); Serial.print(" "); Serial.print(setting[5]);
        Serial.print(" "); Serial.println(setting[6]);

// send the info via the DNT
   if (thingsHaveChanged) {
      OCR4A = 34 + 2*setting[0];
      OCR4B = 34 + 2*setting[1];
      OCR4C = 34 + 2*setting[2];
      OCR5B = 34 + 2*setting[3];
      OCR5A = 34 + 2*setting[4];
      OCR1A = 34 + 2*setting[5];
      OCR1B = 34 + 2*setting[6];
   }

  
  }

}

void printHMC6343HeadingData() {
  Serial.print(F("Heading: "));
  Serial.print(hmc6343.heading); Serial.print(F("  ")); // Print raw heading value
  Serial.print((float) hmc6343.heading/10.0);Serial.print(F(" degrees"));Serial.println(); // Print heading in degrees
}

void printHMC6343AccelData() {
  Serial.print(F("HMC6343 X: "));
  Serial.print(hmc6343.accelX); Serial.print(F("  ")); // Print raw acceleration measurement on x axis
  Serial.print((float) hmc6343.accelX/1024.0);Serial.println(F("g")); // Print x axis acceleration measurement in g forces
  Serial.print(F("HMC6343 Y: "));
  Serial.print(hmc6343.accelY); Serial.print(F("  ")); // Print raw acceleration measurement on y axis
  Serial.print((float) hmc6343.accelY/1024.0);Serial.println(F("g")); // Print y axis acceleration measurement in g forces
  Serial.print(F("HMC6343 Z: "));
  Serial.print(hmc6343.accelZ); Serial.print(F("  ")); // Print raw acceleration measurement on z axis
  Serial.print((float) hmc6343.accelZ/1024.0);Serial.println(F("g")); // Print z axis acceleration measurement in g forces
}

void getCompassmagData() {
  uint8_t      magDataRegister       =  0x03;
/*
  byte         compassmagRealData[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const byte   compassmagFakeData[]  = {0x00};
  twiWriteBytes(compassmagI2CAddress, magDataRegister, compassmagFakeData, 0);     // return the magnetometer pointer address to the magDataRegister: 0x03
  delay(10);
  twiReadBytes(compassmagI2CAddress, magDataRegister, compassmagRealData, 6);
  magDataX = compassmagRealData[0] << 8 | compassmagRealData[1];                   // x component
  magDataY = compassmagRealData[4] << 8 | compassmagRealData[5];                   // y component (HMC5883L z data register is before the y register, don't ask y!...)
  magDataZ = compassmagRealData[2] << 8 | compassmagRealData[3];                   // z component
*/
  Wire.beginTransmission(compassmagI2CAddress);
  Wire.write(magDataRegister);
  Wire.endTransmission();
  Wire.requestFrom(compassmagI2CAddress, (uint8_t) 6);
  magDataX  = Wire.read() << 8;
  magDataX |= Wire.read();
  magDataZ  = Wire.read() << 8;      // HMC5883L z data register is before the y register, don't ask y!...
  magDataZ |= Wire.read();
  magDataY  = Wire.read() << 8;
  magDataY |= Wire.read();

/*
  I2c.begin();
  I2c.read(compassmagI2CAddress, magDataRegister, 6);
  magDataX = I2c.receive() << 8;
  magDataX |= I2c.receive();
  magDataZ = I2c.receive() << 8;      // HMC5883L z data register is before the y register, don't ask y!...
  magDataZ |= I2c.receive();
  magDataY = I2c.receive() << 8;
  magDataY |= I2c.receive();
  I2c.end();
*/

//  Serial.print("compassmagRealData[0] = ");   Serial.print(compassmagRealData[0], HEX); Serial.print("  compassmagRealData[1] = "); Serial.print(compassmagRealData[1], HEX);
//  Serial.print("  compassmagRealData[2] = "); Serial.print(compassmagRealData[2], HEX); Serial.print("  compassmagRealData[3] = "); Serial.print(compassmagRealData[3], HEX);
//  Serial.print("  compassmagRealData[4] = "); Serial.print(compassmagRealData[4], HEX); Serial.print("  compassmagRealData[5] = "); Serial.println(compassmagRealData[5], HEX);
//  Serial.print("magDataX = "); Serial.print(magDataX); Serial.print("  magDataY = "); Serial.print(magDataY); Serial.print("  magDataZ = "); Serial.println(magDataZ);   
}

bool getGPSandHeadingAtInterval(long interval)
{
  bool retval = false;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis4 > interval) { 
    previousMillis4 = currentMillis;
    Serial.println("Getting heading and GPS");

     // First, get the magnetometer heading
    getCompassmagData();
    compassmagHeading = atan2(magDataY*magCalY, magDataX*magCalX) * 180. / M_PI;

// Then, get the GPS
    Wire.beginTransmission(gpsI2CAddress);
    Wire.write(0xFD);
    Wire.endTransmission();
    uint8_t i2cErr = Wire.requestFrom(gpsI2CAddress, (uint8_t) 2);
    if (i2cErr == 0) return false; // got some TWI error. Return

 /*
    I2c.begin();
    uint8_t i2cErr = I2c.read(gpsI2CAddress, 0xFD, 2);
    if (i2cErr) return false; // got some TWI error. Return
*/    
    uint16_t totalBytes = Wire.read() << 8;
    totalBytes |= Wire.read();
    if (!totalBytes) return false; // GPS not ready to send data. Return
    Serial.print(F("GPS is ready to transfer ")); Serial.print(totalBytes, DEC); Serial.println(F(" bytes"));

    while (totalBytes) {
      uint8_t bytes2Read = totalBytes;
      if (totalBytes > MAX_BUFFER_SIZE) bytes2Read = MAX_BUFFER_SIZE;
//      Serial.print(bytes2Read);Serial.print("here5.5");
      Wire.beginTransmission(gpsI2CAddress);
      Wire.write(0xFF);
      Wire.endTransmission();
      i2cErr = Wire.requestFrom(gpsI2CAddress, bytes2Read);
//      i2cErr = I2c.read(gpsI2CAddress, 0xFF, bytes2Read);
      if (i2cErr == 0) return false; // got some TWI error. Return
//      if (i2cErr) return false; // got some TWI error. Return
//      Serial.print("here6 ");
      for (uint8_t i = 0; i < bytes2Read; i++) {
        uint8_t theByte = Wire.read();
        if (theByte == 0xFF) return false; // got some TWI error. Return
//        retval = gps.encode(theByte);
//        Serial.write(char(theByte));
        bool isEncoded = gps.encode(theByte);
        retval |= isEncoded;
      }
      totalBytes -= bytes2Read;
//      delay(50);                    // this delay appears to fix mysterious occasional freezes in this while loop
    }
//    I2c.end();


/*  
    uint16_t bytes = twiReadBytes(gpsI2CAddress, 0xFD, (uint8_t *) i2cBuf, 2);
    if (!bytes) return false; // got some TWI error. Return

    uint16_t totalBytes = ((uint16_t) i2cBuf[0] << 8) | i2cBuf[1];
    if (!totalBytes) return false; // GPS not ready to send data. Return

//  Serial.print("here5");
//  Serial.print("GPS is ready to transfer "); Serial.print(totalBytes, DEC); Serial.println(" bytes");
    while (totalBytes) {
      uint16_t bytes2Read = totalBytes;
      if (totalBytes > 128) bytes2Read = 128;
//      Serial.print(bytes2Read);Serial.print("here5.5");
      bytes = twiReadBytes(gpsI2CAddress, 0xFF, i2cBuf, bytes2Read);
//      Serial.print("here6 "); Serial.print(bytes);
      for (uint8_t i = 0; i < bytes; i++) {
//        retval = gps.encode(i2cBuf[i]);
//        Serial.write(i2cBuf[i]);
        bool isEncoded = gps.encode(i2cBuf[i]);
        retval |= isEncoded;
      }
//    for (uint8_t i = 0; i < bytes; i++) Serial.write(char(i2cBuf[i]));
//      Serial.print("here7 ");
      totalBytes -= bytes2Read;
      delay(50);                    // this delay appears to fix mysterious occasional freezes in this while loop
    }
*/
  }
  return retval;
}

void sendStationNameToSHXandRFMRadiosAtInterval(long interval)
{
  unsigned long currentMillis = millis();
//  if (digitalRead(shxBusyPin) == LOW && currentMillis - previousMillis3 > interval) {
  if (currentMillis - previousMillis3 > interval) { 
    Serial.println(F("Writing station name to SHX1"));
    previousMillis3 = currentMillis;
    intSphereLightSource.setLightsBackupRadio();
    diffLEDLightSource.setLightsBackupRadio();
      
  //send my call sign (VE7XJA)
    shx144.send(byte('V'));
    shx144.send(byte('E'));
    shx144.send(byte('7'));
    shx144.send(byte('X'));
    shx144.send(byte('J'));
    shx144.send(byte('A'));

  //send station name
    shx144.send(byte(' '));
    shx144.send(byte('S'));
    shx144.send(byte('T'));
    shx144.send(byte('A'));
    shx144.send(byte('T'));
    shx144.send(byte('I'));
    shx144.send(byte('O'));
    shx144.send(byte('N'));
    shx144.send(byte(':'));
    shx144.send(byte(' '));  
    shx144.send(byte('A'));
    shx144.send(byte('L'));
    shx144.send(byte('T'));
    shx144.send(byte('A'));
    shx144.send(byte('I'));
    shx144.send(byte('R'));
    shx144.send(byte(' '));
    shx144.send(byte('O'));
    shx144.send(byte('V'));
    shx144.send(byte('E'));
    shx144.send(byte('R'));
    shx144.send(byte(' '));
    shx144.send(byte(' '));

    delay(20);

//    unsigned char* rfm23SendString = new unsigned char[sizeof(" VE7XJA STATION ALTAIR OVER")]; 
//    strcpy((char*) rfm23SendString, " VE7XJA STATION ALTAIR OVER");
//    Serial.print("About to send to RFM23BP: "); Serial.println((char *) rfm23SendString);
    rfm23bp.send(rfm23SendString);
//    rfm23bp.send((const unsigned char*) " VE7XJA STATION ALTAIR OVER");

/*
   //send my call sign (VE7XJA)
    rfm23bp.send(byte('V'));
    rfm23bp.send(byte('E'));
    rfm23bp.send(byte('7'));
    rfm23bp.send(byte('X'));
    rfm23bp.send(byte('J'));
    rfm23bp.send(byte('A'));

  //send station name
    rfm23bp.send(byte(' '));
    rfm23bp.send(byte('S'));
    rfm23bp.send(byte('T'));
    rfm23bp.send(byte('A'));
    rfm23bp.send(byte('T'));
    rfm23bp.send(byte('I'));
    rfm23bp.send(byte('O'));
    rfm23bp.send(byte('N'));
//    rfm23bp.send(byte(':'));
    rfm23bp.send(byte(' '));  
    rfm23bp.send(byte('A'));
    rfm23bp.send(byte('L'));
    rfm23bp.send(byte('T'));
    rfm23bp.send(byte('A'));
    rfm23bp.send(byte('I'));
    rfm23bp.send(byte('R'));
    rfm23bp.send(byte(' '));
    rfm23bp.send(byte('O'));
    rfm23bp.send(byte('V'));
    rfm23bp.send(byte('E'));
    rfm23bp.send(byte('R'));
    rfm23bp.send(byte(' '));
    rfm23bp.send(byte(' '));   
*/

    delay(40);
    intSphereLightSource.setLightsNormal();
    diffLEDLightSource.setLightsNormal();

  } else if (shx144.available()) {
    byte shxTerm[maxTermLength];
    int  shxTermIndex = 0;
    while (shx144.available() && shxTermIndex < maxTermLength) {
      shxTerm[shxTermIndex++] = shx144.read();
      delay(5);
    }
    Serial.print(F("  Number of SHX bytes: "));    Serial.println(shxTermIndex);
    Serial.print(F("  SHX Data: \""));
    for (int i = 0; i < shxTermIndex; i++)
    {
      Serial.print((char)shxTerm[i]);
    }
    Serial.println(F("\""));
    Serial.print(F("  SHX Data (HEX): \""));
    for (int i = 0; i < shxTermIndex; i++)
    {
      Serial.print(shxTerm[i], HEX); Serial.print(F(" "));
    }
    Serial.println(F("\""));  
  } else if (shx144.isBusy()) {
//    Serial.println(F("Cannot send the station name to SHX1, since the SHX1 Busy pin is HIGH, and the SHX1 is not available"));
  }
  
}


void sendStatusToDNTRadioAndReadCommandsAtInterval(long interval)
{
  unsigned long currentMillis = millis();
  if (!dnt900.isBusy() && currentMillis - previousMillis2 > interval) {
    int32_t latitude  = gps.location.lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t longitude = gps.location.lng() * 1000000;  // Longitude, in millionths of a degree.
    int16_t elevation = gps.altitude.meters();         // Elevation above mean sea level in meters.  NOTE: as this is signed 16 bit, this will *turn over*
                                                       // when above 32.77 km!  Would need to add & transmit an extra byte, if one preferred a higher limit.
                                                       // (Or make it unsigned, which would cause issues for launches from Death Valley or the Dead Sea. :)
                                                       // (One could get clever, and make it unsigned, but add, then later remove, 1000 meters, but I'm not 
                                                       //  that clever.)
    
    Serial.println(F("*** Writing status to DNT ***"));
    previousMillis2 = currentMillis;
    intSphereLightSource.setLightsPrimaryRadio();
    diffLEDLightSource.setLightsPrimaryRadio();

    dnt900.send(0xFA);
    dnt900.send(0x0B);

  //send latitude
    dnt900.send(byte(( latitude >> 24) & 0xFF));
    dnt900.send(byte(( latitude >> 16) & 0xFF));
    dnt900.send(byte(( latitude >>  8) & 0xFF));
    dnt900.send(byte(  latitude        & 0xFF));
  //send longitude
    dnt900.send(byte((longitude >> 24) & 0xFF));
    dnt900.send(byte((longitude >> 16) & 0xFF));
    dnt900.send(byte((longitude >>  8) & 0xFF));
    dnt900.send(byte( longitude        & 0xFF));
  //send elevation
    dnt900.send(byte((elevation >>  8) & 0xFF));
    dnt900.send(byte( elevation        & 0xFF));

    dnt900.send(infoByte);

    delay(40);
    intSphereLightSource.setLightsNormal();
    diffLEDLightSource.setLightsNormal();

  long dntReadTry = 0;
  termIndex = termLength = 0;
  
  Serial.println(F("Reading radio"));
  if (infoByte == '7') infoByte = 'r';
  
    while (true) {
      while (!dnt900.available() && dntReadTry < dntMaxReadTries) {
        ++dntReadTry;
//      delay(5);
      }
      if (dntReadTry < dntMaxReadTries) {

        do {
          Serial.println(F("Reading a byte from radio"));
          if (infoByte == 'r') infoByte = 's';

          byte b = dnt900.read();
//        term[termIndex++] = b;
    
          if (b == (byte)0xFC && hasBegun == 0) {
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
        Serial.println(F("DNT is not available for reading"));
        break;
      }
    }

    if (termLength == termIndex && termLength > 0) {
      Serial.println("RxDataTransparent");
      RxDataTransparent(term, termLength);

      termLength = termIndex = hasBegun = 0;    
    }

  } else if (dnt900.isBusy()) {
    Serial.println(F("Cannot send the GPS to DNT, since the DNT is busy"));
  }

  
  //send altitude
//  signed long gps_altitude = gps.altitude();
      
  //send speed
//  unsigned long gps_speed = gps.speed();
    
  //send HDOP
//  unsigned long gps_hdop = gps.hdop();



}

void RxDataTransparent (byte term[], int termLength) {

    byte    inputByte         = term[1];
    boolean thingsHaveChanged = true;
    int     channelToModify   = 0;

    infoByte = 'S';

      if (term[0] == 's') {
        switch(inputByte) {
          case 'A' :
            channelToModify = 0;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'B' :
            channelToModify = 1;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;       
          case 'C' :
            channelToModify = 2;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;       
          case 'D' :
            channelToModify = 3;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'E' :
            channelToModify = 4;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;              
          case 'F' :
            channelToModify = 5;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'G' :
            channelToModify = 6;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'H' :
            channelToModify = 3;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 4;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 5;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            channelToModify = 6;
            setting[channelToModify] += 0.5;
            if (setting[channelToModify] > settingSafeMax[channelToModify])   setting[channelToModify] -= 0.5;
            break;
          case 'N' :
          case 'n' :
            backupRadiosOn = true;
            break;
          case 'O' :
          case 'o' :
            backupRadiosOn = false;
            break;
          case 'U' :
            channelToModify = 3;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 4;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 5;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            channelToModify = 6;
            ++setting[channelToModify];
            if (setting[channelToModify] > settingSafeMax[channelToModify]) --setting[channelToModify];
            break;
          case 'a' :
            channelToModify = 0;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'b' :
            channelToModify = 1;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;       
          case 'c' :
            channelToModify = 2;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;       
          case 'd' :
            channelToModify = 3;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'e' :
            channelToModify = 4;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;              
          case 'f' :
            channelToModify = 5;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'g' :
            channelToModify = 6;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'h' :
            channelToModify = 3;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 4;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 5;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            channelToModify = 6;
            setting[channelToModify] -= 0.5;
            if (setting[channelToModify] < settingSafeMin[channelToModify])   setting[channelToModify] += 0.5;
            break; 
          case 'u' :
            channelToModify = 3;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 4;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 5;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            channelToModify = 6;
            --setting[channelToModify];
            if (setting[channelToModify] < settingSafeMin[channelToModify]) ++setting[channelToModify];
            break;
          case 'X' :
          case 'x' :
            for (int i = 0; i < 7; ++i)    setting[i] = settingNominal[i];
            break;
          default :
            thingsHaveChanged = false;
            break;
        }
      }

    if (thingsHaveChanged) {
      ++infoByte;
      OCR4A = 34 + 2*setting[0];
      OCR4B = 34 + 2*setting[1];
      OCR4C = 34 + 2*setting[2];
      OCR5B = 34 + 2*setting[3];
      OCR5A = 34 + 2*setting[4];
      OCR1A = 34 + 2*setting[5];
      OCR1B = 34 + 2*setting[6];
    }
  
}


void sendGPSCompassStatusToComputerAtInterval(long interval) {

  unsigned long age, date, chars;
  unsigned short sentences, failed;
  byte month, day, hour, minute, second, hundredths;
  int year;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   

    Serial.print(F("ALTAIR Latitude: "));    Serial.println(gps.location.lat(), 6);
    Serial.print(F("ALTAIR Longitude: "));   Serial.println(gps.location.lng(), 6);
    Serial.print(F("ALTAIR LatLong Age: ")); Serial.println(gps.location.age());
    Serial.print(F("ALTAIR Year: "));        Serial.println(gps.date.year());
    Serial.print(F("ALTAIR Month: "));       Serial.println(gps.date.month());
    Serial.print(F("ALTAIR Day: "));         Serial.println(gps.date.day());
    Serial.print(F("ALTAIR Hour: "));        Serial.println(gps.time.hour());
    Serial.print(F("ALTAIR Minute: "));      Serial.println(gps.time.minute());
    Serial.print(F("ALTAIR Second: "));      Serial.println(gps.time.second());
    Serial.print(F("ALTAIR Hundredth: "));   Serial.println(gps.time.centisecond());
//    Serial.print(F("ALTAIR GPSTime Age: ")); Serial.println(gps.time.age());                  // no need to have this, it always reads the same thing
    Serial.print(F("ALTAIR compass magnetometer heading: ")); Serial.println(compassmagHeading);

  }


}


