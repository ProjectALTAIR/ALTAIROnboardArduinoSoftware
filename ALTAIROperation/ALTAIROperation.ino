// Our Arduino code for full operation of the ALTAIR payload
#include <TinyGPS++.h>

#include <ALTAIR_GlobalMotorControl.h>
#include <ALTAIR_GlobalDeviceControl.h>
#include <ALTAIR_GlobalLightControl.h>

float          compassmagHeading        =  -999.;           // heading in degrees East of true North

bool           backupRadiosOn           =  true ;
  
long           previousMillis           =     0 ;
long           previousMillis2          =     0 ;
long           previousMillis3          =     0 ;
long           previousMillis4          =     0 ;
long           previousMillis5          =     0 ;

ALTAIR_GlobalMotorControl   motorControl        ;
ALTAIR_GlobalDeviceControl  deviceControl       ;
ALTAIR_GlobalLightControl   lightControl        ;
TinyGPSPlus                 gps                 ;           // nav mast GPS on I2C (addr = 0x42), and also UM7 with DFRobot GPS input on Serial3

void setup() {

  Serial.begin(9600);

  deviceControl.initializeAllDevices();
  
// normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green laser on, but that heats up the I-drive transistor too much)
  lightControl.initializeAllLightSources();
  
  Serial.println(F("I2C/TWI bus and device initialization complete.  Now initializing all motors ..."));

  if (!motorControl.initializeAllMotors())
  {
    Serial.println("Initialization of motors failed\n\r");
    while(1);
  } 

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

  deviceControl.sitAwareSystem()->arduinoMicro()->getDataAfterInterval(450);

  sendStatusToPrimaryRadioAndReadCommandsAtInterval(1000);

//  delay(100);
  if (backupRadiosOn) sendStationNameToBackupRadiosAtInterval(1333);
//  delay(100);

  sendGPSCompassStatusToComputerAtInterval(5000);

  printNavMastSensorValsAndAdjSettingsAtInterval(2000);

  storeDataOnMicroSDCard();
  

//  delay(100);

}

void storeDataOnMicroSDCard() {

}

void printNavMastSensorValsAndAdjSettingsAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis5 > interval) { 
    previousMillis5 = currentMillis;

// First, the BME280 temp/pres/hum
    deviceControl.sitAwareSystem()->bmeMastPrintInfo();
  
// Next, the BNO055 orientation
    deviceControl.sitAwareSystem()->orientSensors()->bno055()->printInfo();

// And lastly, the Sparkfun HMC6343 magnetometer
    deviceControl.sitAwareSystem()->orientSensors()->hmc6343()->printInfo();

// Now, see if there is serial input for settings adjustment

    byte    inputByte, inputByte2;
    boolean thingsHaveChanged = false;
    int     channelToModify   = 0;
    if (Serial.available()) {
        inputByte = Serial.read();
        if (Serial.available()) inputByte2 = Serial.read();
        performCommand(inputByte, inputByte2);
    } 
 
  }

}

bool getGPSandHeadingAtInterval(long interval)
{
  bool retval = false;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis4 > interval) { 
    previousMillis4 = currentMillis;
    Serial.println("Getting heading and GPS");

     // First, get the magnetometer heading
    compassmagHeading = deviceControl.sitAwareSystem()->orientSensors()->hmc5883l()->getHeading();

// Then, get the GPS
    retval            = deviceControl.sitAwareSystem()->gpsSensors()->neom8n()->getGPS(  &gps   );
  }
  return retval;
}

void sendStationNameToBackupRadiosAtInterval(long interval)
{
  unsigned long currentMillis = millis();
  ALTAIR_GenTelInt* backup1 = deviceControl.telemSystem()->backup1();
  if (currentMillis - previousMillis3 > interval) { 
    Serial.println(F("Writing station name to the first backup radio"));
    previousMillis3 = currentMillis;
    lightControl.intSphereSource()->setLightsBackupRadio();
    lightControl.diffLEDSource()->setLightsBackupRadio();

    backup1->sendCallSign();
    backup1->sendEndMessage();

    delay(20);

    ALTAIR_GenTelInt* backup2 = deviceControl.telemSystem()->backup2();

    backup2->sendCallSign();
    backup2->sendEndMessage();

    delay(40);
    lightControl.intSphereSource()->resetLights();
    lightControl.diffLEDSource()->resetLights();

    byte* newterm = backup1->readALTAIRData();
    if (newterm[0] != 0) {
      if (newterm[1] != 0) {
        performCommand(newterm[0], newterm[1]);
      }
    }

  }
}


void sendStatusToPrimaryRadioAndReadCommandsAtInterval(long interval)
{
  unsigned long currentMillis = millis();
  ALTAIR_GenTelInt* primary = deviceControl.telemSystem()->primary();
  if (!primary->isBusy() && currentMillis - previousMillis2 > interval) {
    previousMillis2 = currentMillis;
    Serial.println(F("*** Writing status to the primary radio ***"));
    lightControl.intSphereSource()->setLightsPrimaryRadio();
    lightControl.diffLEDSource()->setLightsPrimaryRadio();

    primary->sendGPS(gps);

    delay(40);
    lightControl.intSphereSource()->resetLights();
    lightControl.diffLEDSource()->resetLights();

    byte* newterm = primary->readALTAIRData();
    if (newterm[0] != 0) {
      if (newterm[1] != 0) {
        performCommand(newterm[0], newterm[1]);
      }
    }
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

void performCommand(byte byte0, byte byte1) {
  switch(byte0) {
// a motor or servo command
    case 's':
      motorControl.performCommand(byte1);
      break;
// a device (that is not a servo, motor, or light source) command
    case 'd':
      deviceControl.performCommand(byte1);
      break;
// a light source command
    case 'l':
      lightControl.performCommand(byte1);
      break;
// shut down all propulsion motors immediately
    case 'x':
    case 'X':
      motorControl.shutDownAllProps();
      break;
    default:
      break;
  }
}

