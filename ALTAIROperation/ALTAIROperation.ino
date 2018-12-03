// Our Arduino code for full operation of the ALTAIR payload
#include <ALTAIR_GlobalMotorControl.h>
#include <ALTAIR_GlobalDeviceControl.h>
#include <ALTAIR_GlobalLightControl.h>

bool           backupRadiosOn             =  true ;        // If this is set to false, then _neither_ backup radio will be on.
bool           backupRadio2On             =  true ;        // If this is set to true, _and_ if backupRadiosOn is _also_ set to true, then backupRadio2 will be 
                                                           //    initialized and will transmit and receive.  (Otherwise, backupRadio2 will not be initialized.)
unsigned long  previousMillis[6]                  ;
unsigned long  commandTimeoutInterval     =  2000 ;        // in milliseconds
float          compassmagHeading          =  -999.;        // will be set to the heading in degrees East of true North, uncorrected for magnetic declination angle

ALTAIR_GlobalMotorControl   motorControl          ;
ALTAIR_GlobalDeviceControl  deviceControl         ;
ALTAIR_GlobalLightControl   lightControl          ;

void setup() {

  Serial.begin(38400);

  deviceControl.initializeAllDevices(backupRadiosOn, backupRadio2On);
  
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

  sendStatusToPrimaryRadioAtInterval(1000);

//  delay(100);
  if (backupRadiosOn) sendStationNameToBackupRadiosAtInterval(1333);
//  delay(100);

  sendGPSCompassStatusToComputerAtInterval(5000);

  printNavMastSensorValsAndAdjSettingsAtInterval(2000);

  storeDataOnMicroSDCard();

  readCommands();
  
//  delay(100);

}

void storeDataOnMicroSDCard() {

  deviceControl.dataStoreSystem()->storeTimestamp( deviceControl.sitAwareSystem()->gpsSensors()->primary() );   

}

void readCommands() {
  byte command[2] = { 0, 0 };
  unsigned long currentMillis = millis();

  if (backupRadiosOn && backupRadio2On) deviceControl.telemSystem()->rfm23bp()->readALTAIRInfo( command );
  if (command[0] != 0) {
    if (command[1] != 0) {
      if (currentMillis - previousMillis[5] > commandTimeoutInterval) {
        Serial.print(F("RFM23BP command[0] = ")); Serial.print(command[0], HEX); Serial.print(F("  command[1] = ")); Serial.println(command[1], HEX);
        performCommand(command[0], command[1]);
        previousMillis[5] = currentMillis;
      }
    }
  }

  if (backupRadiosOn) deviceControl.telemSystem()->shx144()->readALTAIRInfo( command );
  if (command[0] != 0) {
    if (command[1] != 0) {
      if (currentMillis - previousMillis[5] > commandTimeoutInterval) {
        Serial.print(F("SHX144 command[0] = ")); Serial.print(command[0], HEX); Serial.print(F("  command[1] = ")); Serial.println(command[1], HEX);
        performCommand(command[0], command[1]);
        previousMillis[5] = currentMillis;
      }
    }
  }

  deviceControl.telemSystem()->dnt900()->readALTAIRInfo( command );
  if (command[0] != 0) {
    if (command[1] != 0) {
      if (currentMillis - previousMillis[5] > commandTimeoutInterval) {
        Serial.print(F("DNT900 command[0] = ")); Serial.print(command[0], HEX); Serial.print(F("  command[1] = ")); Serial.println(command[1], HEX);
        performCommand(command[0], command[1]);
        previousMillis[5] = currentMillis;
      }
    }
  }

}

void printNavMastSensorValsAndAdjSettingsAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis[4] > interval) { 
    previousMillis[4] = currentMillis;

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
  if (currentMillis - previousMillis[3] > interval) { 
    previousMillis[3] = currentMillis;
    Serial.println("Getting heading and GPS");

     // First, get the magnetometer heading
    compassmagHeading = deviceControl.sitAwareSystem()->orientSensors()->hmc5883l()->getHeading();

// Then, get the GPS
    retval            = deviceControl.sitAwareSystem()->gpsSensors()->primary()->getGPS();
  }
  return retval;
}

void sendStationNameToBackupRadiosAtInterval(long interval)
{
  unsigned long currentMillis = millis();
  ALTAIR_GenTelInt* backup1 = deviceControl.telemSystem()->backup1();
  if (currentMillis - previousMillis[2] > interval) { 
    Serial.print(F("Writing station name to the first backup radio: ")); Serial.println(backup1->radioName());
    previousMillis[2] = currentMillis;
    lightControl.intSphereSource()->setLightsBackupRadio();
    lightControl.diffLEDSource()->setLightsBackupRadio();

    if (!(backup1->sendCallSign()))   Serial.println(F("Could not send call sign to backup1 radio!"));
    if (!(backup1->sendEndMessage())) Serial.println(F("Could not send end message to backup1 radio!"));

    delay(20);

    if (backupRadio2On) {
      ALTAIR_GenTelInt* backup2 = deviceControl.telemSystem()->backup2();

      if (!(backup2->sendCallSign()))   { Serial.print(F("Could not send call sign to backup2 radio!: "));   Serial.println(backup2->radioName()); }
      if (!(backup2->sendEndMessage())) { Serial.print(F("Could not send end message to backup2 radio!: ")); Serial.println(backup2->radioName()); }
    }

    delay(40);
    lightControl.intSphereSource()->resetLights();
    lightControl.diffLEDSource()->resetLights();

    Serial.println(F("done with backup radios"));
/*
    byte command[2];
    backup1->readALTAIRInfo( command );
    if (command[0] != 0) {
      if (command[1] != 0) {
        performCommand(command[0], command[1]);
      }
    }
*/

  }
}


void sendStatusToPrimaryRadioAtInterval(long interval)
{
  unsigned long currentMillis = millis();
  ALTAIR_GenTelInt* primary = deviceControl.telemSystem()->primary();
  delay(40);

  if (!primary->isBusy() && currentMillis - previousMillis[1] > interval) {
    previousMillis[1] = currentMillis;
   
    Serial.print(F("*** Writing status to the primary radio: "));  Serial.println(primary->radioName());
    lightControl.intSphereSource()->setLightsPrimaryRadio();
    lightControl.diffLEDSource()->setLightsPrimaryRadio();

//     primary->sendGPS(        deviceControl.sitAwareSystem()->gpsSensors()->primary()        );
    primary->sendAllALTAIRInfo( motorControl  ,
                                deviceControl ,
                                lightControl    );

    delay(40);
    lightControl.intSphereSource()->resetLights();
    lightControl.diffLEDSource()->resetLights();

  }
}


void sendGPSCompassStatusToComputerAtInterval(long interval) {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis[0] > interval) {
    ALTAIR_GPSSensor* gps = deviceControl.sitAwareSystem()->gpsSensors()->primary();
    previousMillis[0] = currentMillis;   

    Serial.print(F("ALTAIR Latitude: "));    Serial.println(gps->lat());
    Serial.print(F("ALTAIR Longitude: "));   Serial.println(gps->lon());
    Serial.print(F("ALTAIR LatLong Age: ")); Serial.println(gps->age());
    Serial.print(F("ALTAIR Year: "));        Serial.println(gps->year());
    Serial.print(F("ALTAIR Month: "));       Serial.println(gps->month());
    Serial.print(F("ALTAIR Day: "));         Serial.println(gps->day());
    Serial.print(F("ALTAIR Hour: "));        Serial.println(gps->hour());
    Serial.print(F("ALTAIR Minute: "));      Serial.println(gps->minute());
    Serial.print(F("ALTAIR Second: "));      Serial.println(gps->second());
//    Serial.print(F("ALTAIR Hundredth: "));   Serial.println(gps.time.centisecond());
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

