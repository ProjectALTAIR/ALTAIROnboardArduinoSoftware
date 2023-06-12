// Our Arduino Micro code for monitoring the RPM and temperature of, and the current flowing 
//     through, each of the 4 propulsion motors and electronic speed controllers (ESCs).  
// The Micro acts as an I2C slave (with slave address 08), and it reports 16 signed byte values
//     (4 RPMs, 4 currents, and 8 temperatures) over I2C, which each correspond to the most 
//     recent measurements, when info is requested by the Grand Central I2C master.

#include <Wire.h>
#include <ALTAIR_RPMSensor.h>


byte packRPS(float theRPM) {
  float theRPS = theRPM/60.;
  byte packRPS;
  if (theRPS >= 0. && theRPS < 127.) {
    packRPS = theRPS;
  } else if (theRPS >= -128. && theRPS < 0.) {
    packRPS = theRPS + 256.;
  } else if (theRPS >= 127.) {
    packRPS = 127;
  } else {
    packRPS = 128;
  }
  return packRPS;
}

byte packTemp(float theTemp) {
  float scaledTemp = theTemp*2.;
  byte thePackedTemp;
  if (scaledTemp >= 0. && scaledTemp < 127.) {
    thePackedTemp = scaledTemp;
  } else if (scaledTemp >= -128. && scaledTemp < 0.) {
    thePackedTemp = scaledTemp + 256.;
  } else if (scaledTemp >= 127.) {
    thePackedTemp = 127;
  } else {
    thePackedTemp = 128;
  }
  return thePackedTemp;
}

byte packCurrent(float theCurrent) {
  float scaledCurrent = theCurrent*4.;
  byte packCurr;
  if (scaledCurrent >= 0. && scaledCurrent < 127.) {
    packCurr = scaledCurrent;
  } else if (scaledCurrent >= -128. && scaledCurrent < 0.) {
    packCurr = scaledCurrent + 256.;
  } else if (scaledCurrent >= 127.) {
    packCurr = 127;
  } else {
    packCurr = 128;
  }
  return packCurr;
}


const int RPM_sensorCount = 4;

byte          packedRPS[4];
byte          packedCurrent[4];
byte          packedTemp[8];



const uint8_t RPM_sensor_pins[4] = {A0, A1, A2, A3};
const int RPM_measurement_time = 500;

int i = 0;

int start_millis = 0;
int current_millis = 0;

// Instantiate 4 RPM Sensors
ALTAIR_RPMSensor arrayof_RPMSensors[4];

void setup() {
  
  Serial.println("Begin Sensor initialization");
  for(int i = 0; i<RPM_sensorCount; i++){
    arrayof_RPMSensors[i].initialize_QTRsensor((const uint8_t*) RPM_sensor_pins[i]);
    //arrayof_RPMSensors[i].RPMSensor.setTypeAnalog();
    arrayof_RPMSensors[i].RPMSensor()->setSensorPins((const uint8_t*) { RPM_sensor_pins[i] }, (const uint8_t) 1);

    Serial.println("RPM Sensors initialized");
  }
  
  
  
  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);
 
  /*for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < numCurrentValsToAverage; ++j) currentInAmps[i][j] = 0.;
  }
  // Initialize digital RPM timer pins as input.  (Note that the analog-read pins don't require this initialization.)
  for (int i = 0; i < 4; ++i) {
    pinMode(rpmTimerPin[i], INPUT);
  }
  // Initialize special SPI MISO pin = 14 as an input (to see if device has its USB port plugged in)
  pinMode(usbInputCheckPin, INPUT);*/
}

void loop() {
  current_millis = millis();
  start_millis = current_millis;
  //qtr.read(sensorValues);
    //Serial.println(sensorValues[0]);
  Serial.println("Start of measurement");
  for ( int i = 0; i < RPM_sensorCount; i++) {
    arrayof_RPMSensors[i].resetRisingEdge_counter();
    arrayof_RPMSensors[i].resetFallingEdge_counter();
    Serial.print(arrayof_RPMSensors[i].analog_input_pin()); Serial.print("  ");
  }
  Serial.println(" ");
  while(current_millis-start_millis < RPM_measurement_time){
    for (int i = 0; i < RPM_sensorCount; i++) {    
      arrayof_RPMSensors[i].store_analog_RPM();
      arrayof_RPMSensors[i].Edge_detection();
    }
    //Serial.println(arrayof_RPMSensors[0].analog_rpm);
    current_millis = millis();
    //Serial.print("millis: "); Serial.println(current_millis-start_millis);    
  }
  Serial.println("End of measurement ");
    
  Serial.println("Current RPM values: ");
  for (int i = 0; i < RPM_sensorCount; i++) {
    arrayof_RPMSensors[i].calculate_RPM(RPM_measurement_time);

    Serial.print("# of Edges  ");
    Serial.print(arrayof_RPMSensors[i].risingEdge_counter()); Serial.print("  ");
    Serial.print(arrayof_RPMSensors[i].fallingEdge_counter()); Serial.print(" --> ");
  
    //Serial.print(arrayof_RPMSensors[i]._rpm_rising); Serial.print("  "); 
    //Serial.println(arrayof_RPMSensors[i]._rpm_falling);
  
    Serial.print(arrayof_RPMSensors[i].rpm()); Serial.println(" RPM ");
    packedRPS[i] = arrayof_RPMSensors[i].packRPS(arrayof_RPMSensors[i].rpm());
  }

  Serial.println(" ");
  Serial.println("----");
  delay(5000);

  
  
  /* OLD CODE---------------------------------------------
  double rpm[4] = { 0., 0., 0., 0. };
  float  currentRunningAverage[4] = { 0., 0., 0., 0. };

  for (int i = 0; i < 4; ++i) {
    currentSensorValue[i] = analogRead(currentSensorPin[i]);
    for (int j = 0; j < numCurrentValsToAverage - 1; ++j) {
      currentRunningAverage[i] += currentInAmps[i][j+1];
      currentInAmps[i][j] = currentInAmps[i][j+1];
    }
    currentInAmps[i][numCurrentValsToAverage-1] = (505-currentSensorValue[i])/3.3;
    currentRunningAverage[i] += currentInAmps[i][numCurrentValsToAverage-1];
    currentRunningAverage[i] /= numCurrentValsToAverage;
    packedCurrent[i] = packCurrent(currentRunningAverage[i]);
  }
  for (int i = 0; i < 8; ++i) {
    tempSensorValue[i] = analogRead(tempSensorPin[i]);
    tempInCelsius[i] = 22 + (tempSensorValue[i]-535)/2.;
    packedTemp[i] = packTemp(tempInCelsius[i]);
  }
  for (int i = 0; i < 4; ++i) {
    rpm[i] = getRPM(rpmTimerPin[i]);
    packedRPM[i] = packRPM(rpm[i]);
  }


  if (digitalRead(usbInputCheckPin) == HIGH) {      // i.e., if the USB port is actually connected.
                                                    // Pin 14 is actually the MISO pin on the Arduino Micro 
                                                    // (see e.g. https://forum.arduino.cc/index.php?topic=337715.0 ).
                                                    // I have connected this to the MF-MSMF050-2 fuse (VUSB) on the bottom of the board.
 
        Serial.print(currentSensorValue[0]); Serial.print(" "); 
        Serial.print(currentSensorValue[1]); Serial.print(" "); 
        Serial.print(currentSensorValue[2]); Serial.print(" "); 
        Serial.print(currentSensorValue[3]); Serial.print("      "); 
        Serial.print(tempSensorValue[0]); Serial.print(" "); 
        Serial.print(tempSensorValue[1]); Serial.print(" "); 
        Serial.print(tempSensorValue[2]); Serial.print(" "); 
        Serial.print(tempSensorValue[3]); Serial.print(" "); 
        Serial.print(tempSensorValue[4]); Serial.print(" "); 
        Serial.print(tempSensorValue[5]); Serial.print(" "); 
        Serial.print(tempSensorValue[6]); Serial.print(" "); 
        Serial.print(tempSensorValue[7]); Serial.println(" "); 
        
                           Serial.print(rpm[0]); Serial.print(" "); Serial.print(rpm[1]);            Serial.print(" ");
                           Serial.print(rpm[2]); Serial.print(" "); Serial.print(rpm[3]);            Serial.print("      "); 
        Serial.print(currentRunningAverage[0]);  Serial.print(" "); 
        Serial.print(currentRunningAverage[1]);  Serial.print(" ");
        Serial.print(currentRunningAverage[2]);  Serial.print(" ");
        Serial.print(currentRunningAverage[3]);  Serial.print("      ");
//        Serial.print(" Amps     Temps: "); Serial.print(tempSensorValue); Serial.print(" = "); 
        Serial.print(tempInCelsius[0]);          Serial.print(" "); Serial.print(tempInCelsius[1]);  Serial.print(" ");
        Serial.print(tempInCelsius[2]);          Serial.print(" "); Serial.print(tempInCelsius[3]);  Serial.print(" ");
        Serial.print(tempInCelsius[4]);          Serial.print(" "); Serial.print(tempInCelsius[5]);  Serial.print(" ");
        Serial.print(tempInCelsius[6]);          Serial.print(" "); Serial.print(tempInCelsius[7]);  Serial.println("      ");
        
//  for (int j = 0; j < numRPMPulsesToAverage; ++j) { Serial.print(rpmPulseDuration[0][j]); Serial.print(" "); } Serial.println(" ");
                           Serial.print(packedRPM[0], HEX); Serial.print(" "); Serial.print(packedRPM[1], HEX);            Serial.print(" ");
                           Serial.print(packedRPM[2], HEX); Serial.print(" "); Serial.print(packedRPM[3], HEX);            Serial.print("      "); 
        Serial.print(packedCurrent[0], HEX);  Serial.print(" "); 
        Serial.print(packedCurrent[1], HEX);  Serial.print(" ");
        Serial.print(packedCurrent[2], HEX);  Serial.print(" ");
        Serial.print(packedCurrent[3], HEX);  Serial.print("      ");
        Serial.print(packedTemp[0], HEX);     Serial.print(" "); Serial.print(packedTemp[1], HEX);  Serial.print(" ");
        Serial.print(packedTemp[2], HEX);     Serial.print(" "); Serial.print(packedTemp[3], HEX);  Serial.print(" ");
        Serial.print(packedTemp[4], HEX);     Serial.print(" "); Serial.print(packedTemp[5], HEX);  Serial.print(" ");
        Serial.print(packedTemp[6], HEX);     Serial.print(" "); Serial.print(packedTemp[7], HEX);  Serial.println("      "); Serial.println("      ");
  }
  OLD CODE----------------------------------------------------------------------------------
  */
}

void sendInfo() {
  for (int i = 0; i < 4; ++i) Wire.write(packedRPS[i]);
  for (int i = 0; i < 4; ++i) Wire.write(packedCurrent[i]);
  for (int i = 0; i < 8; ++i) Wire.write(packedTemp[i]);  
}
