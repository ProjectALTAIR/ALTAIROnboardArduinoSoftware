/*
 * Test File for the Current Sensor used in ALTAIR
 * INA169 Analog Current sensor on Adafruit breakout board https://www.adafruit.com/product/1164#technical-details
 * 
 * 
 * Written by Christopher Vogt
 * June 2023
 */

#include <Wire.h>
#include "ALTAIR_CurrentSensor.h"


unsigned short          packedCurrent[CURRENT_SENSOR_COUNT];
byte                    packedCurrent_byte[CURRENT_SENSOR_COUNT];

int currentSensorPins[CURRENT_SENSOR_COUNT] = {A2, A3};

ALTAIR_CurrentSensor  arrayofCurrentSensors[CURRENT_SENSOR_COUNT];


void setup() {
  Serial.println("Begin Sensor initialization");

  // Current Sensors
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
    arrayofCurrentSensors[i].initializeCurrentSensor(currentSensorPins[i]);
  }
      Serial.println("Current Sensors initialized"); 

  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);

}

void loop() {
  Serial.println("Start of measurement");

  // Current Sensor measurements
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
    arrayofCurrentSensors[i].storeAnalogCurrent();
    arrayofCurrentSensors[i].calculateCurrent();
    packedCurrent[i] = arrayofCurrentSensors[i].packCurrent(arrayofCurrentSensors[i].current());
    packedCurrent_byte[i] = arrayofCurrentSensors[i].packCurrent_byte(arrayofCurrentSensors[i].current());
  }
  Serial.println("End of measurement ");

  // Print measurements

  Serial.println("Current Sensors:");
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
      Serial.print(arrayofCurrentSensors[i].currentWindowSum()/CURRENT_AVEREGING_WINDOW_SIZE); Serial.print("  ");
      Serial.print(packedCurrent[i]); Serial.print("  ");
      Serial.print(packedCurrent_byte[i]); Serial.print(" --> ");
      Serial.print(arrayofCurrentSensors[i].current()); Serial.println(" mA");
  }

  Serial.println("----");
  delay(1000);

}

void sendInfo() {
  //Wire.write((byte*)packedCurrent, CURRENT_SENSOR_COUNT * sizeof(unsigned short));

  Wire.write((byte*)packedCurrent_byte, CURRENT_SENSOR_COUNT * sizeof(byte));

  //for (int i = 0; i < currentSensorCount; ++i) Wire.write(packedCurrent[i], sizeof(packedCurrent[i]));
}

