/*
 * Test File for the Temperature Sensor used in ALTAIR
 * LM335 Z Sensor
 * 
 * Written by Christopher Vogt
 * June 2023
 */

#include <Wire.h>
#include "ALTAIR_TempSensor.h"


unsigned short          packedTemp[TEMP_SENSOR_COUNT];
byte                    packedTemp_byte[TEMP_SENSOR_COUNT];

int tempSensorPins[TEMP_SENSOR_COUNT] = {A4, A5, A6, A7};

ALTAIR_TempSensor     arrayofTempSensors[TEMP_SENSOR_COUNT];


void setup() {
  Serial.println("Begin Sensor initialization");
  // Temperature Sensors
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
    arrayofTempSensors[i].initializeTempSensor(tempSensorPins[i]);
  }
  Serial.println("Temperature Sensors initialized");

  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);
}

void loop() {
  Serial.println("Start of measurement");

  // Temperature Sensor measurements
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
    arrayofTempSensors[i].storeAnalogTemp();
    arrayofTempSensors[i].calculateTemp();
    packedTemp[i] = arrayofTempSensors[i].packTemp(arrayofTempSensors[i].tempK());
    packedTemp_byte[i] = arrayofTempSensors[i].packTemp_byte(arrayofTempSensors[i].temp());
  }

  // Print measurements
  Serial.println("Temperature Sensors:");
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
      Serial.print("Averaged analog input: ");
      Serial.print(static_cast<float>(arrayofTempSensors[i].tempWindowSum())/TEMP_AVEREGING_WINDOW_SIZE);
      Serial.print("  Packed for I2C (unsigned short/byte); ")
      Serial.print(packedTemp[i]); Serial.print(" / ");
      Serial.print(packedTemp_byte[i]);
      
      Serial.print(" ---> ");
      Serial.print(arrayofTempSensors[i].temp()); Serial.println(" °C");
  }
 
  Serial.println("----");
  delay(1000);

}

void sendInfo() {
  //Wire.write((byte*)packedTemp, TEMP_SENSOR_COUNT * sizeof(unsigned short));

  Wire.write((byte*)packedTemp_byte, TEMP_SENSOR_COUNT * sizeof(byte));

  //for (int i = 0; i < tempSensorCount; ++i) Wire.write((uint8_t*)packedTemp[i], sizeof(packedTemp[i]));  
}
