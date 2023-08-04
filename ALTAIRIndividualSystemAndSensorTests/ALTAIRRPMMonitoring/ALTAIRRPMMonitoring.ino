

#include <Wire.h>
#include "ALTAIR_RPMSensor.h"

unsigned short          packedRPM_short[RPM_SENSOR_COUNT];
byte          packedRPS[RPM_SENSOR_COUNT];

const uint8_t RPM_sensor_pins[RPM_SENSOR_COUNT] = {A0, A1, A2, A3};
const int RPM_measurement_time = 500;

int start_millis = 0;
int current_millis = 0;

// Instantiate Sensors
ALTAIR_RPMSensor      arrayof_RPMSensors[RPM_SENSOR_COUNT];

void setup() {
  
  Serial.println("Begin Sensor initialization");
  // RPM Sensors
  for(int i = 0; i<RPM_SENSOR_COUNT; i++){
    arrayof_RPMSensors[i].initializeQTRsensor((const uint8_t*) RPM_sensor_pins[i]);
    //arrayof_RPMSensors[i].RPMSensor.setTypeAnalog();
    arrayof_RPMSensors[i].RPMSensor()->setSensorPins((const uint8_t*) { RPM_sensor_pins[i] }, (const uint8_t) 1);

    Serial.println("RPM Sensors initialized");
  }
 
  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);
 
}

void loop() {
  Serial.println("Start of measurement");

  // RPM measurements
  current_millis = millis();
  start_millis = current_millis;

  for ( int i = 0; i < RPM_SENSOR_COUNT; i++) {
    arrayof_RPMSensors[i].resetRisingEdgeCounter();
    arrayof_RPMSensors[i].resetFallingEdgeCounter();
  }
  
  while(current_millis-start_millis < RPM_measurement_time){
    for (int i = 0; i < RPM_SENSOR_COUNT; i++) {    
      arrayof_RPMSensors[i].storeAnalogRPM();
      arrayof_RPMSensors[i].EdgeDetection();
    }
    //Serial.println(arrayof_RPMSensors[0].analog_rpm);
    current_millis = millis();
    //Serial.print("millis: "); Serial.println(current_millis-start_millis);    
  }
  Serial.println("End of measurement ");

  // RPM Calculation
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {
    arrayof_RPMSensors[i].calculateRPM(RPM_measurement_time);
    packedRPM_short[i] = arrayof_RPMSensors[i].packRPM_short(arrayof_RPMSensors[i].rpm());
    packedRPS[i] = arrayof_RPMSensors[i].packRPS(arrayof_RPMSensors[i].rpm());

  }   

  // Print measurements

  Serial.println("RPM Sensors: ");
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {

    Serial.print("# of Edges  ");
    Serial.print(arrayof_RPMSensors[i].risingEdgeCounter()); Serial.print("  ");
    Serial.print(arrayof_RPMSensors[i].fallingEdgeCounter()); Serial.print(" --> ");
  
    //Serial.print(arrayof_RPMSensors[i]._rpm_rising); Serial.print("  "); 
    //Serial.println(arrayof_RPMSensors[i]._rpm_falling);
    //Serial.print(packedRPM_short[i]); Serial.print("  ");
    //Serial.print(packedRPS[i]); Serial.print("   ");

    Serial.print(arrayof_RPMSensors[i].rpm()); Serial.println(" RPM ");

  }

  Serial.println(" ");
  Serial.println("----");
  delay(1000);

  
}

void sendInfo() {
  //Wire.write((byte*)packedRPM_short, RPM_SENSOR_COUNT * sizeof(unsigned short));

  Wire.write((byte*)packedRPS, RPM_SENSOR_COUNT * sizeof(byte));

}