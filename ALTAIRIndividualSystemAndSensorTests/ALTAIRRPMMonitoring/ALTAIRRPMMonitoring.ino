

#include <Wire.h>
#include "ALTAIR_RPMSensor.h"

unsigned short          packedRPM[RPM_SENSOR_COUNT];
byte          packedRPS_byte[RPM_SENSOR_COUNT];

const uint8_t RPM_sensor_pins[RPM_SENSOR_COUNT] = {A0, A1};
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
    packedRPM[i] = arrayof_RPMSensors[i].packRPM(arrayof_RPMSensors[i].rpm());
    packedRPS_byte[i] = arrayof_RPMSensors[i].packRPS_byte(arrayof_RPMSensors[i].rpm());

  }   

  // Print measurements

  Serial.println("RPM Sensors: ");
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {

    Serial.print("# of Edges  ");
    Serial.print(arrayof_RPMSensors[i].risingEdgeCounter()); Serial.print("  ");
    Serial.print(arrayof_RPMSensors[i].fallingEdgeCounter()); Serial.print(" --> ");
  
    //Serial.print(arrayof_RPMSensors[i]._rpm_rising); Serial.print("  "); 
    //Serial.println(arrayof_RPMSensors[i]._rpm_falling);
    //Serial.print(packedRPM[i]); Serial.print("  ");
    //Serial.print(packedRPS_byte[i]); Serial.print("   ");

    Serial.print(arrayof_RPMSensors[i].rpm()); Serial.println(" RPM ");

  }

  Serial.println(" ");
  Serial.println("----");
  delay(1000);

  
}
