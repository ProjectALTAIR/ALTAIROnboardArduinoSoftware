// Our Arduino Micro code for monitoring the RPM and temperature of, and the current flowing 
//     through, each of the 4 propulsion motors and electronic speed controllers (ESCs).  
// The Micro acts as an I2C slave (with slave address 08), and it reports 16 signed byte values
//     (4 RPMs, 4 currents, and 8 temperatures) over I2C, which each correspond to the most 
//     recent measurements, when info is requested by the Grand Central I2C master.

#include <Wire.h>
#include "ALTAIR_RPMSensor.h"
#include "ALTAIR_TempSensor.h"
#include "ALTAIR_CurrentSensor.h"



/*  Direct Analog read for half the sensor setup
  const uint8_t RPM_sensor_pins[RPM_SENSOR_COUNT] = {A0, A1};

  int tempSensorPins[TEMP_SENSOR_COUNT] = {A4, A5, A6, A7};

  int currentSensorPins[CURRENT_SENSOR_COUNT] = {A2, A3};
*/

// Using the 16-to-1 analog multiplexer to a common analog input A0 : CD74HC4067 Multiplexer used
  const uint8_t RPM_sensor_pins[RPM_SENSOR_COUNT] = {A0, A0, A0, A0};

  const int tempSensorPins[TEMP_SENSOR_COUNT] = {A0, A0, A0, A0, A0, A0, A0, A0};

  const int currentSensorPins[CURRENT_SENSOR_COUNT] = {A0, A0, A0, A0};

  const int multiplexerControlPins[4] = {5, 6, 7, 8};
  const int multiplexerEnablePin      = 4;

  /*  MULTIPLEXER LIST
    0....7   Temperature Sensors Port(left) to Starboard(right)
    8....11  RPM Sensors Port to Starboard
    12...15  Current Sensors Port to Starboard
  */

const int RPM_measurement_time = 500;

int start_millis = 0;
int current_millis = 0;

// Instantiate Sensors
ALTAIR_RPMSensor      arrayof_RPMSensors[RPM_SENSOR_COUNT];
ALTAIR_TempSensor     arrayofTempSensors[TEMP_SENSOR_COUNT];
ALTAIR_CurrentSensor  arrayofCurrentSensors[CURRENT_SENSOR_COUNT];


// packed Sensor data for transfer to Grand Central

  unsigned short    packedRPM_short[RPM_SENSOR_COUNT];
  unsigned short    packedCurrent_short[CURRENT_SENSOR_COUNT];
  unsigned short    packedTemp_short[TEMP_SENSOR_COUNT];


byte              packedRPS[RPM_SENSOR_COUNT];
byte              packedCurrent[CURRENT_SENSOR_COUNT];
byte              packedTemp[TEMP_SENSOR_COUNT];

void setup() {
  
  // Initializing MultiplexerControlPins
  for(int i = 0; i<4; i++) {
    pinMode(multiplexerControlPins[i], OUTPUT);
  }
  pinMode(multiplexerEnablePin, OUTPUT);
  digitalWrite(multiplexerEnablePin, LOW);    // LOW enables the Multiplexer! See logic table in datasheet


  //Serial.println("Begin Sensor initialization");
  
  // RPM Sensors
  for(int i = 0; i<RPM_SENSOR_COUNT; i++){
    arrayof_RPMSensors[i].initializeQTRsensor((const uint8_t*) RPM_sensor_pins[i]);
    //arrayof_RPMSensors[i].RPMSensor.setTypeAnalog();
    arrayof_RPMSensors[i].RPMSensor()->setSensorPins((const uint8_t*) { RPM_sensor_pins[i] }, (const uint8_t) 1);

    Serial.println("RPM Sensors initialized");
  }
  
  // Temperature Sensors
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
    arrayofTempSensors[i].initializeTempSensor(tempSensorPins[i]);
  }
      Serial.println("Temperature Sensors initialized");

  // Current Sensors
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
    arrayofCurrentSensors[i].initializeCurrentSensor(currentSensorPins[i]);
  }
      Serial.println("Current Sensors initialized"); 

  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);
 
  
  // Initialize special SPI MISO pin = 14 as an input (to see if device has its USB port plugged in)
  //pinMode(usbInputCheckPin, INPUT);*/
}

void loop() {
  //Serial.println("Start of measurement");

  // TEMPERATURE SENSOR measurements
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
    selectMultiplexerChannel(i);

    arrayofTempSensors[i].storeAnalogTemp();
    arrayofTempSensors[i].calculateTemp();
    packedTemp_short[i] = arrayofTempSensors[i].packTemp_short(arrayofTempSensors[i].tempK());
    packedTemp[i] = arrayofTempSensors[i].packTemp(arrayofTempSensors[i].temp());

    //Serial.println(arrayofTempSensors[i].tempWindowSum());
  }

  
  // RPM SENSOR measurements
  
  for ( int i = 0; i < RPM_SENSOR_COUNT; i++) {
    arrayof_RPMSensors[i].resetRisingEdgeCounter();
    arrayof_RPMSensors[i].resetFallingEdgeCounter();
  }
    
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {
    selectMultiplexerChannel(i + TEMP_SENSOR_COUNT);

    current_millis = millis();
    start_millis = current_millis;

    while(current_millis-start_millis < RPM_measurement_time) {    
      arrayof_RPMSensors[i].storeAnalogRPM();
      arrayof_RPMSensors[i].EdgeDetection();
      current_millis = millis();
    }
    //Serial.println(arrayof_RPMSensors[0].analog_rpm);    
    //Serial.print("millis: "); Serial.println(current_millis-start_millis);    
  }
  

  // CURRENT SENSOR measurements
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
    selectMultiplexerChannel(i + TEMP_SENSOR_COUNT + RPM_SENSOR_COUNT);

    arrayofCurrentSensors[i].storeAnalogCurrent();
    arrayofCurrentSensors[i].calculateCurrent();
    packedCurrent_short[i] = arrayofCurrentSensors[i].packCurrent_short(arrayofCurrentSensors[i].current());
    packedCurrent[i] = arrayofCurrentSensors[i].packCurrent(arrayofCurrentSensors[i].current());
  }

  //Serial.println("End of measurement ");


  // RPM Calculation
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {
    arrayof_RPMSensors[i].calculateRPM(RPM_measurement_time);
    packedRPM_short[i] = arrayof_RPMSensors[i].packRPM_short(arrayof_RPMSensors[i].rpm());
    packedRPS[i] = arrayof_RPMSensors[i].packRPS(arrayof_RPMSensors[i].rpm());

  }   

  // Print measurements
  Serial.println("Temperature Sensors:");
  for ( int i = 0; i < TEMP_SENSOR_COUNT; i++) {
      Serial.print(static_cast<float>(arrayofTempSensors[i].tempWindowSum())/TEMP_AVEREGING_WINDOW_SIZE);
      //Serial.print(packedTemp_short[i]); Serial.print("  ");
      //Serial.print(packedTemp[i]);
      
      Serial.print(" --> ");
      Serial.print(arrayofTempSensors[i].temp()); Serial.println(" °C");

  }

  Serial.println("RPM Sensors: ");
  for (int i = 0; i < RPM_SENSOR_COUNT; i++) {

    //Serial.print("# of Edges  ");
    //Serial.print(arrayof_RPMSensors[i].risingEdgeCounter()); Serial.print("  ");
    //Serial.print(arrayof_RPMSensors[i].fallingEdgeCounter()); Serial.print(" --> ");
  
    //Serial.print(arrayof_RPMSensors[i]._rpm_rising); Serial.print("  "); 
    //Serial.println(arrayof_RPMSensors[i]._rpm_falling);
    Serial.print(packedRPM_short[i]); Serial.print("  ");
    Serial.print(packedRPS[i]); Serial.print("   ");

    Serial.print(arrayof_RPMSensors[i].rpm()); Serial.println(" RPM ");

  }

  Serial.println("Current Sensors:");
  for ( int i = 0; i < CURRENT_SENSOR_COUNT; i++) {
      Serial.print(arrayofCurrentSensors[i].currentWindowSum()/CURRENT_AVEREGING_WINDOW_SIZE); Serial.print("  ");
      //Serial.print(packedCurrent_short[i]); Serial.print("  ");
      Serial.print(packedCurrent[i]); Serial.print(" --> ");
      Serial.print(arrayofCurrentSensors[i].current()); Serial.println(" mA");
  }

  Serial.println(" ");
  Serial.println("----");
  delay(1000);

  
  
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

// Wanted channel gets translated into binary with that being input to the Controls S0-S3 connected to pins 4-7 starting with LSB
void selectMultiplexerChannel(int channel) {
  //Serial.println(channel); 
  //Serial.print("  ");
  for(int i = 0; i < 4; i++) {
    digitalWrite(multiplexerControlPins[i], bitRead(channel, i));
    //Serial.print(bitRead(channel, i));    
  }
  delay(5);
}


// Send packed sensor data via I2C to Grand Central. Receiver side is handled in ArduinoMicro Class
void sendInfo() {
  //Wire.write((byte*)packedTemp_short, TEMP_SENSOR_COUNT * sizeof(unsigned short));
  //Wire.write((byte*)packedCurrent_short, CURRENT_SENSOR_COUNT * sizeof(unsigned short));
  //Wire.write((byte*)packedRPM_short, RPM_SENSOR_COUNT * sizeof(unsigned short));

  Wire.write((byte*)packedTemp, TEMP_SENSOR_COUNT * sizeof(byte));
  Wire.write((byte*)packedCurrent, CURRENT_SENSOR_COUNT * sizeof(byte));
  Wire.write((byte*)packedRPS, RPM_SENSOR_COUNT * sizeof(byte));

}
