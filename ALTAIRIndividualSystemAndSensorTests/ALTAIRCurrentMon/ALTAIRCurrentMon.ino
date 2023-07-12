/*
 * Test File for the Current Sensor used in ALTAIR
 * INA169 Analog Current sensor on Adafruit breakout board https://www.adafruit.com/product/1164#technical-details
 * 
 * SetUp on AdaFruit Grand Central for the moment
 * 
 * Written by Christopher Vogt
 * May 2023
 */

const int     currentSensorPin[4]       =     { A4, A5, A6, A7 };
int           currentSensorValue[4];
const int     numCurrentValsToAverage   =       20;
float         currentInAmps[4][numCurrentValsToAverage];  // average the past 20 values
byte          packedCurrent[4];

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

void setup() {
  
  Serial.begin(9600);
  // initialize Current arrays
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < numCurrentValsToAverage; ++j) currentInAmps[i][j] = 0.;
  }
}

void loop() {
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
  Serial.print("Analog pin input values:  ");
  Serial.print(currentSensorValue[0]); Serial.print(" "); 
  Serial.print(currentSensorValue[1]); Serial.print(" "); 
  Serial.print(currentSensorValue[2]); Serial.print(" "); 
  Serial.print(currentSensorValue[3]); Serial.println("      ");

  Serial.print("Current running average:  ");
  Serial.print(currentRunningAverage[0]);  Serial.print(" "); 
  Serial.print(currentRunningAverage[1]);  Serial.print(" ");
  Serial.print(currentRunningAverage[2]);  Serial.print(" ");
  Serial.print(currentRunningAverage[3]);  Serial.println("      ");

  Serial.print("packed Current measurement:  ");
  Serial.print(packedCurrent[0], HEX);  Serial.print(" "); 
  Serial.print(packedCurrent[1], HEX);  Serial.print(" ");
  Serial.print(packedCurrent[2], HEX);  Serial.print(" ");
  Serial.print(packedCurrent[3], HEX);  Serial.println("      ");
  Serial.println("---");
  delay(500);
}
