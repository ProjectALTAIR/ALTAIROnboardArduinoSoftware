/*
 * Test File for the Temperature Sensor used in ALTAIR
 *  
 * SetUp on AdaFruit Grand Central Analog Input A8-A15 for the moment
 * 
 * Written by Christopher Vogt
 * May 2023
 */

const int     tempSensorPin[8]          =     { A8, A9, A10, A11, A12, A13, A14, A15  };
int           tempSensorValue[8];
float         tempInCelsius[8];
byte          packedTemp[8];


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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; ++i) {
    tempSensorValue[i] = analogRead(tempSensorPin[i]);
    tempInCelsius[i] = 22 + (tempSensorValue[i]-535)/2.;
    packedTemp[i] = packTemp(tempInCelsius[i]);
  }

  Serial.print("Analog pin input values:  ");
  Serial.print(tempSensorValue[0]); Serial.print(" "); 
  Serial.print(tempSensorValue[1]); Serial.print(" "); 
  Serial.print(tempSensorValue[2]); Serial.print(" "); 
  Serial.print(tempSensorValue[3]); Serial.print(" "); 
  Serial.print(tempSensorValue[4]); Serial.print(" "); 
  Serial.print(tempSensorValue[5]); Serial.print(" "); 
  Serial.print(tempSensorValue[6]); Serial.print(" "); 
  Serial.print(tempSensorValue[7]); Serial.println(" ");

  Serial.println("Temperature in Celsius:  ");
  Serial.print(tempInCelsius[0]);          Serial.print(" "); Serial.print(tempInCelsius[1]);  Serial.print(" ");
  Serial.print(tempInCelsius[2]);          Serial.print(" "); Serial.print(tempInCelsius[3]);  Serial.print(" ");
  Serial.print(tempInCelsius[4]);          Serial.print(" "); Serial.print(tempInCelsius[5]);  Serial.print(" ");
  Serial.print(tempInCelsius[6]);          Serial.print(" "); Serial.print(tempInCelsius[7]);  Serial.println("      ");

  Serial.println("packed Temperature measurements:  ");
  Serial.print(packedTemp[0], HEX);     Serial.print(" "); Serial.print(packedTemp[1], HEX);  Serial.print(" ");
  Serial.print(packedTemp[2], HEX);     Serial.print(" "); Serial.print(packedTemp[3], HEX);  Serial.print(" ");
  Serial.print(packedTemp[4], HEX);     Serial.print(" "); Serial.print(packedTemp[5], HEX);  Serial.print(" ");
  Serial.print(packedTemp[6], HEX);     Serial.print(" "); Serial.print(packedTemp[7], HEX);  Serial.println("      "); Serial.println("      ");
  Serial.println("---");

  delay(50);

}
