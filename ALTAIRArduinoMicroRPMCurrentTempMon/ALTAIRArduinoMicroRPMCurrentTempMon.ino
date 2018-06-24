// Our Arduino Micro code for monitoring the RPM and temperature of, and the current flowing 
//     through, each of the 4 propulsion motors and electronic speed controllers (ESCs).  
// The Micro acts as an I2C slave (with slave address 08), and it reports 16 signed byte values
//     (4 RPMs, 4 currents, and 8 temperatures) over I2C, which each correspond to the most 
//     recent measurements, when info is requested by the Mega 2560 I2C master.

#include <Wire.h>

// Pulse timing for measuring the 4 RPMs.
const int     rpmTimerPin[4]            =     {  5,  7, 11, 13 };
const int     numRPMPulsesToAverage     =       20;
const long    numMicrosBeforeRPMTimeout =  1000000;
const long    numMicrosPerMinute        = 60000000;
const int     numMicrosDelayBtwReads    =       20;
const int     numPulsesPerRevolution    =        4;
char          packedRPM[4];

const int     currentSensorPin[4]       =     { A0, A1, A2, A3 };
int           currentSensorValue[4];
const int     numCurrentValsToAverage   =       20;
float         currentInAmps[4][numCurrentValsToAverage];  // average the past 20 values
char          packedCurrent[4];

const int     tempSensorPin[8]          =     { A4, A5, A6, A7,    A8, A9, A10, A11  };
int           tempSensorValue[8];
float         tempInCelsius[8];
char          packedTemp[8];


// get truncated (i.e. robust) mean of the pulse durations
float averageNumMicrosPerPulse(long *rpmPulseDuration) {
  long longestPulse = -999, secondLongestPulse = -999, shortestPulse = -999, secondShortestPulse = -999;
  long truncatedSumOfPulseDurations, sumOfPulseDurations = 0;
  for (int i = 0; i < numRPMPulsesToAverage; ++i) {
     sumOfPulseDurations += abs(rpmPulseDuration[i]);
     if (abs(rpmPulseDuration[i]) > longestPulse) {
       secondLongestPulse = longestPulse;
       longestPulse = abs(rpmPulseDuration[i]);
     }
     if (abs(rpmPulseDuration[i]) < shortestPulse || shortestPulse == -999) {
       secondShortestPulse = shortestPulse;
       shortestPulse = abs(rpmPulseDuration[i]);
     }
  }
  truncatedSumOfPulseDurations = sumOfPulseDurations - longestPulse - secondLongestPulse - shortestPulse - secondShortestPulse;
  return truncatedSumOfPulseDurations / (numRPMPulsesToAverage - 4.);
}

float getRPM(int pinNumber) {
  byte presentReading = digitalRead(pinNumber);
  unsigned long initialTime = micros();
  unsigned long previousTime = initialTime;
  unsigned long presentTime = initialTime;
  long rpmPulseDuration[numRPMPulsesToAverage];
  for (int i = 0; i < numRPMPulsesToAverage; ++i) {
    int j = 0;
// only consider the pulse to have ended if three digitalReads in a row differ from presentReading
    while (1) {
      if (digitalRead(pinNumber) != presentReading) {
        delayMicroseconds(numMicrosDelayBtwReads);
        if (digitalRead(pinNumber) != presentReading) {
          delayMicroseconds(numMicrosDelayBtwReads);
          if (digitalRead(pinNumber) != presentReading) {
            break;
          }
        }
      }
      ++j;
      if (j%10000 == 0) presentTime = micros();
      if (presentTime - initialTime > numMicrosBeforeRPMTimeout) return 0.0;
    }
    presentTime = micros();
    rpmPulseDuration[i] = presentTime - previousTime;
    if (presentReading == HIGH) rpmPulseDuration[i] = -rpmPulseDuration[i];
    previousTime = presentTime;
    presentReading = !presentReading;
  }
  return numMicrosPerMinute/(numPulsesPerRevolution*averageNumMicrosPerPulse(rpmPulseDuration)); 
}

char packRPM(float theRPM) {
  float theRPS = theRPM/60.;
  char packRPS;
  if (theRPS > -128. && theRPS < 127.) {
    packRPS = theRPS;
  } else if (theRPS >= 127.) {
    packRPS = 127;
  } else {
    packRPS = -128;
  }
  return packRPS;
}

char packTemp(float theTemp) {
  float scaledTemp = theTemp*2.;
  char thePackedTemp;
  if (scaledTemp > -128. && scaledTemp < 127.) {
    thePackedTemp = scaledTemp;
  } else if (scaledTemp >= 127.) {
    thePackedTemp = 127;
  } else {
    thePackedTemp = -128;
  }
  return thePackedTemp;
}

char packCurrent(float theCurrent) {
  float scaledCurrent = theCurrent*4.;
  char packCurr;
  if (scaledCurrent > -128. && scaledCurrent < 127.) {
    packCurr = scaledCurrent;
  } else if (scaledCurrent >= 127.) {
    packCurr = 127;
  } else {
    packCurr = -128;
  }
  return packCurr;
}

void setup() {
  Wire.begin(8);
  Wire.onRequest(sendInfo);
  
  Serial.begin(9600);
 
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < numCurrentValsToAverage; ++j) currentInAmps[i][j] = 0.;
  }
  // Initialize digital RPM timer pins as input.  (Note that the analog-read pins don't require this initialization.)
  for (int i = 0; i < 4; ++i) {
    pinMode(rpmTimerPin[i], INPUT);
  }
}

void loop() {
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


 
                           Serial.print(rpm[0]); Serial.print(" "); Serial.print(rpm[1]);            Serial.print(" ");
                           Serial.print(rpm[2]); Serial.print(" "); Serial.print(rpm[3]);            Serial.print("      "); 
//        Serial.print(currentSensorValue[0]); Serial.print(" = "); 
        Serial.print(currentRunningAverage[0]);  Serial.print(" "); 
//        Serial.print(currentSensorValue[1]); Serial.print(" = "); 
        Serial.print(currentRunningAverage[1]);  Serial.print(" ");
        Serial.print(currentRunningAverage[2]);  Serial.print(" ");
        Serial.print(currentRunningAverage[3]);  Serial.print("      ");
//        Serial.print(" Amps     Temps: "); Serial.print(tempSensorValue); Serial.print(" = "); 
        Serial.print(tempInCelsius[0]);          Serial.print(" "); Serial.print(tempInCelsius[1]);  Serial.print(" ");
        Serial.print(tempInCelsius[2]);          Serial.print(" "); Serial.print(tempInCelsius[3]);  Serial.print(" ");
        Serial.print(tempInCelsius[4]);          Serial.print(" "); Serial.print(tempInCelsius[5]);  Serial.print(" ");
        Serial.print(tempInCelsius[6]);          Serial.print(" "); Serial.print(tempInCelsius[7]);  Serial.println("      ");
//  for (int j = 0; j < numRPMPulsesToAverage; ++j) { Serial.print(rpmPulseDuration[0][j]); Serial.print(" "); } Serial.println(" ");

}

void sendInfo() {
  for (int i = 0; i < 4; ++i) Wire.write(packedRPM[i]);
  for (int i = 0; i < 4; ++i) Wire.write(packedCurrent[i]);
  for (int i = 0; i < 8; ++i) Wire.write(packedTemp[i]);  
}

