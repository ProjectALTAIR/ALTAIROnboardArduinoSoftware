/*
  ALTAIRPropulsionSystemTest.ino
  Prepared by Christopher Vogt (christophervogt@uvic.ca)

  Code for subsystem level test of the propulsion system.
  Motor control via serial input.
  Sensor data via I2C through ALTAIR_ArduinoMicro class
*/


//#include <ALTAIR_PWM_setup.h>

#include "ALTAIR_PropulsionSystem.h"
#include <ALTAIR_ArduinoMicro.h>

int start_millis = 0;
int current_millis = 0;

long RPM_measurement_interval = 100;

controlMode mode;
int _controlMode = 0;   //Which Motor                   (individual, pairs...)
int _controlType = 0;   //Which way to set new value  	(set own Value, increment...)
int motorChoice = 0;

float newPWMControlValue;

int _serialBuffer;


ALTAIR_ArduinoMicro ArduinoMicro;
ALTAIR_PropulsionSystem propSystem;


void setPWMControlValues(controlMode mode) {
    Serial.println("Choose how to set new Control Value:");
    Serial.println("[1] Set new Control Value");
    Serial.println("[2] Increment Control Value by +1");
    Serial.println("[3] Decrement Control Value by -1");
    Serial.println("[4] Half Increment Control Value by +0.5");
    Serial.println("[5] Half Decrement Control Value by -0.5");
    
    while (Serial.available() == 0){}
    _controlType = Serial.parseInt();

    switch (_controlType) {
      case 1:   // Set own Control Value
        Serial.println("Select PWM control value as x.x between 1.0 and 10.0");
        while (Serial.available() == 0) {}
        newPWMControlValue = Serial.parseFloat();
        if(propSystem.setPWMControlTo(mode, newPWMControlValue)){
          Serial.println("PWM Control Values are reset!");
          for(int i=0;i<4;i++){Serial.println(propSystem.motors()[i].PWMControlValue());}
        }
        break;
      case 2:   // Increment Control Value by +1
        if (propSystem.incrementPWMControl(mode)) {
          Serial.println("PWM Control Values are reset!");
        }
        break;
      case 3:   // Decrement Control Value by -1
        if (propSystem.decrementPWMControl(mode)) {
          Serial.println("PWM Control Values are reset!");
        }
        break;
      case 4:   // Half Increment Control Value by +0.5
        if (propSystem.halfIncrementPWMControl(mode)) {
          Serial.println("PWM Control Values are reset!");
        }
        break;
      case 5:   // Half Decrement Control Value by -0.5
        if (propSystem.halfDecrementPWMControl(mode)) {
          Serial.println("PWM Control Values are reset!");
        }
        break;
      default:
        Serial.println("Please choose valid value!");
        break;

    }

}





void setup() {
  Serial.begin(9600);
  delay(5000);
 
	Serial.println("Begin PWM initialization");
	//initialize_PWM_registers();
	
  propSystem.initializePropControlRegisters();
  propSystem.initializePWMOutputRegisters();
  if(propSystem.isInitialized() == true) {
    Serial.println("PWM Motor Control initialized!");
  }
  Wire.begin();
  for(int i=0;i<4;i++){Serial.println(propSystem.motors()[i].PWMControlValue());}
  Serial.println("Press [1] to start motor control sequence.");	
}

void loop() {
  
  current_millis = millis();
  start_millis = current_millis;

  
  if(Serial.available() > 0) {
    Serial.println("Something is happening");

  _serialBuffer = Serial.parseInt();


  Serial.println("Select motor control mode:");
  Serial.println("[1] Individual control");
  Serial.println("[2] Outer motor pair");
  Serial.println("[3] Inner motor pair");
  Serial.println("[4] Combined control");
  Serial.println("[5] Shut down all motors");


  while (Serial.available() == 0) {  }
  _controlMode = Serial.parseInt();
  Serial.print("Control Mode: "); Serial.println(_controlMode);
  switch (_controlMode) {
    
    case 1: // Individual Motor Control
      mode = individual;

      Serial.println("Select motor: 1...4 port to stbd");
      while (Serial.available() == 0) {}
      motorChoice = Serial.parseInt();
      
      if(motorChoice <= 4 && motorChoice >= 1){
        propSystem.selectIndividualMotor(&propSystem.motors()[motorChoice-1]);
      }else{
        Serial.println("Please select valid motor");
        break;
      }

  	  setPWMControlValues(mode);

      break;
      
    case 2: // Outer Motor Pair Motor Control
      mode = outerPair;

      setPWMControlValues(mode);
      break;
    case 3: // Inner Motor Pair Motor Control
      mode = innerPair;

      setPWMControlValues(mode);
      break;
    case 4: // Combined Motor Control
      mode = combined;

      setPWMControlValues(mode);
      break;
    case 5:
      if(propSystem.shutDownAllProps()) {
        Serial.println("All motors turned off!");
      }
      break;
    default:
      Serial.println("Please choose a valid motor control mode!");
      break;
  }
  //delay(100);
  
  } else {
    Serial.println("Nothing is happening");
    Serial.println("RPM Values:");
    ArduinoMicro.getDataAfterInterval(RPM_measurement_interval);

    Serial.println("PWM Control Values:");
    for (int i = 0; i < 4; i++) {
      Serial.print(propSystem.motors()[i].PWMValue()); Serial.print("  ");
      Serial.println(propSystem.motors()[i].PWMControlValue());
    }

	  Serial.println("----");

    Serial.println("Press [1] to start motor control sequence.");
    delay(1000);
  }

}
