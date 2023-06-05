/*
  ALTAIRPropulsionSystemTest.ino
  Prepared by Christopher Vogt (christophervogt@uvic.ca)

  Code for subsystem level test of the propulsion system.
  Motor control via serial input.
  Sensor data via I2C through ALTAIR_ArduinoMicro class
*/


#include <ALTAIR_PWM_setup.h>
#include <ALTAIR_ArduinoMicro.h>

int start_millis = 0;
int current_millis = 0;
int i = 0;
int PWM_control_value = 0;
int PWM_value = 0;
long RPM_measurement_interval = 2000;

ALTAIR_ArduinoMicro ArduinoMicro;


void setup() {
  Serial.begin(9600);
  delay(5000);
 
	Serial.println("Begin PWM initialization");
	initialize_PWM_registers();
	
  Wire.begin();	
}

void loop() {
  
  current_millis = millis();
  start_millis = current_millis;

	Serial.println("Which Pin would you like to control? ('1' for control of all 4)");
	while (Serial.available() == 0) {
    while(current_millis < start_millis+2000){
      current_millis = millis();
    }
    break;
  }  
	int pinChoice = Serial.parseInt();	
	PWM_motor_control(pinChoice);

  ArduinoMicro.getDataAfterInterval(RPM_measurement_interval);

	Serial.println("----");

}
