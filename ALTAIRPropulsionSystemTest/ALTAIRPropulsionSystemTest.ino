//#include "ALTAIR_RPMSensor.h"
#include "C:\Users\Chris\source\repos\ProjectALTAIR\ALTAIROnboardArduinoSoftware\libraries\ALTAIR_Devices\ALTAIR_RPMSensor.h"
#include "C:\Users\Chris\source\repos\ProjectALTAIR\ALTAIROnboardArduinoSoftware\libraries\ALTAIR_Motors\ALTAIR_PWM_setup.h"

uint8_t RPM_sensor_pins[4] = {A0, A1, A2, A3};
const int RPM_measurement_time = 500;

int i = 2;

int PWM_control_value = 0;
int PWM_value = 0;

int start_millis = 0;
int current_millis = 0;

ALTAIR_RPMSensor test_RPM_sensor(i);

ALTAIR_RPMSensor arrayof_RPMSensors[4] = {
		ALTAIR_RPMSensor(RPM_sensor_pins[0]),
		ALTAIR_RPMSensor(RPM_sensor_pins[1]),
		ALTAIR_RPMSensor(RPM_sensor_pins[2]),
		ALTAIR_RPMSensor(RPM_sensor_pins[3]),
};

void setup() {
	initialize_PWM_registers();
	
	

	
	Serial.begin(9600);
}

void loop() {
	Serial.println("Which Pin would you like to control? ");

	while (Serial.available() == 0) {}
  
	int pinChoice = Serial.parseInt();
	
	PWM_motor_control(pinChoice);

	current_millis = millis();
	start_millis = current_millis;
	while(current_millis-start_millis <= RPM_measurement_time){
		for (int i = 0; i < 4; i++) {
			arrayof_RPMSensors[i].store_analog_RPM();
			arrayof_RPMSensors[i].risingEdge_detection();
			arrayof_RPMSensors[i].fallingEdge_detection();
		}
		current_millis = millis();
	}
		
	Serial.print("Current RPM values: ");
	for (int i = 0; i < 4; i++) {
		arrayof_RPMSensors[i].calculate_RPM(RPM_measurement_time);
		Serial.print(arrayof_RPMSensors[i].rpm); Serial.print("  ");
	}
	
	Serial.println(" ");
	Serial.println("----");


}
