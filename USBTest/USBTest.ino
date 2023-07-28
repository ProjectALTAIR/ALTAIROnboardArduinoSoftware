

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if(isUSBconnected() == 1){
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    //Serial.println("is connected");
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    //Serial.println("not connected");
  }

  delay(500);                      // wait for a second
}

bool isUSBconnected()
{
	/*
  // Count frame numbers
	uint8_t f = USB->DEVICE.FNUM.bit.FNUM;
	delay(3);
	return f != USB->DEVICE.FNUM.bit.FNUM;
  */
  return USB->DEVICE.CTRLA.bit.ENABLE;
}