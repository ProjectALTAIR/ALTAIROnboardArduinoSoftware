/*
  ALTAIRUSBConnectionTest.ino
  Prepared by Christopher Vogt (christophervogt@uvic.ca)

  Quick test to check whether the USB is connected in addition to the
  external power supply.
  This code checks a register in the USB System of the Grand Central's
  SAMD51 chip.
  In this state, only a variable in this code is set, no information transfer
  is implemented. But this can be easily done by sending this variable (0 or 1)
  to another device from which the information about connected/ not connected
  is observed.
*/

/*
  The information whether a USB connection is there or not can be retrieved from
  the registers in the USB section of the SAMD51 chip.
  More specifically, it was found that the FNUM (Frame Number) register is most
  useful (38.8.4 in the SAMD51 datasheet).
  The FNUM is no direct status register, but the value written in FNUM is
  always changing when a USB is connected, even if there is no current data transfer.
  By checking if the value has changed (refresh rate is ~1ms), the information
  about a connection can be retrieved.
*/

uint8_t fnum1;
uint8_t fnum2;
bool    _isUSBconnected;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // For indicating the connection status on the Grand Central
  Serial1.begin(9600);              // For transfering the information via Serial
}

void loop() {
  // Check connection status
  _isUSBconnected = isUSBconnected();  
  
  // Take action
    LEDwhenConnected(_isUSBconnected);
    
    Serial1.write((uint8_t) _isUSBconnected);
    Serial1.flush();

  delay(500);

}

bool isUSBconnected()    // returns 1 (TRUE) if USB is connected, 0 (FALSE) if not
{	
  // Count frame numbers
	fnum1 = USB->DEVICE.FNUM.bit.FNUM;
	delay(5);
  fnum2 = USB->DEVICE.FNUM.bit.FNUM;
  // Compare the two frame numbers
	return fnum1 != fnum2;
}

void LEDwhenConnected(bool _isUSBconnected)
{
  if(_isUSBconnected == 1){
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}