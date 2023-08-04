/*
  ALTAIRUSBConnectionTestReceiver.ino
  Prepared by Christopher Vogt (christophervogt@uvic.ca)

  Readout counterpart to the ALTAIRUSBConnectionTest.ino.
  Retrieving 1 or 0 for USB connected/disconnected
*/

uint8_t read;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

}

void loop() {
  while(!Serial1.available()){};
  read = Serial1.read();
  if(read == 1){
    Serial.println("USB connected");
  } else if(read == 0){
    Serial.println("USB disconnected");
  } else {
    Serial.println("Something went wrong");
  }
}
