#include "LTC4162-LADM.h"
#include <Wire.h>

// Define I2C address for LTC4162-L
#define LTC4162_I2C_ADDRESS 0x68

// Define SMBALERT pin
#define SMBALERT_PIN 2

// Function declarations
int my_read_register(uint8_t address, uint8_t command_code, uint16_t *data, struct port_configuration *pc);
int my_write_register(uint8_t address, uint8_t command_code, uint16_t data, struct port_configuration *pc);

LTC4162_chip_cfg_t LTC4162 =
{
  LTC4162_I2C_ADDRESS, // .address (7-bit)
  my_read_register,    // .read_register
  my_write_register,   // .write_register
  NULL                 // .port_configuration not used
};

uint16_t data;

void setup() {
  // Start serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for the Serial Monitor to open
  }
  Serial.println("Press any key to demo reading and writing to the LTC4162-LADM.");

  // Initialize I2C communication
  Wire.begin(); // Use default pins for the Grand Central M4

  // Initialize SMBALERT pin
  pinMode(SMBALERT_PIN, INPUT_PULLUP);

  // Check for the presence of the LTC4162-LADM chip in a loop
  while (true) {
    Wire.beginTransmission(LTC4162_I2C_ADDRESS);
    if (Wire.endTransmission() == 0) {
      Serial.println("LTC4162-LADM initialized successfully");
      break;
    } else {
      Serial.println("Failed to find LTC4162-LADM chip. Retrying in 2 seconds...");
      delay(2000); // Wait for 2 seconds before retrying
    }
  }
}

void loop() {
  if (Serial.available() < 1) {
    return; // Don't do anything unless we've gotten bytes over the serial port.
  } else {
    while (Serial.available() > 0) {
      Serial.read(); // Throw away all the incoming characters
    }

    // Test Read/Write Operations
    test_read_write_operations();
  }

  // Check SMBALERT
  if (digitalRead(SMBALERT_PIN) == LOW) {
    Serial.println("SMBALERT triggered!");
    // Handle SMBALERT condition here
    handleSMBALERT();
  }
}

// Implement the read and write functions using Wire library for I2C communication
int my_read_register(uint8_t address, uint8_t command_code, uint16_t *data, struct port_configuration *pc) {
  Wire.beginTransmission(address);
  Wire.write(command_code);
  if (Wire.endTransmission() != 0) {
    Serial.println("Transmission failed during read setup");
    return -1; // Transmission failed
  }

  Wire.requestFrom(address, (uint8_t)2); // Request 2 bytes from the device
  if (Wire.available() < 2) {
    Serial.println("Not enough data received");
    return -1; // Not enough data received
  }

  *data = Wire.read();
  *data |= (Wire.read() << 8);

  return 0;
}

int my_write_register(uint8_t address, uint8_t command_code, uint16_t data, struct port_configuration *pc) {
  Wire.beginTransmission(address);
  Wire.write(command_code);
  Wire.write(data & 0xFF);
  Wire.write((data >> 8) & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println("Transmission failed during write");
    return -1; // Transmission failed
  }
  return 0; // Transmission succeeded
}

// Handle SMBALERT condition
void handleSMBALERT() {
  uint16_t alertStatus;
  if (LTC4162_read_register(&LTC4162, 0x36, &alertStatus) == 0) { // Reading LIMIT_ALERTS_REG
    Serial.print("Alert status: ");
    Serial.println(alertStatus, HEX);

    // Clear the alert by writing to the appropriate register if needed
    LTC4162_write_register(&LTC4162, 0x36, 0xFFFF); // Clear alerts by writing 0xFFFF to LIMIT_ALERTS_REG
  } else {
    Serial.println("Failed to read alert register");
  }
}

// Test Read/Write Operations
void test_read_write_operations() {
  uint16_t readValue;

  // Write to vbat_lo_alert_limit register (0x01)
  Serial.println("Writing 0x1234 to vbat_lo_alert_limit register (0x01)");
  if (LTC4162_write_register(&LTC4162, 0x01, 0x1234) == 0) {
    Serial.println("Write successful");
  } else {
    Serial.println("Failed to write to vbat_lo_alert_limit register");
  }

  // Read from vbat_lo_alert_limit register (0x01)
  Serial.println("Reading from vbat_lo_alert_limit register (0x01)");
  if (LTC4162_read_register(&LTC4162, 0x01, &readValue) == 0) {
    Serial.print("Read vbat_lo_alert_limit register (0x01): ");
    Serial.println(readValue, HEX);
  } else {
    Serial.println("Failed to read from vbat_lo_alert_limit register");
  }

  // Write to vin_lo_alert_limit register (0x03)
  Serial.println("Writing 0x5678 to vin_lo_alert_limit register (0x03)");
  if (LTC4162_write_register(&LTC4162, 0x03, 0x5678) == 0) {
    Serial.println("Write successful");
  } else {
    Serial.println("Failed to write to vin_lo_alert_limit register");
  }

  // Read from vin_lo_alert_limit register (0x03)
  Serial.println("Reading from vin_lo_alert_limit register (0x03)");
  if (LTC4162_read_register(&LTC4162, 0x03, &readValue) == 0) {
    Serial.print("Read vin_lo_alert_limit register (0x03): ");
    Serial.println(readValue, HEX);
  } else {
    Serial.println("Failed to read from vin_lo_alert_limit register");
  }
}
