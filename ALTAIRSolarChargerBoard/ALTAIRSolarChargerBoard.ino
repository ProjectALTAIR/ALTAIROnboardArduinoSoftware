//Code to opperate the ALTAIR custom solar charging board

#include <Wire.h>
#include "LTC4162-LADM.h"
#include "LTC4162-LADM_pec.h"
#include "LTC4162-LADM_formats.h"

// Safety thresholds
#define MAX_BATTERY_VOLTAGE 11.7 // Maximum voltage for a 3-cell Li-Po
#define MAX_CHARGE_CURRENT 2.0   // Maximum charge current (2A)
#define MAX_TEMPERATURE 100.0     // Maximum temperature in Celsius
#define MAX_CHARGE_TIME 240      // Maximum charge time in minutes (4 hours, default)
#define SAFE_CHARGE_CURRENT 0.5  // Safe charge current to use in case of overcurrent

int read_register(uint8_t address, uint8_t command_code, uint16_t *data);
int write_register(uint8_t address, uint8_t command_code, uint16_t data);

float read_voltage();
float read_current();
float read_temperature();
float read_input_voltage(); 
uint16_t read_charge_status();
uint16_t read_charger_state();
float read_battery_percentage(float voltage);
void set_safe_charge_current();
void configure_charging();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Starting I2C...");

  // Check I2C communication with the chip
  while (!check_i2c_communication()) {
    Serial.println("Failed to communicate with the chip. Retrying in 2 seconds...");
    delay(2000);
  }

  configure_charging();
}

void loop() {
  Serial.println("Loop running..."); // debugging print

  // Read and print the system status
  read_system_status();

  // Voltage
  float voltage = read_voltage();
  if (voltage >= 0) {
    Serial.print("Battery Voltage: ");
    Serial.print(voltage, 3);
    Serial.println(" V");
    if (voltage > MAX_BATTERY_VOLTAGE) {
      Serial.println("Warning: Battery voltage exceeds the maximum limit!");
    }
  }

  // Input Voltage (Solar Panel Voltage)
  float input_voltage = read_input_voltage();
  if (input_voltage >= 0) {
    Serial.print("Solar Panel Voltage: ");
    Serial.print(input_voltage, 3);
    Serial.println(" V");
  }

  // Battery current
  float current = read_current();
  if (current >= 0) {
    Serial.print("Battery Current: ");
    Serial.print(current, 3);
    Serial.println(" A");
    if (current > MAX_CHARGE_CURRENT) {
      Serial.println("Warning: Battery current exceeds the maximum limit! Setting safe charge current.");
      set_safe_charge_current();
    }
  }

  // Input current
  float input_current = read_input_current();
  if (input_current >= 0) {
    Serial.print("Input Current: ");
    Serial.print(input_current, 2);
    Serial.println(" A");
  }

  // Input Current Limit
  float input_current_limit = read_input_current_limit();
  if (input_current_limit >= 0) {
    Serial.print("Input Current Limit: ");
    Serial.print(input_current_limit, 2);
    Serial.println(" A");
  }

  // Charge Current Setting
  float charge_current_setting = read_charge_current_setting();
  if (charge_current_setting >= 0) {
    Serial.print("Charge Current Setting: ");
    Serial.print(charge_current_setting, 2);
    Serial.println(" A");
  }

  // Temperature
  float temperature = read_temperature();
  if (temperature >= 0) {
    Serial.print("Die Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");
    if (temperature > MAX_TEMPERATURE) {
      Serial.println("Warning: Die temperature exceeds the maximum limit!");
    }
  }

  // Charge status
  uint16_t status = read_charge_status();
  if (status != 0xFFFF) {
    Serial.print("Charge Status: ");
    Serial.println(get_charge_status_text(status));
  }

  // Charger state
  uint16_t charger_state = read_charger_state();
  if (charger_state != 0xFFFF) {
    Serial.print("Charger State: ");
    Serial.println(get_charger_state_text(charger_state));
  }

  // Battery percentage
  float battery_percentage = read_battery_percentage(voltage);
  Serial.print("Battery Percentage: ");
  Serial.print(battery_percentage, 2);
  Serial.println(" %");

  delay(2000); // Time in ms before updating, (using 2 sec for now)
}

// Check I2C communication
bool check_i2c_communication() {
  Wire.beginTransmission(LTC4162_ADDR_68);
  return (Wire.endTransmission() == 0);
}

// Set a safe charge current
void set_safe_charge_current() {
  uint16_t safeChargeCurrent = LTC4162_ICHARGE_R2U(SAFE_CHARGE_CURRENT); // Use the macro for safe current
  write_register(LTC4162_ADDR_68, 0x1A, safeChargeCurrent); // 0x1A for CHARGE_CURRENT_SETTING_REG
}

// All the register readings return 0 for success, otherwise for failure
float read_voltage() {
  uint16_t voltage;
  if (read_register(LTC4162_ADDR_68, 0x3A, &voltage) != 0) { // 0x3A for VBAT_REG
    Serial.println("Failed to read voltage");
    return -1;
  }
  return LTC4162_VBAT_FORMAT_I2R(voltage)*3; // Use the macro to convert to real voltage
}

float read_current() {
  uint16_t current;
  if (read_register(LTC4162_ADDR_68, 0x3D, &current) != 0) { // 0x3D for IBAT_REG
    Serial.println("Failed to read current");
    return -1;
  }
  return LTC4162_IBAT_FORMAT_I2R(current); // Use the macro to convert to real current
}

float read_input_current() {
  uint16_t input_current;
  if (read_register(LTC4162_ADDR_68, 0x3E, &input_current) != 0) { // 0x3E for IIN_REG
    Serial.println("Failed to read input current");
    return -1;
  }
  return LTC4162_IIN_FORMAT_I2R(input_current); // Use the macro to convert to real current
}

float read_input_current_limit() {
  uint16_t iin_limit_dac;
  if (read_register(LTC4162_ADDR_68, 0x46, &iin_limit_dac) != 0) { // 0x46 for IIN_LIMIT_DAC_REG
    Serial.println("Failed to read input current limit");
    return -1;
  }
  return LTC4162_IINLIM_U2R(iin_limit_dac); // Use the macro to convert DAC to real current

}
float read_charge_current_setting() {
  uint16_t charge_current_setting;
  if (read_register(LTC4162_ADDR_68, 0x1A, &charge_current_setting) != 0) { // 0x1A for CHARGE_CURRENT_SETTING_REG
    Serial.println("Failed to read charge current setting");
    return -1;
  }
  return LTC4162_ICHARGE_U2R(charge_current_setting); // Use the macro to convert DAC to real current

}
float read_temperature() {
  uint16_t temperature;
  if (read_register(LTC4162_ADDR_68, 0x3F, &temperature) != 0) { // 0x3F for DIE_TEMP_REG
    Serial.println("Failed to read temperature");
    return -1;
  }
  return LTC4162_DIE_TEMP_FORMAT_I2R(temperature); // Use the macro to convert to real temperature
}

// Function to read the input voltage (solar panel voltage)
float read_input_voltage() {
  uint16_t input_voltage;
  if (read_register(LTC4162_ADDR_68, 0x3B, &input_voltage) != 0) { // 0x3B for VIN_REG
    Serial.println("Failed to read input voltage");
    return -1;
  }
  return LTC4162_VIN_FORMAT_I2R(input_voltage); // Use the macro to convert to real voltage
}

uint16_t read_charge_status() {
  uint16_t status;
  if (read_register(LTC4162_ADDR_68, 0x35, &status) != 0) { // 0x35 for CHARGE_STATUS_REG
    Serial.println("Failed to read charge status");
    return 0xFFFF; // Error code for failed read
  }
  return status;
}

uint16_t read_charger_state() {
  uint16_t state;
  if (read_register(LTC4162_ADDR_68, 0x34, &state) != 0) { // 0x34 for CHARGER_STATE_REG
    Serial.println("Failed to read charger state");
    return 0xFFFF; // Error code for failed read
  }
  return state;
}

float read_battery_percentage(float voltage) {
  // Discharge from 3.0V to 3.9 per cell
  float percentage = ((voltage/3) - 3.0) / (3.9 - 3.0) * 100;
  return constrain(percentage, 0, 100); // limit the percentage so it's always between 0 and 100
}

String get_charge_status_text(uint16_t status) {
  switch (status) {
    case 0x1: return "Constant voltage";
    case 0x2: return "Constant current";
    case 0x4: return "Input current limit active";
    case 0x8: return "Input voltage control loop active";
    case 0x10: return "Thermal regulation active";
    case 0x20: return "Input current limit regulator active";
    default: return "Charger off";
  }
}

String get_charger_state_text(uint16_t state) {
  if (state & 0x1000) return "Battery detection failed";
  if (state & 0x0800) return "Battery detection phase";
  if (state & 0x0100) return "Charger suspended";
  if (state & 0x0080) return "Precharge phase";
  if (state & 0x0040) return "Constant current/constant voltage phase";
  if (state & 0x0020) return "NTC pause";
  if (state & 0x0010) return "Timer termination";
  if (state & 0x0008) return "C/x termination";
  if (state & 0x0004) return "Max charge time fault";
  if (state & 0x0002) return "Battery missing fault";
  if (state & 0x0001) return "Battery short fault";
  return "Unknown state";
}

int read_register(uint8_t address, uint8_t command_code, uint16_t *data) {
  Wire.beginTransmission(address);
  Wire.write(command_code);
  if (Wire.endTransmission() != 0) {
    Serial.println("I2C read transmission failed");
    return -1; 
  }

  Wire.requestFrom(address, (uint8_t)2); // Request 2 bytes from the device
  if (Wire.available() < 2) {
    Serial.println("I2C read insufficient data");
    return -1; 
  }

  *data = Wire.read();
  *data |= (Wire.read() << 8);

  return 0;
}

int write_register(uint8_t address, uint8_t command_code, uint16_t data) {
  Wire.beginTransmission(address);
  Wire.write(command_code);
  Wire.write(data & 0xFF);
  Wire.write((data >> 8) & 0xFF);
  if (Wire.endTransmission() != 0) {
    Serial.println("I2C write transmission failed");
    return -1; // Transmission failed
  }

  return 0;
}

void configure_charging() {
  Serial.println("Configuring charging parameters...");

  // Enable MPPT (It's on by default on this flavor of chip... but just in case.)
  uint16_t chargerConfig;
  if (read_register(LTC4162_ADDR_68, 0x14, &chargerConfig) == 0) { // 0x14 for CONFIG_BITS_REG
    chargerConfig |= (1 << 1); // Set the mppt_en bit (Bit 1)
    write_register(LTC4162_ADDR_68, 0x14, chargerConfig);
  }

  // Input current limit : 1.2A
  uint16_t iin_limit_target = 23; // Calculated value for 1.2A limit
  write_register(LTC4162_ADDR_68, 0x15, iin_limit_target); // 0x15 for IIN_LIMIT_TARGET_REG

  // Set maximum charge time
  uint16_t maxChargeTime = MAX_CHARGE_TIME; // Set to 4 hours for now
  write_register(LTC4162_ADDR_68, 0x1E, maxChargeTime); // 0x1E for MAX_CHARGE_TIME_REG
}
//Uncomment the following to enable JEITA charging guidelines
/* 
  // JEITA temperature thresholds
  uint16_t jeita_t1 = (uint16_t)((0.0 + 264.4) / 0.0215); // 0°C
  write_register(LTC4162_ADDR_68, 0x1F, jeita_t1); // 0x1F for JEITA_T1_REG

  uint16_t jeita_t2 = (uint16_t)((10.0 + 264.4) / 0.0215); // 10°C
  write_register(LTC4162_ADDR_68, 0x20, jeita_t2); // 0x20 for JEITA_T2_REG

  uint16_t jeita_t3 = (uint16_t)((25.0 + 264.4) / 0.0215); // 25°C
  write_register(LTC4162_ADDR_68, 0x21, jeita_t3); // 0x21 for JEITA_T3_REG

  uint16_t jeita_t4 = (uint16_t)((40.0 + 264.4) / 0.0215); // 40°C
  write_register(LTC4162_ADDR_68, 0x22, jeita_t4); // 0x22 for JEITA_T4_REG

  uint16_t jeita_t5 = (uint16_t)((45.0 + 264.4) / 0.0215); // 45°C
  write_register(LTC4162_ADDR_68, 0x23, jeita_t5); // 0x23 for JEITA_T5_REG

  uint16_t jeita_t6 = (uint16_t)((60.0 + 264.4) / 0.0215); // 60°C
  write_register(LTC4162_ADDR_68, 0x24, jeita_t6); // 0x24 for JEITA_T6_REG

  // Enable JEITA
  if (read_register(LTC4162_ADDR_68, 0x29, &chargerConfig) == 0) { // 0x29 for CHARGER_CONFIG_BITS_REG
    chargerConfig |= (1 << 0); // Set the en_jeita bit (Bit 0) to 1
    write_register(LTC4162_ADDR_68, 0x29, chargerConfig);
  }
}
*/

void read_system_status() {
  uint16_t system_status;

  // Read the SYSTEM_STATUS_REG
  if (read_register(LTC4162_ADDR_68, 0x39, &system_status) != 0) {
    Serial.println("Failed to read SYSTEM_STATUS_REG");
    return;
  }

  // Print the relevant system status information
  Serial.print("System Status: ");
  if (system_status & (1 << 8)) Serial.print("Charger Active, ");
  if (system_status & (1 << 7)) Serial.print("Cell Count Error, ");
  if (system_status & (1 << 5)) Serial.print("No RT Resistor, ");
  if (system_status & (1 << 4)) Serial.print("Thermal Shutdown, ");
  if (system_status & (1 << 3)) Serial.print("VIN Overvoltage Lockout, ");
  if (system_status & (1 << 2)) Serial.print("VIN > VBAT, ");
  if (system_status & (1 << 1)) Serial.print("VIN > 4.2V, ");
  if (system_status & (1 << 0)) Serial.print("INTVCC > 2.8V");

  Serial.println();
}

