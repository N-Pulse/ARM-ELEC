#include "Utils.h"

// Print a 1-Wire ROM as 16 hex chars, e.g. "28FF3A1200000032"
void printROM(const uint8_t* rom) {
  for (uint8_t i = 0; i < 8; i++) {
    if (rom[i] < 16) Serial.print('0'); // zero-pad single hex digits
    Serial.print(rom[i], HEX);
  }
}

void printDeviceAddress(const DeviceAddress addr) {
  for (uint8_t i = 0; i < 8; i++) {
    if (addr[i] < 16) Serial.print('0');
    Serial.print(addr[i], HEX);
  }
}

// Safe float printer for CSV: avoids printing "-nan" or garbage
void printFloatCSV(float value, uint8_t decimals) {
  if (isnan(value)) Serial.print("nan");
  else              Serial.print(value, decimals);
}

bool aboveTempLimit(float value) {
  // isnan check first — NAN comparisons always return false, which would be a silent bug
  return !isnan(value) && value > Config::MAX_TEMP_C;
}

bool aboveHumidityLimit(float value) {
  return !isnan(value) && value > Config::MAX_RH_PERCENT;
}

// LIS3DH stores thresholds as integer LSBs, not raw g.
// LSB size depends on the chosen range (±2g, ±4g, ±8g, ±16g).
uint8_t thresholdGToReg(float g, int range_g) {
  float mg         = g * 1000.0f; // convert to milli-g
  float mg_per_lsb = 186.0f;      // default: ±16g
  switch (range_g) {
    case 2:  mg_per_lsb = 16.0f;  break;
    case 4:  mg_per_lsb = 32.0f;  break;
    case 8:  mg_per_lsb = 62.0f;  break;
    case 16: mg_per_lsb = 186.0f; break;
  }
  int reg = (int)round(mg / mg_per_lsb);
  if (reg < 0)   reg = 0;
  if (reg > 127) reg = 127; // register is 7-bit
  return (uint8_t)reg;
}

// Raw I2C write to a LIS3DH register (the Adafruit driver doesn't expose all registers)
void writeIMUReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(Config::ADDR_LIS3DH);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readIMUReg(uint8_t reg) {
  Wire.beginTransmission(Config::ADDR_LIS3DH);
  Wire.write(reg);
  Wire.endTransmission(false); // repeated start — keep bus busy between write and read
  Wire.requestFrom((int)Config::ADDR_LIS3DH, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

// Pull PIN_POWER_CUT LOW to open the load switch, then log the cause
void requestPowerCut(const char* reason) {
  safety.power_cut_flag = true;
  digitalWrite(Config::PIN_POWER_CUT, LOW);
  Serial.print("[ALERT] : power cut requested due to ");
  Serial.println(reason);
}

// Lookup sensor temperature limit by ROM address
// Returns true if address is found in the mapping; out_limit is set to the limit
// Returns false if not found; out_limit is set to Config::MAX_TEMP_C (fallback)
bool getSensorTempLimit(const uint8_t* address, float& out_limit) {
  // Check bus 0
  for (uint8_t i = 0; i < Config::bus0_sensors_count; i++) {
    if (memcmp(address, Config::bus0_sensors[i].address, 8) == 0) {
      out_limit = Config::bus0_sensors[i].temp_limit_c;
      return true;
    }
  }
  // Check bus 1
  for (uint8_t i = 0; i < Config::bus1_sensors_count; i++) {
    if (memcmp(address, Config::bus1_sensors[i].address, 8) == 0) {
      out_limit = Config::bus1_sensors[i].temp_limit_c;
      return true;
    }
  }
  // Check direct line
  for (uint8_t i = 0; i < Config::direct_sensors_count; i++) {
    if (memcmp(address, Config::direct_sensors[i].address, 8) == 0) {
      out_limit = Config::direct_sensors[i].temp_limit_c;
      return true;
    }
  }
  // Not found — use default
  out_limit = Config::MAX_TEMP_C;
  return false;
}

// Check if an address matches a specific sensor purpose (e.g., BATTERY, SOCKET)
bool isSensorByAddress(const uint8_t* address, Config::SensorPurpose purpose) {
  // Check bus 0
  for (uint8_t i = 0; i < Config::bus0_sensors_count; i++) {
    if (memcmp(address, Config::bus0_sensors[i].address, 8) == 0 &&
        Config::bus0_sensors[i].purpose == purpose) {
      return true;
    }
  }
  // Check bus 1
  for (uint8_t i = 0; i < Config::bus1_sensors_count; i++) {
    if (memcmp(address, Config::bus1_sensors[i].address, 8) == 0 &&
        Config::bus1_sensors[i].purpose == purpose) {
      return true;
    }
  }
  // Check direct line
  for (uint8_t i = 0; i < Config::direct_sensors_count; i++) {
    if (memcmp(address, Config::direct_sensors[i].address, 8) == 0 &&
        Config::direct_sensors[i].purpose == purpose) {
      return true;
    }
  }
  return false;
}