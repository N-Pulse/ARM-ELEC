#include "Setup.h"

// ISRs: trigger power cut immediately for minimal response time.
// ARDUINO_ISR_ATTR places the function in IRAM (fast memory) on ESP32.
// digitalWrite is safe to call from ISR; sets the flag for main loop verification.
void ARDUINO_ISR_ATTR onOvercurrentIRQ() {
  overcurrent_irq = true;
  if (Config::ENABLE_POWER_CUT_ON_OVERCURRENT) { 
    safety.power_cut_flag = true;
    digitalWrite(Config::PIN_POWER_CUT, LOW);
  }
}
void ARDUINO_ISR_ATTR onImpactIRQ() {
  impact_irq = true;
  if (Config::ENABLE_POWER_CUT_ON_CONFIRMED_SHOCK) {
    safety.power_cut_flag = true;
    digitalWrite(Config::PIN_POWER_CUT, LOW);
  }
}

bool setupHumiditySensors() {
  bool ok = true;

  // SHT41 on bus 0 (Wire = default I2C)
  if (!sht41_bus0.begin(Wire)) {
    Serial.println("[SETUP] SHT41 bus0 not found"); ok = false;
  } else { Serial.println("[SETUP] SHT41 bus0 OK"); }

  // SHT41 on bus 1 (Wire1 = pins 9/10)
  if (!sht41_bus1.begin(Wire1)) {
    Serial.println("[SETUP] SHT41 bus1 not found"); ok = false;
  } else { Serial.println("[SETUP] SHT41 bus1 OK"); }

  // Si7021 on bus 1 (constructed with &Wire1 in DataTypes.cpp)
  if (!si7021_bus1.begin()) {
    Serial.println("[SETUP] Si7021 bus1 not found"); ok = false;
  } else { Serial.println("[SETUP] Si7021 bus1 OK"); }

  return ok;
}

bool setupDS2484Line(DS2484Line& line) {
  Serial.print("[SETUP] Setting up "); Serial.println(line.name);

  // DS2484 is an I2C-to-1-Wire bridge chip
  if (!line.bridge->begin(line.wire, DS248X_ADDRESS)) {
    Serial.println("[SETUP]  DS2484 not found");
    line.present = false; return false;
  }

  // Check the 1-Wire bus is actually connected and not shorted
  if (!line.bridge->OneWireReset()) {
    Serial.println("[SETUP]  1-Wire reset failed");
    if (line.bridge->shortDetected())          Serial.println("[SETUP]   -> Short detected");
    if (!line.bridge->presencePulseDetected()) Serial.println("[SETUP]   -> No presence pulse");
    line.present = false; return false;
  }

  // Walk the bus and collect ROMs of any DS18B20s found
  line.bridge->OneWireSearchReset();
  line.found = 0;
  uint8_t rom[8];
  while (line.found < Config::MAX_SENSORS_PER_LINE && line.bridge->OneWireSearch(rom)) {
    if (rom[0] == DS18B20_FAMILY_CODE) { // 0x28 = DS18B20, skip other devices
      memcpy(line.roms[line.found], rom, 8);
      Serial.print("[SETUP]  DS18B20 #"); Serial.print(line.found + 1);
      Serial.print(" ROM: "); printROM(rom); Serial.println();
      line.found++;
    }
  }

  line.present = true;
  Serial.print("[SETUP]  Total sensors on "); Serial.print(line.name);
  Serial.print(" = "); Serial.println(line.found);
  return true;
}

bool setupDirectOneWireLine() {
  Serial.println("[SETUP] Setting up direct 1-Wire line");
  directTemps.begin();
  directTemps.setResolution(9);          // 9-bit = 0.5°C, needs 94ms for conversion, 12-bit = 0.0625°C but needs 750ms, we want faster updates for the impact line
  directTemps.setWaitForConversion(false); // non-blocking: we manage the 750ms ourselves

  line_direct.found = 0;
  uint8_t count = directTemps.getDeviceCount();
  for (uint8_t i = 0; i < count && line_direct.found < Config::MAX_SENSORS_PER_LINE; i++) {
    DeviceAddress addr;
    if (directTemps.getAddress(addr, i)) {
      memcpy(line_direct.addresses[line_direct.found], addr, 8);
      Serial.print("[SETUP]  Direct DS18B20 #"); Serial.print(line_direct.found + 1);
      Serial.print(" ROM: "); printDeviceAddress(addr); Serial.println();
      line_direct.found++;
    }
  }

  line_direct.present = (line_direct.found > 0);
  return line_direct.present;
}

bool setupINA260() {
  if (!ina260.begin(Config::ADDR_INA260, &Wire)) {
    Serial.println("[SETUP] INA260 not found"); return false;
  }

  // Configure hardware alert: fires when current > MAX_CURRENT_MA
  ina260.setAlertType(INA260_ALERT_OVERCURRENT);
  ina260.setAlertLimit(Config::MAX_CURRENT_MA);
  ina260.setAlertLatch(INA260_ALERT_LATCH_ENABLED);       // stays asserted until we read the register
  ina260.setAlertPolarity(INA260_ALERT_POLARITY_INVERTED); // active-low -> RISING edge on MCU pin

  // Connect the alert pin to our ISR
  pinMode(Config::PIN_INT_CURRENT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Config::PIN_INT_CURRENT), onOvercurrentIRQ, RISING);

  Serial.println("[SETUP] INA260 OK + alert configured");
  return true;
}

// Write raw registers to configure LIS3DH click/interrupt detection.
// The Adafruit driver doesn't expose these, so we write them directly.
static void configureImpactInterrupt() {
  writeIMUReg(0x20, Config::ODR_BITS_200HZ | 0x07); // CTRL_REG1: 200Hz, enable X/Y/Z axes
  writeIMUReg(0x21, 0x00);                           // CTRL_REG2: no high-pass filter
  writeIMUReg(0x22, 0x40);                           // CTRL_REG3: route IA1 event to INT1 pin
  writeIMUReg(0x23, 0x80 | 0x08 | Config::RANGE_BITS_16G); // CTRL_REG4: block update, ±16g
  writeIMUReg(0x24, 0x08);                           // CTRL_REG5: latch interrupt (stays until read)
  writeIMUReg(0x25, 0x00);                           // CTRL_REG6: no second interrupt config
  writeIMUReg(0x32, thresholdGToReg(Config::IMPACT_THRESHOLD_G, 16)); // INT1_THS: threshold
  writeIMUReg(0x33, Config::IMPACT_DURATION_SAMPLES);                  // INT1_DURATION: min samples
  writeIMUReg(0x30, 0x2A);                           // INT1_CFG: trigger on high X, Y, or Z
  (void)readIMUReg(0x31);                            // INT1_SRC: dummy read to clear any latched flag
}

bool setupLIS3DH() {
  if (!imu.begin(Config::ADDR_LIS3DH)) {
    Serial.println("[SETUP] LIS3DH not found"); return false;
  }

  imu.setRange(LIS3DH_RANGE_16_G);
  delay(10); // let the range change settle before writing interrupt registers
  configureImpactInterrupt();

  pinMode(Config::PIN_INT_IMU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Config::PIN_INT_IMU), onImpactIRQ, RISING);

  Serial.println("[SETUP] LIS3DH OK + impact interrupt configured");
  return true;
}

void initSDLogging() {
  if (!Config::ENABLE_SD_LOGGING) return;
  // TODO: SD.begin(Config::PIN_SD_CS) and open/create CSV file
}