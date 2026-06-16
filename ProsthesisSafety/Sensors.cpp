#include "Sensors.h"

TH_Reading readSHT41Sensor(SHT4x_7semi& sensor) {
  TH_Reading out;
  // readTemperatureHumidity returns false on CRC error or timeout
  out.ok = sensor.readTemperatureHumidity(out.temp_c, out.rh_percent);
  return out;
}

TH_Reading readSi7021Sensor(Adafruit_Si7021& sensor) {
  TH_Reading out;
  out.rh_percent = sensor.readHumidity();
  out.temp_c     = sensor.readTemperature();
  out.ok         = !isnan(out.rh_percent) && !isnan(out.temp_c);
  return out;
}

PowerReading readPowerSensor() {
  PowerReading out;
  out.current_ma = ina260.readCurrent();
  out.bus_mv     = ina260.readBusVoltage();
  out.power_mw   = ina260.readPower();
  out.ok         = !isnan(out.current_ma);
  return out;
}

IMUReading readIMUSensor() {
  IMUReading out;
  sensors_event_t event;
  imu.getEvent(&event); // fills event.acceleration in m/s²
  // Convert m/s² to g (1g = 9.80665 m/s²)
  out.ax_g  = event.acceleration.x / 9.80665f;
  out.ay_g  = event.acceleration.y / 9.80665f;
  out.az_g  = event.acceleration.z / 9.80665f;
  out.mag_g = sqrtf(out.ax_g*out.ax_g + out.ay_g*out.ay_g + out.az_g*out.az_g);
  out.ok    = true;
  return out;
}

// Read all fast sensors and store into global reading_* variables
void readFastSensors() {
  reading_sht41_bus0  = readSHT41Sensor(sht41_bus0);
  reading_sht41_bus1  = readSHT41Sensor(sht41_bus1);
  reading_si7021_bus1 = readSi7021Sensor(si7021_bus1);
  reading_power       = readPowerSensor();
  reading_imu         = readIMUSensor();
}

// Phase 1: broadcast "start converting" to all sensors on this line, then set a timer
void startDS2484Conversion(DS2484Line& line) {
  if (!line.present || line.conversionPending || line.found == 0) return;
  for (uint8_t i = 0; i < line.found; i++) {
    line.bridge->OneWireReset();
    line.bridge->OneWireWriteByte(DS18B20_CMD_MATCH_ROM);
    for (uint8_t j = 0; j < 8; j++) line.bridge->OneWireWriteByte(line.roms[i][j]);
    line.bridge->OneWireWriteByte(DS18B20_CMD_CONVERT_T);
  }
  line.conversionPending  = true;
  line.conversionStartMs  = millis();
}

// Phase 2: once 94ms have passed, read the scratchpad from each sensor
void finishDS2484Conversion(DS2484Line& line) {
  if (!line.conversionPending) return;
  if (millis() - line.conversionStartMs < Config::DS2484_DS18B20_CONVERSION_MS) return;

  for (uint8_t i = 0; i < line.found; i++) {
    line.bridge->OneWireReset();
    line.bridge->OneWireWriteByte(DS18B20_CMD_MATCH_ROM);
    for (uint8_t j = 0; j < 8; j++) line.bridge->OneWireWriteByte(line.roms[i][j]);
    line.bridge->OneWireWriteByte(DS18B20_CMD_READ_SCRATCHPAD);

    uint8_t data[9];
    for (uint8_t k = 0; k < 9; k++) line.bridge->OneWireReadByte(&data[k]);
    // Raw value is a signed 16-bit int: LSB in data[0], MSB in data[1]
    // Divide by 16 because the last 4 bits are the fractional part (2^-4 = 0.0625°C/LSB)
    int16_t raw          = (data[1] << 8) | data[0];
    line.temperatures[i] = (float)raw / 16.0f;
  }
  line.conversionPending = false;
}

// Same state machine for the direct (no bridge) line — uses DallasTemperature library instead
void startDirectOneWireConversion() {
  if (!line_direct.present || line_direct.conversionPending || line_direct.found == 0) return;
  directTemps.requestTemperatures(); // sends CONVERT_T to all devices on the bus
  line_direct.conversionPending = true;
  line_direct.conversionStartMs = millis();
}

void finishDirectOneWireConversion() {
  if (!line_direct.conversionPending) return;
  if (millis() - line_direct.conversionStartMs < Config::DIRECT_DS18B20_CONVERSION_MS) return;

  for (uint8_t i = 0; i < line_direct.found; i++) {
    line_direct.temperatures[i] = directTemps.getTempC(line_direct.addresses[i]);
  }
  line_direct.conversionPending = false;
}

