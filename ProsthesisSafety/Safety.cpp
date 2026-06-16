#include "Safety.h"
#include "Utils.h"

void evaluateSafetyConditions() {
  // Don't reset latching flags — they stay up once triggered
  bool new_temp_limit     = false;
  bool new_humidity_limit = false;
  bool new_overcurrent    = false;

  // Mark sensor health — any single failure marks the system degraded
  if (!reading_sht41_bus0.ok || !reading_sht41_bus1.ok ||
      !reading_si7021_bus1.ok || !reading_power.ok || !reading_imu.ok) {
    safety.sensors_ok = false;
  }

  // T/RH limit checks on I2C humidity sensors
  if (aboveTempLimit(reading_sht41_bus0.temp_c)  ||
      aboveTempLimit(reading_sht41_bus1.temp_c)  ||
      aboveTempLimit(reading_si7021_bus1.temp_c)) {
    new_temp_limit = true;
  }

  if (aboveHumidityLimit(reading_sht41_bus0.rh_percent) ||
      aboveHumidityLimit(reading_sht41_bus1.rh_percent) ||
      aboveHumidityLimit(reading_si7021_bus1.rh_percent)) {
    new_humidity_limit = true;
  }

  // Temperature limit checks on DS18B20s across all 3 lines (with per-sensor limits)
  for (uint8_t i = 0; i < line_bus0.found; i++) {
    float limit = Config::MAX_TEMP_C;
    getSensorTempLimit(line_bus0.roms[i], limit);
    if (!isnan(line_bus0.temperatures[i]) && line_bus0.temperatures[i] > limit)
      new_temp_limit = true;
  }
  for (uint8_t i = 0; i < line_bus1.found; i++) {
    float limit = Config::MAX_TEMP_C;
    getSensorTempLimit(line_bus1.roms[i], limit);
    if (!isnan(line_bus1.temperatures[i]) && line_bus1.temperatures[i] > limit)
      new_temp_limit = true;
  }
  for (uint8_t i = 0; i < line_direct.found; i++) {
    float limit = Config::MAX_TEMP_C;
    getSensorTempLimit((const uint8_t*)line_direct.addresses[i], limit);
    if (!isnan(line_direct.temperatures[i]) && line_direct.temperatures[i] > limit)
      new_temp_limit = true;
  }

  // Software overcurrent check (INA260 polled reading, separate from the IRQ path)
  if (!isnan(reading_power.current_ma) && reading_power.current_ma > Config::MAX_CURRENT_MA)
    new_overcurrent = true;

  // Latch temp and humidity flags (stay true once triggered)
  safety.any_temp_limit     = safety.any_temp_limit || new_temp_limit;
  safety.any_humidity_limit = safety.any_humidity_limit || new_humidity_limit;
  safety.overcurrent_limit  = safety.overcurrent_limit || new_overcurrent;

  // Trigger power cuts
  if (new_temp_limit && Config::ENABLE_POWER_CUT_ON_OVERHEAT)
    requestPowerCut("temperature limit exceeded");
  if (new_humidity_limit && Config::ENABLE_POWER_CUT_ON_OVERHUMIDITY)
    requestPowerCut("humidity limit exceeded");
  if (new_overcurrent && Config::ENABLE_POWER_CUT_ON_OVERCURRENT)
    requestPowerCut("overcurrent (polled)");
}

void handleCurrentInterrupt() {
  if (!overcurrent_irq) return;

  noInterrupts(); overcurrent_irq = false; interrupts();

  // Power cut already triggered from ISR; verify via I2C and log
  if (ina260.alertFunctionFlag()) {
    safety.overcurrent_irq_latched = true;
  }
}

void handleIMUInterrupt() {
  if (!impact_irq) return;

  noInterrupts(); impact_irq = false; interrupts();

  // Power cut already triggered from ISR; verify via I2C and log
  uint8_t src = readIMUReg(0x31);
  bool ia     = src & 0x40; // bit 6 = IA (Interrupt Active): at least one condition was met

  if (ia) {
    safety.shock_flag = true;
  }
}