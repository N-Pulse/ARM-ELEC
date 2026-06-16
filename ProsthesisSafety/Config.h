#pragma once

// All tunable parameters live here. Edit this file to change behavior.
namespace Config {

  // ---- Physical pin assignments ----
  static const uint8_t SDA1_PIN                = 9;   // I2C bus 1 data
  static const uint8_t SCL1_PIN                = 10;  // I2C bus 1 clock
  static const uint8_t DIRECT_ONEWIRE_PIN      = 7;   // 1-Wire bus (no bridge chip)
  static const uint8_t PIN_INT_IMU             = 5;   // LIS3DH interrupt -> MCU
  static const uint8_t PIN_INT_CURRENT         = 6;   // INA260 alert -> MCU
  static const uint8_t PIN_POWER_CUT           = 11;  // pull LOW to cut load power
  static const uint8_t PIN_SD_CS               = 12;  // SD chip-select (future)

  // ---- I2C addresses ----
  static const uint8_t ADDR_DS2484             = 0x18;
  static const uint8_t ADDR_LIS3DH            = 0x19;
  static const uint8_t ADDR_INA260            = 0x40;
  // SHT41 uses its default address, no override needed

  // ---- How many DS18B20s to scan per 1-Wire line ----
  static const uint8_t MAX_SENSORS_PER_LINE    = 3;

  // ---- Timing ----
  static const uint32_t LOG_PERIOD_MS                  = 500;  // how often the main loop logs
  static const uint32_t DIRECT_DS18B20_CONVERSION_MS   = 94;   // DS18B20 needs 94ms (9-bit) or 750ms (12-bit)to convert
  static const uint32_t DS2484_DS18B20_CONVERSION_MS   = 94;  // same for bridge-connected ones

  // ---- Safety limits — trigger alerts above these values ----
  static constexpr float MAX_TEMP_C            = 85.0f;   // °C  (normal use: 85.0)
  static constexpr float MAX_RH_PERCENT        = 50.0f;   // %RH (normal use: 95.0)
  static constexpr float MAX_CURRENT_MA        = 40.0f;   // mA  (normal use: 4000.0)

  // ---- What to do when a limit is crossed ----
  // set to true to actually cut power on that event
  static const bool ENABLE_POWER_CUT_ON_OVERCURRENT      = false;
  static const bool ENABLE_POWER_CUT_ON_OVERHEAT         = false;
  static const bool ENABLE_POWER_CUT_ON_OVERHUMIDITY     = false;
  static const bool ENABLE_POWER_CUT_ON_CONFIRMED_SHOCK  = false;

  // ---- Impact detection tuning (LIS3DH) ----
  static constexpr float  IMPACT_THRESHOLD_G      = 4.0f;  // g to trigger interrupt
  // static constexpr float  CONFIRM_PEAK_G          = 5.5f;  // g to confirm as real shock
  static const uint8_t    IMPACT_DURATION_SAMPLES = 1;     // consecutive samples above threshold
  static const uint8_t    ODR_BITS_200HZ          = 0x60;  // output data rate register value
  static const uint8_t    RANGE_BITS_16G          = 0x30;  // ±16g range register value

  // ---- Sensor mapping: hard-coded 1-Wire addresses and per-sensor thresholds ----
  // Run once with thresholds zeroed to discover your sensors, then copy their addresses here
  enum class SensorPurpose { BATTERY, SOCKET, MOTOR, UNKNOWN };

  struct SensorMapping {
    uint8_t address[8];
    SensorPurpose purpose;
    float temp_limit_c;
  };

  // Bus 0 (DS2484) — all motor sensors
  static constexpr SensorMapping bus0_sensors[] = {
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6C, 0x7E, 0xBA, 0x6B }, SensorPurpose::MOTOR, 85.0f },
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0x93, 0xDC, 0x84 }, SensorPurpose::MOTOR, 85.0f },
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x7B, 0x21, 0xAF, 0x96 }, SensorPurpose::MOTOR, 85.0f },
  };
  static constexpr uint8_t bus0_sensors_count = sizeof(bus0_sensors) / sizeof(bus0_sensors[0]);

  // Bus 1 (DS2484) — all motor sensors
  static constexpr SensorMapping bus1_sensors[] = {
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0x49, 0x39, 0xED }, SensorPurpose::MOTOR, 85.0f },
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0xC5, 0x78, 0x97 }, SensorPurpose::MOTOR, 85.0f },
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0xE5, 0xC2, 0x1A }, SensorPurpose::MOTOR, 85.0f },
  };
  static constexpr uint8_t bus1_sensors_count = sizeof(bus1_sensors) / sizeof(bus1_sensors[0]);

  // Direct line (no DS2484 bridge) — battery, socket, motor
  static constexpr SensorMapping direct_sensors[] = {
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0x54, 0x36, 0xC9 }, SensorPurpose::BATTERY, 45.0f },
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0x86, 0x2C, 0xE3 }, SensorPurpose::SOCKET,  41.0f }, // 41
    {{ 0x28, 0xFF, 0x64, 0x0E, 0x6D, 0x5B, 0xB0, 0x80 }, SensorPurpose::MOTOR,   85.0f },
  };
  static constexpr uint8_t direct_sensors_count = sizeof(direct_sensors) / sizeof(direct_sensors[0]);

  // ---- Serial / logging ----
  static const bool     PRINT_CSV_HEADER_AT_BOOT        = false;
  static const uint16_t PRINT_CSV_HEADER_EVERY_N_LOGS   = false;   // reprint header every N lines
  static const bool     ENABLE_SD_LOGGING               = false; // SD not wired yet
}