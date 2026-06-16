#pragma once

// Standard libs
#include <Wire.h>
#include <SPI.h>
#include <math.h>    // isnan(), sqrtf()
#include <string.h>  // memcpy()

// Sensor libraries
#include <Adafruit_Sensor.h>
#include <7semi_SHT4x.h>
#include "Adafruit_INA260.h"
#include "Adafruit_Si7021.h"
#include <Adafruit_LIS3DH.h>
#include "Adafruit_DS248x.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#include "Config.h"

// DS18B20 1-Wire command bytes (from datasheet)
#define DS18B20_FAMILY_CODE          0x28  // first ROM byte identifies the sensor model
#define DS18B20_CMD_CONVERT_T        0x44  // tell sensor to start measuring
#define DS18B20_CMD_MATCH_ROM        0x55  // address one specific sensor by its ROM
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE  // read the result back

// ---- Data structs ----
// Each sensor type has its own reading struct.
// ok = false means the read failed; values will be NAN.

struct TH_Reading {
  bool  ok         = false;
  float temp_c     = NAN;
  float rh_percent = NAN;
};

struct PowerReading {
  bool  ok         = false;
  float current_ma = NAN;
  float bus_mv     = NAN;
  float power_mw   = NAN;
};

struct IMUReading {
  bool  ok    = false;
  float ax_g  = NAN;  // x-axis acceleration in g
  float ay_g  = NAN;
  float az_g  = NAN;
  float mag_g = NAN;  // vector magnitude: sqrt(ax^2+ay^2+az^2)
};

struct SafetyFlags {
  bool any_temp_limit          = false;  // latches once triggered
  bool any_humidity_limit      = false;  // latches once triggered
  bool overcurrent_limit       = false;  // latches once triggered
  bool shock_flag              = false;  // latches once triggered
  bool overcurrent_irq_latched = false;  // set when INA260 fires its alert pin
  bool power_cut_flag          = false;  // set once power has been cut
  bool sensors_ok              = true;
};

// One entry per DS2484-bridged 1-Wire line
struct DS2484Line {
  const char*      name;            // label for debug prints
  TwoWire*         wire;            // which I2C bus the DS2484 sits on
  Adafruit_DS248x* bridge;          // the DS2484 driver object
  uint8_t          roms[Config::MAX_SENSORS_PER_LINE][8]; // 64-bit ROM of each found sensor
  uint8_t          found            = 0;
  float            temperatures[Config::MAX_SENSORS_PER_LINE] = {NAN, NAN, NAN};
  bool             present          = false;
  bool             conversionPending = false; // true while waiting for temperature conversion
  unsigned long    conversionStartMs = 0;
};

// The one line wired directly (no DS2484 bridge)
struct DirectOneWireLine {
  DeviceAddress addresses[Config::MAX_SENSORS_PER_LINE];
  uint8_t       found               = 0;
  float         temperatures[Config::MAX_SENSORS_PER_LINE] = {NAN, NAN, NAN};
  bool          present             = false;
  bool          conversionPending   = false;
  unsigned long conversionStartMs   = 0;
};

// ---- Sensor driver objects ----
// Defined once in DataTypes.cpp; extern here so all files can use them.
extern SHT4x_7semi        sht41_bus0;
extern SHT4x_7semi        sht41_bus1;
extern Adafruit_Si7021    si7021_bus1;
extern Adafruit_INA260    ina260;
extern Adafruit_LIS3DH    imu;
extern Adafruit_DS248x    ds2484_bus0;
extern Adafruit_DS248x    ds2484_bus1;
extern OneWire            oneWireDirect;
extern DallasTemperature  directTemps;

// ---- 1-Wire line instances ----
extern DS2484Line         line_bus0;
extern DS2484Line         line_bus1;
extern DirectOneWireLine  line_direct;

// ---- Latest readings (updated each log cycle) ----
extern TH_Reading     reading_sht41_bus0;
extern TH_Reading     reading_sht41_bus1;
extern TH_Reading     reading_si7021_bus1;
extern PowerReading   reading_power;
extern IMUReading     reading_imu;
extern SafetyFlags    safety;

// ---- Interrupt flags ----
// Set to true inside the ISR, cleared and handled in the main loop.
// volatile = don't let the compiler cache these in a register
extern volatile bool overcurrent_irq;
extern volatile bool impact_irq;

// ---- Misc globals ----
extern bool          sensors_detected; // false if any sensor failed init
extern unsigned long lastLogMs;        // timestamp of last log line
extern uint32_t      logCounter;       // total lines logged so far