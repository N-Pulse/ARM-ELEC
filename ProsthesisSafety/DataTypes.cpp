#include "DataTypes.h"

// Actual memory allocation for everything declared extern in DataTypes.h.
// Only this file defines them; all other files just reference them.

// ---- Sensor drivers ----
SHT4x_7semi       sht41_bus0;
SHT4x_7semi       sht41_bus1;
Adafruit_Si7021   si7021_bus1(&Wire1);  // explicitly on bus 1
Adafruit_INA260   ina260;
Adafruit_LIS3DH   imu = Adafruit_LIS3DH();
Adafruit_DS248x   ds2484_bus0;
Adafruit_DS248x   ds2484_bus1;
OneWire           oneWireDirect(Config::DIRECT_ONEWIRE_PIN);
DallasTemperature directTemps(&oneWireDirect); // wraps OneWire for DS18B20 convenience

// ---- 1-Wire lines ----
DS2484Line        line_bus0   = {"line_bus0", &Wire,  &ds2484_bus0};
DS2484Line        line_bus1   = {"line_bus1", &Wire1, &ds2484_bus1};
DirectOneWireLine line_direct; // addresses filled during setup

// ---- Readings (start invalid; filled after first sensor poll) ----
TH_Reading    reading_sht41_bus0;
TH_Reading    reading_sht41_bus1;
TH_Reading    reading_si7021_bus1;
PowerReading  reading_power;
IMUReading    reading_imu;
SafetyFlags   safety;

// ---- Interrupt flags ----
volatile bool overcurrent_irq = false;
volatile bool impact_irq      = false;

// ---- Misc ----
bool          sensors_detected = true;
unsigned long lastLogMs        = 0;
uint32_t      logCounter       = 0;