#include "Config.h"
#include "DataTypes.h"
#include "Utils.h"
#include "Setup.h"
#include "Sensors.h"
#include "Safety.h"
#include "Logging.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // wait for USB serial on ESP32-S3
  delay(500);
  Serial.println("[SETUP] Booting ...");

  // Output pins
  pinMode(LED_BUILTIN,           OUTPUT);
  pinMode(Config::PIN_POWER_CUT, OUTPUT);
  digitalWrite(Config::PIN_POWER_CUT, HIGH); // HIGH = power on (active-low switch)

  // Start both I2C buses
  Wire.begin();
  Wire.setClock(400000);                              // 400kHz fast mode
  Wire1.begin(Config::SDA1_PIN, Config::SCL1_PIN);
  Wire1.setClock(400000);

  // Init each sensor group; AND results so any failure marks sensors_detected = false
  sensors_detected &= setupHumiditySensors();
  sensors_detected &= setupDS2484Line(line_bus0);
  sensors_detected &= setupDS2484Line(line_bus1);
  sensors_detected &= setupDirectOneWireLine();
  sensors_detected &= setupINA260();
  sensors_detected &= setupLIS3DH();
  safety.sensors_ok = sensors_detected;

  initSDLogging();

  // Kick off the first DS18B20 conversion immediately so valid readings
  // are ready after ~94ms instead of waiting for the second log cycle
  startDS2484Conversion(line_bus0);
  startDS2484Conversion(line_bus1);
  startDirectOneWireConversion();

  if (Config::PRINT_CSV_HEADER_AT_BOOT) printCSVHeader();

  lastLogMs = millis();
}

void loop() {
  // 1. Handle any interrupt flags raised since last loop (fast, non-blocking)
  handleCurrentInterrupt();
  handleIMUInterrupt();

  // 2. Log at fixed period (100ms default)
  if (millis() - lastLogMs >= Config::LOG_PERIOD_MS) {
    lastLogMs += Config::LOG_PERIOD_MS; // increment rather than = millis() to avoid drift
    logOncePerPeriod();
  }

  // 3. Reflect alarm state on the built-in LED
  updateStatusLED();
}