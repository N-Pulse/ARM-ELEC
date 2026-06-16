#include "Logging.h"
#include "Utils.h"
#include "Sensors.h"
#include "Safety.h"

void printCSVHeader() {
  Serial.println(
    "t_ms,"
    "hand_temp_c,hand_rh,"
    "wrist_temp_c,wrist_rh,"
    "socket_temp_c,socket_rh,"
    "bus0_ds1_c,bus0_ds2_c,bus0_ds3_c,"
    "bus1_ds1_c,bus1_ds2_c,bus1_ds3_c,"
    "battery,socket,direct_ds1_c,"
    "imu_ax_g,imu_ay_g,imu_az_g,imu_mag_g,"
    "current_ma,bus_mv,power_mw,"
    "temp_limit_flag,humidity_limit_flag,overcurrent_limit_flag,"
    "shock_flag,power_cut_flag,sensors_ok"
  );
}

void printCSVLine() {
  Serial.print(millis()); Serial.print(',');

  // I2C humidity sensors
  printFloatCSV(reading_sht41_bus0.temp_c);     Serial.print(',');
  printFloatCSV(reading_sht41_bus0.rh_percent); Serial.print(',');
  printFloatCSV(reading_sht41_bus1.temp_c);     Serial.print(',');
  printFloatCSV(reading_sht41_bus1.rh_percent); Serial.print(',');
  printFloatCSV(reading_si7021_bus1.temp_c);    Serial.print(',');
  printFloatCSV(reading_si7021_bus1.rh_percent); Serial.print(',');

  // DS18B20 temperatures — always print MAX_SENSORS_PER_LINE columns,
  // filling with "nan" for slots where no sensor was found
  for (uint8_t i = 0; i < Config::MAX_SENSORS_PER_LINE; i++) {
    printFloatCSV(i < line_bus0.found ? line_bus0.temperatures[i] : NAN);
    Serial.print(',');
  }
  for (uint8_t i = 0; i < Config::MAX_SENSORS_PER_LINE; i++) {
    printFloatCSV(i < line_bus1.found ? line_bus1.temperatures[i] : NAN);
    Serial.print(',');
  }
  for (uint8_t i = 0; i < Config::MAX_SENSORS_PER_LINE; i++) {
    printFloatCSV(i < line_direct.found ? line_direct.temperatures[i] : NAN);
    if (i < Config::MAX_SENSORS_PER_LINE - 1) Serial.print(',');
  }

  // IMU
  Serial.print(','); printFloatCSV(reading_imu.ax_g);
  Serial.print(','); printFloatCSV(reading_imu.ay_g);
  Serial.print(','); printFloatCSV(reading_imu.az_g);
  Serial.print(','); printFloatCSV(reading_imu.mag_g);

  // Power
  Serial.print(','); printFloatCSV(reading_power.current_ma);
  Serial.print(','); printFloatCSV(reading_power.bus_mv);
  Serial.print(','); printFloatCSV(reading_power.power_mw);

  // Safety flags as 0/1
  Serial.print(','); Serial.print(safety.any_temp_limit        ? 1 : 0);
  Serial.print(','); Serial.print(safety.any_humidity_limit    ? 1 : 0);
  Serial.print(','); Serial.print(safety.overcurrent_limit     ? 1 : 0);
  Serial.print(','); Serial.print(safety.shock_flag            ? 1 : 0);
  Serial.print(','); Serial.print(safety.power_cut_flag        ? 1 : 0);
  Serial.print(','); Serial.println(safety.sensors_ok          ? 1 : 0);
}

void logToSDCard() {
  if (!Config::ENABLE_SD_LOGGING) return;
  // TODO: mirror printCSVLine() to an open SD File handle; flush every N lines to reduce wear
}

// One complete log cycle: read -> evaluate -> print
void logOncePerPeriod() {
  // Read results from PREVIOUS conversion (already done)
  finishDS2484Conversion(line_bus0);
  finishDS2484Conversion(line_bus1);
  finishDirectOneWireConversion();

  readFastSensors();
  evaluateSafetyConditions();
  printCSVLine();
  logToSDCard();

  // Start NEXT conversion immediately after reading
  startDS2484Conversion(line_bus0);
  startDS2484Conversion(line_bus1);
  startDirectOneWireConversion();

  logCounter++;
}

// LED on = at least one alarm is active
void updateStatusLED() {
  bool any_alarm =
      safety.any_temp_limit         ||
      safety.any_humidity_limit     ||
      safety.overcurrent_limit      ||
      safety.shock_flag             ||
      safety.overcurrent_irq_latched||
      safety.power_cut_flag         ||
      !safety.sensors_ok;

  digitalWrite(LED_BUILTIN, any_alarm ? HIGH : LOW);
}

// One complete log cycle: read -> evaluate -> print
// void logOncePerPeriod() {
//   readFastSensors();
//   evaluateSafetyConditions();
//   // check if not none
//   if (Config::PRINT_CSV_HEADER_EVERY_N_LOGS) {
//     if (logCounter % Config::PRINT_CSV_HEADER_EVERY_N_LOGS == 0) {
//       printCSVHeader();
//     }
//   }
//   printCSVLine();
//   logToSDCard();
//   logCounter++;
// }

// // LED on = at least one alarm is active
// void updateStatusLED() {
//   bool any_alarm =
//       safety.any_temp_limit         ||
//       safety.any_humidity_limit     ||
//       safety.overcurrent_limit      ||
//       safety.shock_flag             ||
//       safety.overcurrent_irq_latched||
//       safety.power_cut_flag         ||
//       !safety.sensors_ok;

//   digitalWrite(LED_BUILTIN, any_alarm ? HIGH : LOW);
// }