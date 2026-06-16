#pragma once
#include "DataTypes.h"

// Single-shot reads for fast sensors (I2C, call each log cycle)
TH_Reading    readSHT41Sensor(SHT4x_7semi& sensor);
TH_Reading    readSi7021Sensor(Adafruit_Si7021& sensor);
PowerReading  readPowerSensor();
IMUReading    readIMUSensor();
void          readFastSensors(); // calls all four above and stores results

// Non-blocking DS18B20 state machine (tied to log cycle).
// DS18B20 needs ~94ms to convert: start on log N, read on log N+1.
void startDS2484Conversion(DS2484Line& line);
void finishDS2484Conversion(DS2484Line& line);
void startDirectOneWireConversion();
void finishDirectOneWireConversion();