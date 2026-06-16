#pragma once
#include "DataTypes.h"

// Printing helpers
void    printROM(const uint8_t* rom);              // print 8-byte 1-Wire ROM as hex
void    printDeviceAddress(const DeviceAddress addr);
void    printFloatCSV(float value, uint8_t decimals = 2); // prints "nan" if NAN

// Threshold checks — return true if value is valid AND over the limit
bool    aboveTempLimit(float value);
bool    aboveHumidityLimit(float value);

// Convert g threshold to LIS3DH register byte (depends on chosen ±range)
uint8_t thresholdGToReg(float g, int range_g);

// Direct register read/write for LIS3DH (used for interrupt config)
void    writeIMUReg(uint8_t reg, uint8_t val);
uint8_t readIMUReg(uint8_t reg);

// Sensor mapping lookup
bool    getSensorTempLimit(const uint8_t* address, float& out_limit);
bool    isSensorByAddress(const uint8_t* address, Config::SensorPurpose purpose);

// Cut power and log why; sets safety.power_cut_flag
void    requestPowerCut(const char* reason);