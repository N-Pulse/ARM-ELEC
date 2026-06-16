#pragma once
#include "DataTypes.h"
#include "Utils.h"

// One function per sensor group; each returns false if init fails
bool setupHumiditySensors();
bool setupDS2484Line(DS2484Line& line);
bool setupDirectOneWireLine();
bool setupINA260();
bool setupLIS3DH();
void initSDLogging();

// ISRs must be declared here so they can be passed to attachInterrupt()
void ARDUINO_ISR_ATTR onOvercurrentIRQ();
void ARDUINO_ISR_ATTR onImpactIRQ();