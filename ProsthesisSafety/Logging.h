#pragma once
#include "DataTypes.h"

void printCSVHeader();     // column names row
void printCSVLine();       // one data row with current readings + flags
void logToSDCard();        // stub — mirrors printCSVLine() to SD when enabled
void logOncePerPeriod();   // called by loop(); reads sensors, evaluates safety, then logs
void updateStatusLED();    // blink LED_BUILTIN if any alarm is active