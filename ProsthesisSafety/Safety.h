#pragma once
#include "DataTypes.h"
#include "Sensors.h"

// Check all current readings against limits; set safety flags accordingly
void evaluateSafetyConditions();

// Handle interrupt flags set by ISRs (call at top of loop())
void handleCurrentInterrupt();
void handleIMUInterrupt();