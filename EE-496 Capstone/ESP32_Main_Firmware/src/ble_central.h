#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <Arduino.h>

// Global shared variable tracking target watts
extern double targetWatts;

// Initialization Routine
void initBLECentral();

// Core 0 Manager Routine
void updateBLECentral();

#endif // BLE_CENTRAL_H
