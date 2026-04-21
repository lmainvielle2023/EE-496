#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <Arduino.h>

// Global shared variable tracking rider power
extern double riderWatts;
extern double targetGoalWatts;

// Initialization Routine
void initBLECentral();

// Core 0 Manager Routine
void updateBLECentral();

bool isLeftCrankConnected();
bool isRightCrankConnected();
bool isGoalNodeConnected();
double getTargetGoalWatts();

#endif // BLE_CENTRAL_H
