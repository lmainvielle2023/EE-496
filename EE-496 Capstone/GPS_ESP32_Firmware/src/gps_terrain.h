#ifndef GPS_TERRAIN_H
#define GPS_TERRAIN_H

#include <Arduino.h>

void initGPSTerrain();

// Returns the goal watts based on current terrain.
// Falls back to baseGoalWatts if GPS/WiFi is unavailable.
float updateGPSTerrain(float baseGoalWatts);

#endif // GPS_TERRAIN_H
