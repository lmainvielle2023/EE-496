#ifndef LOAD_CELL_H
#define LOAD_CELL_H

#include <Arduino.h>

void initLoadCell();
float getPedalForce();
float calculateWatts(float force_lbs, float rpm);

#endif // LOAD_CELL_H
