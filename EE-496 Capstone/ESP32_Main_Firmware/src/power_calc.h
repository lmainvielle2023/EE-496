#ifndef POWER_CALC_H
#define POWER_CALC_H

#include <Arduino.h>

void initPowerCalc();
void addRegularForce(float force);
void addSenseForce(float force);
void setRPM(float rpm);
void updatePowerCalc();

#endif // POWER_CALC_H
