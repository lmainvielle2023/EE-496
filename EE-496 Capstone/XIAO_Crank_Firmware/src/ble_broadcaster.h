#ifndef BLE_BROADCASTER_H
#define BLE_BROADCASTER_H

#include <Arduino.h>

void initBLEBroadcaster(bool isRightNode);
void updateBLEBroadcaster(float current_force, float current_rpm);

#endif // BLE_BROADCASTER_H
