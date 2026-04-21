#include <Arduino.h>
#include "gps_terrain.h"
#include "ble_goal_broadcaster.h"

// The baseline watt target before terrain adjustment.
// Uphill adds up to 100W, downhill subtracts up to 100W.
constexpr float BASE_GOAL_WATTS = 200.0f;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting GPS Goal-Watts Node...");

    // BLE must init before WiFi on ESP32 to claim contiguous RAM
    initBLEGoalBroadcaster();
    initGPSTerrain();
}

void loop() {
    float goalWatts = updateGPSTerrain(BASE_GOAL_WATTS);
    updateBLEGoalWatts(goalWatts);
    delay(50);
}
