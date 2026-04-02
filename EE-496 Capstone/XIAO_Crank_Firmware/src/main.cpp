#include <Arduino.h>
#include "crank_pins.h"
#include "load_cell.h"
#include "ble_broadcaster.h"

// Assume a generic pedaling RPM until IMU is implemented
const float GENERIC_RPM = 60.0f;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Starting XIAO Crank Sensor Node...");

    // Initialize systems
    initLoadCell();
    initBLEBroadcaster();
}

void loop() {
    // 1. Fetch current live push force from user in lbs
    float force = getPedalForce();
    
    // 2. Convert tangential push to physical Wattage representation
    float watts = calculateWatts(force, GENERIC_RPM);

    // 3. Print out to Serial Monitor for verification
    Serial.print("Force (lbs): ");
    Serial.print(force, 2);
    Serial.print(" | Power (Watts) @ 60 RPM: ");
    Serial.println(watts, 2);

    // 4. Send over BLE
    updateBLEBroadcaster(watts);
    
    // Loop interval
    delay(100);
}
