#include "ble_broadcaster.h"
#include "crank_pins.h"
#include "imu_rpm.h"
#include "load_cell.h"
#include <Arduino.h>

// Set this to true when compiling for the Right Pedal (Seeed Sense)
// Set this to false when compiling for the Left Pedal (Standard Seeed)
#define IS_RIGHT_NODE false // Currently configured for LEFT node

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to attach
  Serial.println(IS_RIGHT_NODE ? "Starting XIAO Crank Sensor Node (RIGHT)..."
                               : "Starting XIAO Crank Sensor Node (LEFT)...");

  // Initialize systems
  initLoadCell();
  initBLEBroadcaster(IS_RIGHT_NODE);

  if (IS_RIGHT_NODE) {
    initIMU();
  }
}

void loop() {
  // 1. Fetch current live push force from user in lbs
  float force = getPedalForce();
  float rpm = 0.0f;

  // 2. Read IMU for RPM if we are the Right Node
  if (IS_RIGHT_NODE) {
    rpm = getPedalRPM();
  }

  // 3. Print out to Serial Monitor for verification
  Serial.print("Force (lbs): ");
  Serial.print(force, 2);
  if (IS_RIGHT_NODE) {
    Serial.print(" | RPM: ");
    Serial.print(rpm, 1);
  }
  Serial.println();

  // 4. Send over BLE
  updateBLEBroadcaster(force, rpm);

  // Loop interval
  delay(100);
}
