#include "ble_broadcaster.h"
#include "crank_pins.h"
#include "imu_rpm.h"
#include "load_cell.h"
#include <Arduino.h>

#ifndef PEDAL_IS_SENSE_NODE
#error "PEDAL_IS_SENSE_NODE must be set in platformio.ini"
#endif

constexpr bool kIsSenseNode = PEDAL_IS_SENSE_NODE != 0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to attach
  Serial.println(kIsSenseNode ? "Starting XIAO Crank Sensor Node (SENSE)..."
                              : "Starting XIAO Crank Sensor Node (REGULAR)...");

  if (kIsSenseNode) {
    initIMU();
  }

  // Initialize systems
  initLoadCell();
  initBLEBroadcaster(kIsSenseNode);
}

void loop() {
  static unsigned long lastPrintMs = 0;

  // 1. Fetch current live push force from user in lbs
  float force = getPedalForce();
  float rpm = 0.0f;

  // 2. Read IMU for RPM only on the Sense node
  if (kIsSenseNode) {
    rpm = getPedalRPM();
  }

  // 3. Print out to Serial Monitor for verification without flooding the port
  if (millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();
    Serial.print("Force (lbs): ");
    Serial.print(force, 2);
    if (kIsSenseNode) {
      Serial.print(" | RPM: ");
      Serial.print(rpm, 1);
    }
    Serial.println();
  }

  // 4. Send over BLE
  updateBLEBroadcaster(force, rpm);

  // Poll often so the BLE stack and HX711 sampler stay responsive.
  delay(20);
}
