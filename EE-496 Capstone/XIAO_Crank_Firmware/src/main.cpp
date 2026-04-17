#include "ble_broadcaster.h"
#include "crank_pins.h"
#include "imu_rpm.h"
#include "load_cell.h"
#include <Arduino.h>

#ifndef PEDAL_IS_RIGHT_NODE
#error "PEDAL_IS_RIGHT_NODE must be set in platformio.ini"
#endif

constexpr bool kIsRightPedalNode = PEDAL_IS_RIGHT_NODE != 0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to attach
  Serial.println(kIsRightPedalNode ? "Starting XIAO Crank Sensor Node (RIGHT)..."
                                   : "Starting XIAO Crank Sensor Node (LEFT)...");

  // Initialize systems
  initLoadCell();
  initBLEBroadcaster(kIsRightPedalNode);

  if (kIsRightPedalNode) {
    initIMU();
  }
}

void loop() {
  static unsigned long lastPrintMs = 0;

  // 1. Fetch current live push force from user in lbs
  float force = getPedalForce();
  float rpm = 0.0f;

  // 2. Read IMU for RPM if we are the Right Node
  if (kIsRightPedalNode) {
    rpm = getPedalRPM();
  }

  // 3. Print out to Serial Monitor for verification without flooding the port
  if (millis() - lastPrintMs >= 100) {
    lastPrintMs = millis();
    Serial.print("Force (lbs): ");
    Serial.print(force, 2);
    if (kIsRightPedalNode) {
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
