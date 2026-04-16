#include "imu_rpm.h"
#include "LSM6DS3.h"
#include <Wire.h>

// Initialize the library instance
LSM6DS3 myIMU(I2C_MODE, 0x6A);

static bool imuInitialized = false;

// Filter constraints
const float EMA_ALPHA_RPM = 0.1f;
static float filteredRPM = 0.0f;
static bool rpmFilterInitialized = false;

void initIMU() {
  Serial.println("Initializing IMU (Library Mode)...");

  // CRITICAL: You STILL need to power the IMU pin on this specific board!
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(50);

  if (myIMU.begin() != 0) {
    Serial.println("IMU Error: Device not found.");
    imuInitialized = false;
  } else {
    Serial.println("IMU Initialized Successfully!");
    imuInitialized = true;
  }
}

float getPedalRPM() {
  if (!imuInitialized) {
    Serial.println("[IMU ERROR] Not Initialized! Returning 0 RPM");
    return 0.0f;
  }

  // The library handles the Wire1 bus, bit-shifting, and scaling automatically!
  // It returns degrees per second directly.
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  // Magnitude of rotation (deg/s)
  float gyroMagnitude = sqrt((gx * gx) + (gy * gy) + (gz * gz));

  // RPM = (deg/s / 360) * 60 = deg/s / 6.0
  float rawRPM = gyroMagnitude / 6.0f;

  if (isnan(rawRPM) || rawRPM < 5.0f)
    rawRPM = 0.0f;

  if (!rpmFilterInitialized) {
    filteredRPM = rawRPM;
    rpmFilterInitialized = true;
  } else {
    filteredRPM =
        (EMA_ALPHA_RPM * rawRPM) + ((1.0f - EMA_ALPHA_RPM) * filteredRPM);
  }

  return filteredRPM;
}