#include "imu_rpm.h"
#include <Wire.h>
#include <math.h>

namespace {

#if defined(TARGET_SEEED_XIAO_NRF52840_SENSE) || defined(TARGET_SEEED_XIAO_NRF52840_SENSE_PLUS)
TwoWire &imuWire = Wire1;
#else
TwoWire &imuWire = Wire;
#endif

constexpr uint8_t kImuAddress = 0x6A;
constexpr uint8_t kWhoAmIRegister = 0x0F;
constexpr uint8_t kCtrl2GRegister = 0x11;
constexpr uint8_t kCtrl3CRegister = 0x12;
constexpr uint8_t kGyroOutputRegister = 0x22;
constexpr uint8_t kExpectedWhoAmIPrimary = 0x6A;
constexpr uint8_t kExpectedWhoAmIAlternate = 0x69;
constexpr uint8_t kGyroConfig104Hz245Dps = 0x40;
constexpr uint8_t kControl3Config = 0x44; // BDU + auto-increment
constexpr float kGyroDpsPerLsb = 0.00875f; // 245 dps full-scale
constexpr float kRpmFilterAlpha = 0.10f;
constexpr float kMinimumValidRpm = 5.0f;
constexpr unsigned long kRpmStaleTimeoutMs = 250;

bool imuInitialized = false;
float filteredRPM = 0.0f;
bool rpmFilterInitialized = false;
unsigned long lastFreshImuSampleMs = 0;

void resetRpmFilter() {
  filteredRPM = 0.0f;
  rpmFilterInitialized = false;
}

bool readRegisters(uint8_t startRegister, uint8_t *buffer, size_t length) {
  imuWire.beginTransmission(kImuAddress);
  imuWire.write(startRegister);
  if (imuWire.endTransmission(false) != 0) {
    return false;
  }

  if (imuWire.requestFrom(kImuAddress, static_cast<uint8_t>(length)) != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (!imuWire.available()) {
      return false;
    }
    buffer[i] = imuWire.read();
  }

  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  imuWire.beginTransmission(kImuAddress);
  imuWire.write(reg);
  imuWire.write(value);
  return imuWire.endTransmission() == 0;
}

bool readGyroDps(float &gx, float &gy, float &gz) {
  uint8_t rawBytes[6];
  if (!readRegisters(kGyroOutputRegister, rawBytes, sizeof(rawBytes))) {
    return false;
  }

  const int16_t rawX = static_cast<int16_t>((rawBytes[1] << 8) | rawBytes[0]);
  const int16_t rawY = static_cast<int16_t>((rawBytes[3] << 8) | rawBytes[2]);
  const int16_t rawZ = static_cast<int16_t>((rawBytes[5] << 8) | rawBytes[4]);

  gx = rawX * kGyroDpsPerLsb;
  gy = rawY * kGyroDpsPerLsb;
  gz = rawZ * kGyroDpsPerLsb;
  return true;
}

} // namespace

void initIMU() {
  Serial.println("Initializing IMU (direct I2C mode)...");

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(50);
#endif

  imuWire.begin();
  imuWire.setClock(400000);

  uint8_t whoAmI = 0;
  if (!readRegisters(kWhoAmIRegister, &whoAmI, 1) ||
      (whoAmI != kExpectedWhoAmIPrimary &&
       whoAmI != kExpectedWhoAmIAlternate)) {
    Serial.println("IMU Error: Device not found.");
    imuInitialized = false;
    return;
  }

  if (!writeRegister(kCtrl3CRegister, kControl3Config) ||
      !writeRegister(kCtrl2GRegister, kGyroConfig104Hz245Dps)) {
    Serial.println("IMU Error: Failed to configure gyro.");
    imuInitialized = false;
    return;
  }

  resetRpmFilter();
  lastFreshImuSampleMs = millis();
  imuInitialized = true;
  Serial.println("IMU Initialized Successfully!");
}

float getPedalRPM() {
  if (!imuInitialized) {
    return 0.0f;
  }

  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  if (readGyroDps(gx, gy, gz)) {
    const float gyroMagnitude = sqrtf((gx * gx) + (gy * gy) + (gz * gz));
    float rawRPM = gyroMagnitude / 6.0f;

    if (isnan(rawRPM) || rawRPM < kMinimumValidRpm) {
      rawRPM = 0.0f;
    }

    if (!rpmFilterInitialized) {
      filteredRPM = rawRPM;
      rpmFilterInitialized = true;
    } else {
      filteredRPM =
          (kRpmFilterAlpha * rawRPM) + ((1.0f - kRpmFilterAlpha) * filteredRPM);
    }

    lastFreshImuSampleMs = millis();
    return filteredRPM;
  }

  if (rpmFilterInitialized && (millis() - lastFreshImuSampleMs) <= kRpmStaleTimeoutMs) {
    return filteredRPM;
  }

  resetRpmFilter();
  return 0.0f;
}
