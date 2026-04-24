#include "imu_rpm.h"
#include <Wire.h>
#include <math.h>
#include <hal/nrf_gpio.h> // Force hardware-level GPIO control

namespace {

// The actual hardware pin for the IMU LDO on the Seeed XIAO nRF52840 Sense
// is Port 1, Pin 8 (P1.08). We use the Nordic HAL directly to avoid 
// Arduino pin mapping/PlatformIO environment mismatch issues.
const uint32_t kImuPowerPin = NRF_GPIO_PIN_MAP(1, 8);

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
constexpr unsigned long kImuDebugIntervalMs = 500;
constexpr bool kImuDebug = true;

bool imuInitialized = false;
TwoWire *activeImuWire = nullptr;
uint8_t activeImuAddress = 0x6A;
float filteredRPM = 0.0f;
bool rpmFilterInitialized = false;
unsigned long lastFreshImuSampleMs = 0;
unsigned long lastImuDebugMs = 0;

void resetRpmFilter() {
  filteredRPM = 0.0f;
  rpmFilterInitialized = false;
}

bool readRegistersFrom(TwoWire &wire, uint8_t address, uint8_t startRegister,
                       uint8_t *buffer, size_t length) {
  wire.beginTransmission(address);
  wire.write(startRegister);
  
  // Seeed's official library uses a standard STOP condition, so we will too.
  // This correctly checks if the IMU is actually powered and ACKing.
  if (wire.endTransmission() != 0) {
    return false;
  }

  if (wire.requestFrom(address, static_cast<uint8_t>(length)) != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = wire.read();
  }

  return true;
}

bool readRegisters(uint8_t startRegister, uint8_t *buffer, size_t length) {
  if (activeImuWire == nullptr) {
    return false;
  }
  return readRegistersFrom(*activeImuWire, activeImuAddress, startRegister, buffer, length);
}

bool writeRegisterTo(TwoWire &wire, uint8_t address, uint8_t reg, uint8_t value) {
  wire.beginTransmission(address);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  if (activeImuWire == nullptr) {
    return false;
  }
  return writeRegisterTo(*activeImuWire, activeImuAddress, reg, value);
}

bool tryDetectImu(TwoWire &wire, const char *wireName, uint8_t address) {
  uint8_t whoAmI = 0;
  const bool ok = readRegistersFrom(wire, address, kWhoAmIRegister, &whoAmI, 1);

  Serial.print("IMU probe ");
  Serial.print(wireName);
  Serial.print(" addr 0x");
  Serial.print(address, HEX);
  Serial.print(" -> ");
  if (!ok) {
    Serial.println("no response");
    return false;
  }

  Serial.print("WHO_AM_I 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == kExpectedWhoAmIPrimary || whoAmI == kExpectedWhoAmIAlternate) {
    activeImuWire = &wire;
    activeImuAddress = address;
    return true;
  }

  return false;
}

bool detectImuOnAnyBus() {
  Wire.begin();
  Wire1.begin();

  const uint8_t addresses[] = {0x6A, 0x6B};

  for (uint8_t address : addresses) {
    if (tryDetectImu(Wire1, "Wire1", address)) {
      return true;
    }
  }

  for (uint8_t address : addresses) {
    if (tryDetectImu(Wire, "Wire", address)) {
      return true;
    }
  }

  return false;
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

  nrf_gpio_cfg_output(kImuPowerPin);
  nrf_gpio_pin_set(kImuPowerPin);

  // FIX: Allow up to 1 full second for the IMU to boot on battery power
  bool imuFound = false;
  for (int i = 0; i < 10; i++) {
    delay(100);
    if (detectImuOnAnyBus()) {
      imuFound = true;
      break;
    }
  }

  if (!imuFound) {
    Serial.println("IMU Error: Device not found on Wire/Wire1 at 0x6A/0x6B.");
    imuInitialized = false;
    return;
  }

  Serial.print("IMU selected address 0x");
  Serial.println(activeImuAddress, HEX);

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
    float rawRPM = gyroMagnitude / 6.0f; // 1 RPM = 6 degrees per second

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

    if (kImuDebug && (millis() - lastImuDebugMs >= kImuDebugIntervalMs)) {
      lastImuDebugMs = millis();
      Serial.print("[IMU] gx=");
      Serial.print(gx, 2);
      Serial.print(" gy=");
      Serial.print(gy, 2);
      Serial.print(" gz=");
      Serial.print(gz, 2);
      Serial.print(" dps | rawRPM=");
      Serial.print(gyroMagnitude / 6.0f, 1);
      Serial.print(" | filteredRPM=");
      Serial.println(filteredRPM, 1);
    }

    return filteredRPM;
  }

  if (kImuDebug && (millis() - lastImuDebugMs >= kImuDebugIntervalMs)) {
    lastImuDebugMs = millis();
    Serial.println("[IMU] Gyro read failed.");
  }

  if (rpmFilterInitialized && (millis() - lastFreshImuSampleMs) <= kRpmStaleTimeoutMs) {
    return filteredRPM;
  }

  resetRpmFilter();
  return 0.0f;
}