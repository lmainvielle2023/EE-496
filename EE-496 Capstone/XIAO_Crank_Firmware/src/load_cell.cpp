#include "load_cell.h"
#include "crank_pins.h"
#include <HX711.h>
#include <kvstore_global_api.h>
#include <math.h>

namespace {

HX711 scale;

// ============================================================
// CALIBRATION - Adjust with a known weight
//   1. Upload firmware, open serial monitor
//   2. Ensure the pedal is unloaded and send 't' to zero it
//   3. Send 'c' to enter calibration mode
//   4. Place a known weight (e.g. 5 lbs) on the load cell
//   5. Send '+'/'-' for coarse adjust (±100)
//   6. Send 'f'/'g' for fine adjust (±10)
//   7. Send 's' to save the calibration factor to flash
// ============================================================
float calibrationFactor = 2280.0f;

constexpr const char *kCalibrationFactorKey = "/kv/load_cell_cal_factor";
constexpr float kForceFilterAlpha = 0.25f;
constexpr float kZeroDeadbandLbs = 0.30f;
constexpr float kMaxZeroMeanLbs = 1.00f;
constexpr float kMaxZeroRangeLbs = 0.50f;
constexpr unsigned long kStartupSettleMs = 3000;
constexpr unsigned long kReadTimeoutMs = 2;
constexpr unsigned long kStaleSampleTimeoutMs = 350;
constexpr unsigned long kZeroSampleTimeoutMs = 250;
constexpr unsigned long kInterSampleDelayMs = 20;
constexpr uint8_t kDiscardedStartupSamples = 10;
constexpr uint8_t kZeroValidationSamples = 8;
constexpr uint8_t kTareSamples = 25;
constexpr bool kLoadCellDebug = false;

float filteredForceLbs = 0.0f;
bool filterInitialized = false;
bool calibrationMode = false;
unsigned long lastFreshSampleMs = 0;
unsigned long lastDebugPrintMs = 0;

void resetForceFilter() {
  filteredForceLbs = 0.0f;
  filterInitialized = false;
}

bool calibrationFactorLooksValid(float factor) {
  return isfinite(factor) && fabsf(factor) >= 1.0f && fabsf(factor) <= 1000000.0f;
}

void printCalibrationFactor() {
  Serial.print("Calibration Factor: ");
  Serial.println(calibrationFactor, 3);
}

void loadSavedCalibrationFactor() {
  float savedFactor = 0.0f;
  size_t actualSize = 0;

  const int result = kv_get(kCalibrationFactorKey, &savedFactor, sizeof(savedFactor), &actualSize);
  if (result == 0 && actualSize == sizeof(savedFactor) && calibrationFactorLooksValid(savedFactor)) {
    calibrationFactor = savedFactor;
    Serial.print("Loaded saved calibration factor: ");
    Serial.println(calibrationFactor, 3);
    return;
  }

  Serial.print("Using default calibration factor: ");
  Serial.println(calibrationFactor, 3);
}

void saveCalibrationFactor() {
  const int result = kv_set(kCalibrationFactorKey, &calibrationFactor, sizeof(calibrationFactor), 0);
  if (result == 0) {
    Serial.print("Saved calibration factor: ");
    Serial.println(calibrationFactor, 3);
  } else {
    Serial.print("Calibration save failed, error: ");
    Serial.println(result);
    Serial.println("Use the printed factor to update load_cell.cpp manually if needed.");
  }
}

void clearSavedCalibrationFactor() {
  const int result = kv_remove(kCalibrationFactorKey);
  if (result == 0) {
    Serial.println("Saved calibration factor cleared.");
  } else {
    Serial.print("Clear saved calibration failed, error: ");
    Serial.println(result);
  }
}

float publishForce(float filteredForce) {
  if (fabsf(filteredForce) <= kZeroDeadbandLbs) {
    return 0.0f;
  }

  return filteredForce > 0.0f ? filteredForce : 0.0f;
}

bool collectStableZeroWindow(float &meanForceLbs, float &rangeForceLbs) {
  float minForce = 0.0f;
  float maxForce = 0.0f;
  float forceSum = 0.0f;

  for (uint8_t i = 0; i < kZeroValidationSamples; ++i) {
    if (!scale.wait_ready_timeout(kZeroSampleTimeoutMs)) {
      return false;
    }

    const float sampleForce = scale.get_units(1);
    if (i == 0) {
      minForce = sampleForce;
      maxForce = sampleForce;
    } else {
      minForce = min(minForce, sampleForce);
      maxForce = max(maxForce, sampleForce);
    }

    forceSum += sampleForce;
    delay(kInterSampleDelayMs);
  }

  meanForceLbs = forceSum / kZeroValidationSamples;
  rangeForceLbs = maxForce - minForce;
  return true;
}

bool zeroScaleIfStable(const char *context) {
  float meanForceLbs = 0.0f;
  float rangeForceLbs = 0.0f;

  Serial.print("Checking load cell stability for ");
  Serial.print(context);
  Serial.println(" zero...");

  if (!collectStableZeroWindow(meanForceLbs, rangeForceLbs)) {
    Serial.println("Zero skipped: HX711 did not provide enough samples.");
    return false;
  }

  if (fabsf(meanForceLbs) > kMaxZeroMeanLbs || rangeForceLbs > kMaxZeroRangeLbs) {
    Serial.print("Zero skipped: mean=");
    Serial.print(meanForceLbs, 2);
    Serial.print(" lbs, range=");
    Serial.print(rangeForceLbs, 2);
    Serial.println(" lbs. Unload pedal and try again.");
    return false;
  }

  scale.tare(kTareSamples);
  resetForceFilter();
  lastFreshSampleMs = millis();

  Serial.print("Zero complete. Offset: ");
  Serial.println(scale.get_offset());
  return true;
}

bool forceZeroScale(const char *context) {
  Serial.print("Forcing ");
  Serial.print(context);
  Serial.println(" zero. Keep the pedal unloaded...");

  if (!scale.wait_ready_timeout(kZeroSampleTimeoutMs)) {
    Serial.println("Zero skipped: HX711 not ready.");
    return false;
  }

  scale.tare(kTareSamples);
  resetForceFilter();
  lastFreshSampleMs = millis();

  Serial.print("Zero complete. Offset: ");
  Serial.println(scale.get_offset());
  return true;
}

void maybePrintDebug(float rawForceLbs, float filteredForce, float publishedForce) {
  if (!kLoadCellDebug) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastDebugPrintMs < 250) {
    return;
  }

  lastDebugPrintMs = now;
  Serial.print("[LOAD] Raw: ");
  Serial.print(rawForceLbs, 2);
  Serial.print(" | Filtered: ");
  Serial.print(filteredForce, 2);
  Serial.print(" | Published: ");
  Serial.println(publishedForce, 2);
}

void checkCalibrationInput() {
  while (Serial.available()) {
    const char c = Serial.read();

    if (c == 'c' || c == 'C') {
      calibrationMode = !calibrationMode;
      Serial.println(calibrationMode ? "\n=== CALIBRATION MODE ON ==="
                                       "\nPlace a known weight on the sensor."
                                       "\n  '+'/'-' = coarse adjust (±100)"
                                       "\n  'f'/'g' = fine adjust (±10)"
                                       "\n  's' = save factor to flash"
                                       "\n  'p' = print current factor"
                                       "\nSend 'c' to exit calibration mode.\n"
                                     : "\n=== CALIBRATION MODE OFF ===\n");
    }

    if (c == 't' || c == 'T') {
      forceZeroScale("manual");
    }

    if (c == 'z' || c == 'Z') {
      zeroScaleIfStable("manual stable");
    }

    if (c == 'p' || c == 'P') {
      printCalibrationFactor();
    }

    if (c == 's' || c == 'S') {
      saveCalibrationFactor();
    }

    if (c == 'r' || c == 'R') {
      clearSavedCalibrationFactor();
    }

    if (!calibrationMode) {
      continue;
    }

    bool changed = false;
    if (c == '+') {
      calibrationFactor += 100.0f;
      changed = true;
    }
    if (c == '-') {
      calibrationFactor -= 100.0f;
      changed = true;
    }
    if (c == 'f') {
      calibrationFactor += 10.0f;
      changed = true;
    }
    if (c == 'g') {
      calibrationFactor -= 10.0f;
      changed = true;
    }

    if (changed) {
      scale.set_scale(calibrationFactor);
      resetForceFilter();
      printCalibrationFactor();
    }
  }
}

} // namespace

void initLoadCell() {
  Serial.println("Initializing HX711 Load Cell...");
  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.println("Waiting for HX711 to stabilize...");
  delay(kStartupSettleMs);

  Serial.println("Discarding startup readings...");
  for (uint8_t i = 0; i < kDiscardedStartupSamples; ++i) {
    if (scale.wait_ready_timeout(kZeroSampleTimeoutMs)) {
      scale.read();
    }
    delay(kInterSampleDelayMs);
  }

  loadSavedCalibrationFactor();
  scale.set_scale(calibrationFactor);

  if (!zeroScaleIfStable("startup")) {
    Serial.println("Startup zero skipped. Send 't' when the pedal is unloaded.");
  }

  Serial.println("Load Cell initialization complete.");
  Serial.println(">>> Send 't' to force zero | 'z' for stable zero | 'c' for calibration | 's' to save factor <<<");
}

float getPedalForce() {
  checkCalibrationInput();

  const unsigned long now = millis();
  if (scale.wait_ready_timeout(kReadTimeoutMs)) {
    const float rawForceLbs = scale.get_units(1);

    if (!filterInitialized) {
      filteredForceLbs = rawForceLbs;
      filterInitialized = true;
    } else {
      filteredForceLbs = (kForceFilterAlpha * rawForceLbs) +
                         ((1.0f - kForceFilterAlpha) * filteredForceLbs);
    }

    lastFreshSampleMs = now;

    const float publishedForce =
        calibrationMode ? filteredForceLbs : publishForce(filteredForceLbs);
    maybePrintDebug(rawForceLbs, filteredForceLbs, publishedForce);
    return publishedForce;
  }

  if (filterInitialized && (now - lastFreshSampleMs) <= kStaleSampleTimeoutMs) {
    return calibrationMode ? filteredForceLbs : publishForce(filteredForceLbs);
  }

  resetForceFilter();
  return 0.0f;
}
