#include "load_cell.h"
#include "crank_pins.h"
#include <HX711.h>

HX711 scale;

// ============================================================
// CALIBRATION - Adjust with a known weight
//   1. Upload firmware, open serial monitor
//   2. Send 'c' to enter calibration mode
//   3. Place a known weight (e.g. 5 lbs) on the load cell
//   4. Send '+'/'-' for coarse adjust (±100)
//   5. Send 'f'/'g' for fine adjust (±10)
//   6. Note the printed calibration factor and update below
//   7. Note the printed zero offset and update below
// ============================================================
float CALIBRATION_FACTOR = 2280.0f;

// Set to 0 to force a live tare on boot.
// Once you have a stable offset, hardcode it here for consistency.
const long KNOWN_ZERO_OFFSET = 0;

// Filtering
const float EMA_ALPHA = 0.3f;
float filteredForce = 0.0f;
bool filterInitialized = false;

// Calibration mode flag
bool calibrationMode = false;

void initLoadCell() {
  Serial.println("Initializing HX711 Load Cell...");
  scale.begin(HX711_DOUT, HX711_SCK);

  // --- Critical: let the HX711 fully stabilize ---
  Serial.println("Waiting for HX711 to stabilize...");
  delay(3000);

  // Flush the first unstable readings
  Serial.println("Flushing initial readings...");
  for (int i = 0; i < 10; i++) {
    if (scale.is_ready()) {
      scale.read();
    }
    delay(100);
  }

  scale.set_scale(CALIBRATION_FACTOR);

  if (KNOWN_ZERO_OFFSET != 0) {
    // Use a known, stable offset — no dependency on boot conditions
    scale.set_offset(KNOWN_ZERO_OFFSET);
    Serial.print("Using hardcoded zero offset: ");
    Serial.println(KNOWN_ZERO_OFFSET);
  } else {
    // Live tare — make sure load cell is TRULY unloaded!
    Serial.println(">> ENSURE LOAD CELL IS UNLOADED FOR TARE <<");
    delay(1000);
    scale.tare(30); // Average 30 readings for a more stable zero
    Serial.print("Live tare complete. Zero offset: ");
    Serial.println(scale.get_offset());
    Serial.println(
        ">> Save this offset value to KNOWN_ZERO_OFFSET for consistency <<");
  }

  Serial.println("Load Cell initialization complete.");
  Serial.println(">>> Send 'c' for calibration | 't' to re-tare <<<");
}

void checkCalibrationInput() {
  if (Serial.available()) {
    char c = Serial.read();

    // Toggle calibration mode
    if (c == 'c' || c == 'C') {
      calibrationMode = !calibrationMode;
      Serial.println(calibrationMode ? "\n=== CALIBRATION MODE ON ==="
                                       "\nPlace a known weight on the sensor."
                                       "\n  '+'/'-' = coarse adjust (±100)"
                                       "\n  'f'/'g' = fine adjust (±10)"
                                       "\nSend 'c' to exit calibration mode.\n"
                                     : "\n=== CALIBRATION MODE OFF ===\n");
    }

    // Manual re-tare command
    if (c == 't' || c == 'T') {
      Serial.println("\n>> Re-taring... ensure load cell is unloaded! <<");
      delay(500);
      scale.tare(30);
      filteredForce = 0.0f;
      filterInitialized = false;
      Serial.print("New zero offset: ");
      Serial.println(scale.get_offset());
    }

    // Calibration adjustments
    if (calibrationMode) {
      bool changed = false;
      if (c == '+') {
        CALIBRATION_FACTOR += 100.0f;
        changed = true;
      }
      if (c == '-') {
        CALIBRATION_FACTOR -= 100.0f;
        changed = true;
      }
      if (c == 'f') {
        CALIBRATION_FACTOR += 10.0f;
        changed = true;
      }
      if (c == 'g') {
        CALIBRATION_FACTOR -= 10.0f;
        changed = true;
      }

      if (changed) {
        scale.set_scale(CALIBRATION_FACTOR);
        Serial.print("Calibration Factor: ");
        Serial.println(CALIBRATION_FACTOR);
      }
    }
  }
}

float getPedalForce() {
  checkCalibrationInput();

  if (scale.is_ready()) {
    float raw = scale.get_units(5);

    // Only clamp negatives in production mode —
    // in calibration, negative values are diagnostic
    if (!calibrationMode && raw < 0.0f) {
      raw = 0.0f;
    }

    // Exponential Moving Average filter
    if (!filterInitialized) {
      filteredForce = raw;
      filterInitialized = true;
    } else {
      filteredForce = (EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * filteredForce);
    }

    Serial.print("[DEBUG] Raw: ");
    Serial.print(raw, 2);
    Serial.print(" | Filtered: ");
    Serial.println(filteredForce, 2);

    return filteredForce;
  }
  return filteredForce;
}
