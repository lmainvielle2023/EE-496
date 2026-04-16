#include "load_cell.h"
#include "crank_pins.h"
#include <HX711.h>

HX711 scale;

// ============================================================
// CALIBRATION FACTOR - Adjust this with a known weight!
// How to calibrate:
//   1. Upload firmware, open serial monitor
//   2. Send 'c' to enter calibration mode
//   3. Place a known weight (e.g. 5 lbs) on the load cell
//   4. Send '+' or '-' to adjust until the reading matches
//   5. Note the printed calibration factor and update this value
// ============================================================
float CALIBRATION_FACTOR = 2280.0f;

// Filtering
const float EMA_ALPHA = 0.3f;              // Smoothing factor (0.0-1.0, lower = smoother)
float filteredForce = 0.0f;
bool filterInitialized = false;

// Calibration mode flag
bool calibrationMode = false;

void initLoadCell() {
    Serial.println("Initializing HX711 Load Cell...");
    scale.begin(HX711_DOUT, HX711_SCK);

    // Wait for HX711 to stabilize after power-on
    Serial.println("Waiting for HX711 to stabilize...");
    delay(2000);

    scale.set_scale(CALIBRATION_FACTOR);
    scale.tare(20); // Average 20 readings for a more stable zero

    Serial.print("Load Cell tared. Zero offset: ");
    Serial.println(scale.get_offset());
    Serial.println("Load Cell initialization complete.");
    Serial.println(">>> Send 'c' to enter calibration mode <<<");
}

void checkCalibrationInput() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'c' || c == 'C') {
            calibrationMode = !calibrationMode;
            Serial.println(calibrationMode ? 
                "\n=== CALIBRATION MODE ON ===" 
                "\nPlace a known weight on the sensor."
                "\nSend '+' to increase factor, '-' to decrease."
                "\nSend 'c' to exit calibration mode.\n" :
                "\n=== CALIBRATION MODE OFF ===\n");
        }
        if (calibrationMode) {
            if (c == '+') {
                CALIBRATION_FACTOR += 100.0f;
            } else if (c == '-') {
                CALIBRATION_FACTOR -= 100.0f;
            }
            if (c == '+' || c == '-') {
                scale.set_scale(CALIBRATION_FACTOR);
                Serial.print("Calibration Factor: ");
                Serial.println(CALIBRATION_FACTOR);
            }
        }
    }
}

float getPedalForce() {
    // Check for calibration commands
    checkCalibrationInput();

    if (scale.is_ready()) {
        float raw = scale.get_units(5); // Average 5 readings from HX711

        // Clamp negative drift to 0
        if (raw < 0.0f) {
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
    return filteredForce; // Return last filtered value if not ready
}

