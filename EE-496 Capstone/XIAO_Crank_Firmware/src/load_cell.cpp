#include "load_cell.h"
#include "crank_pins.h"
#include <HX711.h>

HX711 scale;

// Calibration factor (placeholder value, needs to be tuned for the ShangHJ Kit)
float CALIBRATION_FACTOR = 2280.f;

// Physical Constants
const float CRANK_LENGTH_M = 0.170f;       // 170mm standard crank arm
const float LBS_TO_NEWTONS = 4.44822f;     // Newton conversion parameter

void initLoadCell() {
    Serial.println("Initializing HX711 Load Cell...");
    scale.begin(HX711_DOUT, HX711_SCK);

    scale.set_scale(CALIBRATION_FACTOR);
    scale.tare(); // Reset scale to 0

    Serial.println("Load Cell initialization complete. Tared to 0.");
}

float getPedalForce() {
    if (scale.is_ready()) {
        float force_lbs = scale.get_units(1); // average of 1 reading to keep it fast
        // Constrain any negative drift to 0
        if (force_lbs < 0.0f) {
            force_lbs = 0.0f;
        }
        return force_lbs;
    }
    return 0.0f;
}

float calculateWatts(float force_lbs, float rpm) {
    if (rpm <= 0.0f || force_lbs <= 0.0f) {
        return 0.0f;
    }
    
    // 1. Convert Input Force to Newtons
    float force_newtons = force_lbs * LBS_TO_NEWTONS;
    
    // 2. Calculate Torque in Nm
    // Assuming Force is applied completely tangentially for this prototype step
    float torque_nm = force_newtons * CRANK_LENGTH_M;
    
    // 3. Calculate Power (Watts) = Torque * Angular Velocity
    // Angular Velocity (rad/s) = RPM * (2 * PI / 60)
    float angular_velocity = rpm * (2.0f * PI / 60.0f);
    float power_watts = torque_nm * angular_velocity;
    
    return power_watts;
}
