#include "power_calc.h"
#include "ble_central.h" // For riderWatts global

namespace {

constexpr float CRANK_LENGTH_M = 0.170f;
constexpr float LBS_TO_NEWTONS = 4.44822f;
constexpr unsigned long FORCE_STALE_MS = 400;
constexpr unsigned long RPM_STALE_MS = 400;
constexpr unsigned long POWER_LOG_INTERVAL_MS = 1000;

float currentRegularForceLbs = 0.0f;
float currentSenseForceLbs = 0.0f;
float currentRPM = 0.0f;
unsigned long lastRegularForceMs = 0;
unsigned long lastSenseForceMs = 0;
unsigned long lastRPMMs = 0;
unsigned long lastPowerLogMs = 0;

float freshOrZero(float value, unsigned long sampleTimeMs, unsigned long timeoutMs) {
    if (sampleTimeMs == 0) {
        return 0.0f;
    }

    return (millis() - sampleTimeMs) <= timeoutMs ? value : 0.0f;
}

} // namespace

void initPowerCalc() {
    currentRegularForceLbs = 0.0f;
    currentSenseForceLbs = 0.0f;
    currentRPM = 0.0f;
    lastRegularForceMs = 0;
    lastSenseForceMs = 0;
    lastRPMMs = 0;
    lastPowerLogMs = 0;
}

void addRegularForce(float force) {
    currentRegularForceLbs = max(force, 0.0f);
    lastRegularForceMs = millis();
}

void addSenseForce(float force) {
    currentSenseForceLbs = max(force, 0.0f);
    lastSenseForceMs = millis();
}

void setRPM(float r) {
    currentRPM = max(r, 0.0f);
    lastRPMMs = millis();
}

float getCurrentRPM() {
    return currentRPM;
}

void updatePowerCalc() {
    const float regularForceLbs = freshOrZero(currentRegularForceLbs, lastRegularForceMs, FORCE_STALE_MS);
    const float senseForceLbs = freshOrZero(currentSenseForceLbs, lastSenseForceMs, FORCE_STALE_MS);
    const float rpm = freshOrZero(currentRPM, lastRPMMs, RPM_STALE_MS);

    // Assumes each load cell has been calibrated to the tangential pedal force
    // that produces crank torque.
    const float totalForceNewtons = (regularForceLbs + senseForceLbs) * LBS_TO_NEWTONS;
    float torqueNm = totalForceNewtons * CRANK_LENGTH_M;
    float angVel = rpm * (2.0f * PI / 60.0f);
    
    // Write to the global used by Motor Control
    riderWatts = (double)(torqueNm * angVel);

    const unsigned long now = millis();
    if (now - lastPowerLogMs >= POWER_LOG_INTERVAL_MS) {
        lastPowerLogMs = now;
        Serial.print("Regular[");
        Serial.print(isRegularCrankConnected() ? "OK" : "DISC");
        Serial.print("]: ");
        Serial.print(regularForceLbs, 2);
        Serial.print(" lbs | Sense[");
        Serial.print(isSenseCrankConnected() ? "OK" : "DISC");
        Serial.print("]: ");
        Serial.print(senseForceLbs, 2);
        Serial.print(" lbs | RPM: ");
        Serial.print(rpm, 1);
        Serial.print(" | Goal[");
        Serial.print(isGoalNodeConnected() ? "OK" : "DISC");
        Serial.print("]: ");
        Serial.print(getTargetGoalWatts(), 1);
        Serial.print(" | Rider Watts: ");
        Serial.println(riderWatts, 1);
    }
}
