#include "power_calc.h"
#include "ble_central.h" // For riderWatts global

namespace {

constexpr float CRANK_LENGTH_M = 0.170f;
constexpr float LBS_TO_NEWTONS = 4.44822f;
constexpr unsigned long FORCE_STALE_MS = 400;
constexpr unsigned long RPM_STALE_MS = 400;

float currentLeftForceLbs = 0.0f;
float currentRightForceLbs = 0.0f;
float currentRPM = 0.0f;
unsigned long lastLeftForceMs = 0;
unsigned long lastRightForceMs = 0;
unsigned long lastRPMMs = 0;

float freshOrZero(float value, unsigned long sampleTimeMs, unsigned long timeoutMs) {
    if (sampleTimeMs == 0) {
        return 0.0f;
    }

    return (millis() - sampleTimeMs) <= timeoutMs ? value : 0.0f;
}

} // namespace

void initPowerCalc() {
    currentLeftForceLbs = 0.0f;
    currentRightForceLbs = 0.0f;
    currentRPM = 0.0f;
    lastLeftForceMs = 0;
    lastRightForceMs = 0;
    lastRPMMs = 0;
}

void addLeftForce(float force) {
    currentLeftForceLbs = max(force, 0.0f);
    lastLeftForceMs = millis();
}

void addRightForce(float force) {
    currentRightForceLbs = max(force, 0.0f);
    lastRightForceMs = millis();
}

void setRPM(float r) {
    currentRPM = max(r, 0.0f);
    lastRPMMs = millis();
}

void updatePowerCalc() {
    const float leftForceLbs = freshOrZero(currentLeftForceLbs, lastLeftForceMs, FORCE_STALE_MS);
    const float rightForceLbs = freshOrZero(currentRightForceLbs, lastRightForceMs, FORCE_STALE_MS);
    const float rpm = freshOrZero(currentRPM, lastRPMMs, RPM_STALE_MS);

    // Assumes each load cell has been calibrated to the tangential pedal force
    // that produces crank torque.
    const float totalForceNewtons = (leftForceLbs + rightForceLbs) * LBS_TO_NEWTONS;
    float torqueNm = totalForceNewtons * CRANK_LENGTH_M;
    float angVel = rpm * (2.0f * PI / 60.0f);
    
    // Write to the global used by Motor Control
    riderWatts = (double)(torqueNm * angVel);
    
    /* 
    // Optional debug printing, can be noisy
    Serial.print("Left: "); Serial.print(leftForceLbs);
    Serial.print(" | Right: "); Serial.print(rightForceLbs);
    Serial.print(" | RPM: "); Serial.print(rpm);
    Serial.print(" -> Rider Watts: "); Serial.println(riderWatts);
    */
}
