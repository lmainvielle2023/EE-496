#include "power_calc.h"
#include "ble_central.h" // For riderWatts global

#define BUFFER_SIZE 20 // Approx 2 seconds of data at 10Hz BLE update rate
const float CRANK_LENGTH_M = 0.170f;
const float LBS_TO_NEWTONS = 4.44822f;

static float leftBuffer[BUFFER_SIZE];
static float rightBuffer[BUFFER_SIZE];
static int leftIdx = 0;
static int rightIdx = 0;

static float currentRPM = 0.0f;

void initPowerCalc() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        leftBuffer[i] = 0.0f;
        rightBuffer[i] = 0.0f;
    }
}

void addLeftForce(float force) {
    leftBuffer[leftIdx] = force;
    leftIdx = (leftIdx + 1) % BUFFER_SIZE;
}

void addRightForce(float force) {
    rightBuffer[rightIdx] = force;
    rightIdx = (rightIdx + 1) % BUFFER_SIZE;
}

void setRPM(float r) {
    currentRPM = r;
}

void updatePowerCalc() {
    float leftSum = 0;
    float rightSum = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        leftSum += leftBuffer[i];
        rightSum += rightBuffer[i];
    }
    
    float avgLeft = leftSum / BUFFER_SIZE;
    float avgRight = rightSum / BUFFER_SIZE;
    
    float totalForceNewtons = (avgLeft + avgRight) * LBS_TO_NEWTONS;
    float torqueNm = totalForceNewtons * CRANK_LENGTH_M;
    float angVel = currentRPM * (2.0f * PI / 60.0f);
    
    // Write to the global used by Motor Control
    riderWatts = (double)(torqueNm * angVel);
    
    /* 
    // Optional debug printing, can be noisy
    Serial.print("Avg Left: "); Serial.print(avgLeft);
    Serial.print(" | Avg Right: "); Serial.print(avgRight);
    Serial.print(" | RPM: "); Serial.print(currentRPM);
    Serial.print(" -> Rider Watts: "); Serial.println(riderWatts);
    */
}
