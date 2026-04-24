#include "motor_control.h"
#include "system_pins.h"
#include "ble_central.h" // Gives access to riderWatts global

// Global Pulses
volatile long encoderPulses = 0;

// Setup constants
const float MAX_MOTOR_WATTS = 75.0f; // Scale reference for 75 W demo goal

// IRAM_ATTR places the routine into Internal RAM for fast execution on ESP32
void IRAM_ATTR encoderISR() {
    int phaseB = digitalRead(ENCODER_B_PIN);
    if (phaseB == HIGH) {
        encoderPulses++;
    } else {
        encoderPulses--;
    }
}

void initMotorControl() {
    Serial.println("Initializing Motor Control...");

    // Setup TB6612FNG channel A output pins
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
#ifdef MOTOR_STBY_PIN
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STBY_PIN, HIGH);
#endif

    // Initial stop
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, 0);

    // Setup Encoder Input Pins with Internal Pullups
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    // Attach hardware interrupt for Phase A RISING edge
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

    Serial.println("Motor Control Initialized.");
}

// Variables for RPM calculation (useful for future speed limiting)
unsigned long lastRPMCalcTime = 0;
long lastEncoderPulses = 0;
const double GEAR_RATIO = 34.0;
const double PULSES_PER_REV = 11.0;
const double TOTAL_PPR = GEAR_RATIO * PULSES_PER_REV;
double currentMotorRPM = 0;

void updateMotorControl() {
    // 1. Calculate the motor power required to reach the external goal
    float requiredMotorWatts = (float)targetGoalWatts - (float)riderWatts;
    
    // Clamp to 0 (motor doesn't fight rider or brake them)
    if (requiredMotorWatts < 0.0f) {
        requiredMotorWatts = 0.0f;
    }
    
    // 2. Map required watts to PWM (Open Loop Feed-Forward)
    // Prototype Mapping: PWM = 255 * (req / MAX_W)
    float pwmRaw = 255.0f * (requiredMotorWatts / MAX_MOTOR_WATTS);
    
    // Clamp PWM to 8-bit safety limits
    if (pwmRaw > 255.0f) pwmRaw = 255.0f;
    if (pwmRaw < 0.0f) pwmRaw = 0.0f;
    
    int pwmOutput = (int)pwmRaw;
    
    // 3. Keep encoder logic active just in case we need RPM data for future features
    noInterrupts();
    long currentPulses = encoderPulses;
    interrupts();

    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - lastRPMCalcTime;

    if (elapsedTime > 0) {
        long deltaPulses = currentPulses - lastEncoderPulses;
        currentMotorRPM = ((double)deltaPulses / TOTAL_PPR) / ((double)elapsedTime / 60000000.0);
        lastEncoderPulses = currentPulses;
        lastRPMCalcTime = currentTime;
    }

    // 4. Set Motor Driver
    if (pwmOutput > 0) {
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, LOW); // Forward
    } else {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, LOW); // Coast
    }
    
    analogWrite(MOTOR_ENA_PIN, pwmOutput);
}
