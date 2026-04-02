#include "motor_control.h"
#include "system_pins.h"
#include "pid_gains.h"
#include <PID_v1.h>

// Global Pulses
volatile long encoderPulses = 0;

// Variables for PID
double Setpoint = 0;   // Target RPM/Power
double Input = 0;      // Real RPM
double Output = 0;     // PWM adjustment

// Setup PID (Direct Action)
PID motorPID(&Input, &Output, &Setpoint, PID_KP, PID_KI, PID_KD, DIRECT);

// IRAM_ATTR places the routine into Internal RAM for fast execution on ESP32
void IRAM_ATTR encoderISR() {
    // Read Phase B to determine direction.
    // If Phase B is identically high when Phase A rises, they're spinning one way.
    int phaseB = digitalRead(ENCODER_B_PIN);
    if (phaseB == HIGH) {
        encoderPulses++;
    } else {
        encoderPulses--;
    }
}

void initMotorControl() {
    Serial.println("Initializing Motor Control...");

    // Setup L298N Output Pins
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    // Initial stop
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, 0);

    // Setup Encoder Input Pins with Internal Pullups
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    // Attach hardware interrupt for Phase A RISING edge
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

    // Initialize PID settings
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255); // 8-bit PWM maximum

    Serial.println("Motor Control Initialized.");
}

// Variables for RPM calculation
unsigned long lastRPMCalcTime = 0;
long lastEncoderPulses = 0;
const double GEAR_RATIO = 34.0;
const double PULSES_PER_REV = 11.0;
const double TOTAL_PPR = GEAR_RATIO * PULSES_PER_REV;

void updateMotorControl() {
    // Read the current Target Setpoint (usually fetched from Manager via BLE)
    // Read the current Terrain Resistance (calculated from GPS Slope)
    // Convert current encoder pulses into an RPM (Input)
    
    // Safely copy encoder pulses by temporarily disabling interrupts
    noInterrupts();
    long currentPulses = encoderPulses;
    interrupts();

    // Calculate accurate RPM based on time elapsed since last loop
    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - lastRPMCalcTime;

    if (elapsedTime > 0) {
        long deltaPulses = currentPulses - lastEncoderPulses;
        // Formula: (delta pulses / total pulses per revolution) / (elapsed minutes)
        // elapsedTime is in microseconds. To convert to minutes = elapsedTime / 60,000,000.0
        Input = ((double)deltaPulses / TOTAL_PPR) / ((double)elapsedTime / 60000000.0);
        
        lastEncoderPulses = currentPulses;
        lastRPMCalcTime = currentTime;
    }

    // Compute new PID
    motorPID.Compute();

    // Set Motor Driver logic (example forward logic with PID PWM)
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_ENA_PIN, (int)Output);
}
