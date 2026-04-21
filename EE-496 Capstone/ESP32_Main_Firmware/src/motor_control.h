#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Global shared variables (read by Manager, updated by Worker)
extern volatile long encoderPulses;

// Initialization routine
void initMotorControl();

// Core 1 Worker routine
void updateMotorControl();

// Hardware Interrupt Handlers
void IRAM_ATTR encoderISR();

#endif // MOTOR_CONTROL_H
