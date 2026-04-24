#ifndef SYSTEM_PINS_H
#define SYSTEM_PINS_H

// L298N Motor Driver Pins, channel A
#define MOTOR_PWM_PIN 14  // ENA, remove ENA jumper and connect this pin
#define MOTOR_IN1_PIN 26  // IN1
#define MOTOR_IN2_PIN 27  // IN2
// If STBY is connected to an ESP32 GPIO, define MOTOR_STBY_PIN here.
// If STBY is tied to 3.3V, leave this undefined.
// #define MOTOR_STBY_PIN 25

// Backward-compatible alias for older motor-control code.
#define MOTOR_ENA_PIN MOTOR_PWM_PIN

// Motor Encoder Pins
#define ENCODER_A_PIN 32 // Phase A Interrupt
#define ENCODER_B_PIN 33 // Phase B Interrupt

#endif // SYSTEM_PINS_H
