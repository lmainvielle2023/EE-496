#include <Arduino.h>
#include "system_pins.h"

String serialLine;
const int PWM_LEVELS[] = {0, 64, 128, 191, 255, 191, 128, 64, 0};
const int PWM_LEVEL_COUNT = sizeof(PWM_LEVELS) / sizeof(PWM_LEVELS[0]);
const unsigned long STEP_MS = 3000;

void stopMotor() {
    analogWrite(MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
}

void motorForwardFullPower() {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, 255);
}

void motorForwardPwm(int pwm) {
    pwm = constrain(pwm, 0, 255);

    if (pwm == 0) {
        stopMotor();
        return;
    }

    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    analogWrite(MOTOR_PWM_PIN, pwm);
}

bool readSerialCommand(String &command) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            serialLine.trim();
            if (serialLine.length() > 0) {
                command = serialLine;
                serialLine = "";
                return true;
            }
            serialLine = "";
        } else {
            serialLine += c;
        }
    }

    return false;
}

void runSpinTest() {
    Serial.println("Starting speed ramp test.");
    Serial.println("Type Stop at any time to stop early.");

    for (int i = 0; i < PWM_LEVEL_COUNT; i++) {
        int pwm = PWM_LEVELS[i];
        int percent = (pwm * 100) / 255;

        Serial.print("PWM ");
        Serial.print(pwm);
        Serial.print(" (about ");
        Serial.print(percent);
        Serial.println("%)");

        motorForwardPwm(pwm);

        unsigned long start = millis();
        while (millis() - start < STEP_MS) {
            String command;
            if (readSerialCommand(command)) {
                command.toLowerCase();
                if (command == "stop" || command == "off") {
                    stopMotor();
                    Serial.println("Ramp stopped.");
                    return;
                }
            }
            delay(10);
        }
    }

    stopMotor();
    Serial.println("Ramp complete. Type Go to run again.");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    stopMotor();

    Serial.println("L298N Motor Spin Test Ready.");
    Serial.println("Pins: ENA=GPIO14, IN1=GPIO26, IN2=GPIO27.");
    Serial.println("Type Go and press Enter to run 0/25/50/75/100/75/50/25/0% PWM.");
}

void loop() {
    String command;

    if (readSerialCommand(command)) {
        command.toLowerCase();

        if (command == "go" || command == "on") {
            runSpinTest();
        } else if (command == "stop" || command == "off") {
            stopMotor();
            Serial.println("Motor stopped.");
        } else {
            Serial.println("Type Go to spin or Stop to stop.");
        }
    }

    delay(10);
}
