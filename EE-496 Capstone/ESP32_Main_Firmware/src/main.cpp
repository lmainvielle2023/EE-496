#include <Arduino.h>
#include "system_pins.h"
#include "motor_control.h"
#include "terrain_predict.h"
#include "ble_central.h"
#include "power_calc.h"

// Task handles
TaskHandle_t ManagerTaskHandle;
TaskHandle_t WorkerTaskHandle;

// Function prototypes for FreeRTOS tasks
void TaskCore0_Manager(void *pvParameters);
void TaskCore1_Worker(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial monitor time to attach
    Serial.println("Starting ESP32 E-Bike Controller...");

    // BLE must init before WiFi to claim contiguous RAM for BT controller
    initMotorControl();
    initBLECentral();
    initTerrainPredict();
    initPowerCalc();

    // Create Manager Task pinned to Core 0
    // Handles BLE, GPS, WiFi, and HTTP requests
    xTaskCreatePinnedToCore(
        TaskCore0_Manager,    // Task function
        "ManagerTask",        // Task name
        10000,                // Stack size (bytes in ESP-IDF!)
        NULL,                 // Task parameters
        1,                    // Priority
        &ManagerTaskHandle,   // Task handle
        0                     // Core 0
    );

    // Create Worker Task pinned to Core 1
    // Handles fast Motor Control and PID loop
    xTaskCreatePinnedToCore(
        TaskCore1_Worker,     // Task function
        "WorkerTask",         // Task name
        10000,                // Stack size
        NULL,                 // Task parameters
        configMAX_PRIORITIES - 1, // High Priority! (Real-time)
        &WorkerTaskHandle,    // Task handle
        1                     // Core 1
    );
}

void loop() {
    // The main loop is implicitly running on Core 1 by default, but FreeRTOS handles our xTasks.
    // We can just delay and let the RTOS tasks do their job, or use this to feed the watchdog.
    delay(1000);
}

// ---------------------------------------------------------------- //
// CORE 0: Network & Comm Manager
// ---------------------------------------------------------------- //
void TaskCore0_Manager(void *pvParameters) {
    Serial.println("Manager Task running on Core: " + String(xPortGetCoreID()));

    while (true) {
        // Run BLE Scan & Data Extraction
        updateBLECentral();

        // Run power math (averages and torque)
        updatePowerCalc();

        // Run GPS Parse & Terrain WiFi Prediction
        updateTerrainPredict();

        // Delay to yield to FreeRTOS idle task
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---------------------------------------------------------------- //
// CORE 1: Real-time Motor Control Worker
// ---------------------------------------------------------------- //
void TaskCore1_Worker(void *pvParameters) {
    Serial.println("Worker Task running on Core: " + String(xPortGetCoreID()));

    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz loop (10ms)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // Run PID calculation and adjust L298N PWM
        updateMotorControl();

        // Delay exactly up to the next 10ms boundary, maintaining strict 100Hz loop
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
