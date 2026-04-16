# EE-496 Capstone: Scaled E-Bike System Architecture & Setup Guide

This document provides a comprehensive overview of the dual-crank smart e-bike prototype, detailing the network topology, hardware wiring, and firmware setup required for the presentation loop.

---

## 1. Network Topology Overview
The system utilizes a distributed `Star` topology operating over **Bluetooth Low Energy (BLE)**.

*   **1 Central Node (ESP32):** Manages all mathematics, acts as a BLE Central Hub concurrently digesting data streams, tracks GPS via Serial UART, and commands the L298N PWM.
*   **2 Peripheral Nodes (Seeed XIAO):** One on the left pedal, one on the right. They act purely as data-gathering broadcasters.

---

## 2. Hardware Wiring Guide

### A. The Crank Sensor Nodes (Seeed XIAO nRF52840)
You will flash the **Left Pedal** with a standard Seeed XIAO nRF52840, and the **Right Pedal** with the Seeed XIAO nRF52840 **Sense** (because the Sense variant contains the built-in LSM6DS3 IMU required for RPM tracking).

**Both Nodes require an HX711 Load Cell Amplifier:**
*   **HX711 DOUT** ➔ XIAO Pad **D2**
*   **HX711 SCK** ➔ XIAO Pad **D3**
*   **HX711 VCC** ➔ XIAO 3.3V
*   **HX711 GND** ➔ XIAO GND

### B. The Main Controller (ESP32)
The central hub coordinates the L298N Motor driver, the Rotary Encoder (for measuring actual motor speed), and the NEO-6M GPS modules.

**Encoder Sensors (Wheel Speed / Motor Speed):**
*   **Encoder Phase A** ➔ ESP32 Pin **32** *(Requires external/internal pullup)*
*   **Encoder Phase B** ➔ ESP32 Pin **33** *(Requires external/internal pullup)*

**L298N Motor Controller (Drive):**
*   **ENA (PWM Speed)** ➔ ESP32 Pin **14**
*   **IN1 (Direction 1)** ➔ ESP32 Pin **27**
*   **IN2 (Direction 2)** ➔ ESP32 Pin **26**

**U-blox NEO-6M (GPS Module):**
*   **TX Pin** ➔ ESP32 Pin **16** (RX2)
*   **RX Pin** ➔ ESP32 Pin **17** (TX2)

---

## 3. Firmware Flashing & Configuration

Because the Left and Right pedals use the exact same code logic in slightly different ways, they share a single unified PlatformIO project to prevent you from managing two codebases!

### A. Flashing the Right Pedal (The "Sense" Node)
1. Open `/XIAO_Crank_Firmware/src/main.cpp`.
2. Ensure the macro is set to true: `#define IS_RIGHT_NODE true`
3. Plug in the Seeed Sense board via USB-C.
4. Flash the code via PlatformIO:
   ```bash
   cd "XIAO_Crank_Firmware"
   pio run -t upload
   ```
5. *What it does:* Broadcasts BLE as `CRANK_RIGHT`. Initializes the `HX711` on D2/D3. Internally powers on the `LSM6DS3` IMU on the hidden `Wire1` bus (Address 0x6A). Transmits Force & RPM.

### B. Flashing the Left Pedal (The Standard Node)
1. Open `/XIAO_Crank_Firmware/src/main.cpp`.
2. Change the macro to false: `#define IS_RIGHT_NODE false`
3. Plug in the standard Seeed XIAO board via USB-C.
4. Flash the code via PlatformIO!
5. *What it does:* Broadcasts BLE as `CRANK_LEFT`. Skips the IMU logic. Transmits Force only.

### C. Flashing the Main Controller (ESP32)
1. Ensure the XIAO board is unplugged, and plug in the ESP32.
2. Flash the code via PlatformIO:
   ```bash
   cd "ESP32_Main_Firmware"
   pio run -t upload
   ```
3. *What it does:* 
   * **Core 0:** Bootstraps the BLE Client, scans for `CRANK_RIGHT` and `CRANK_LEFT`, connects concurrently to both, and ingests `targetWatts` and `RPM`. Also handles high-latency Terrain prediction (HTTP/Wi-Fi).
   * **Core 1:** Fast loop. Runs the math `requiredMotorWatts = 200W - riderWatts` and directly manipulates the 8-bit PWM on Pin 14 to assist the rider proportionally.

---

## 4. Tuning the Physics (Future)

When you are ready to adjust the 200W goal or the PWM maximums for your Capstone demonstration, navigate to `ESP32_Main_Firmware/src/motor_control.cpp`.

```cpp
const float GOAL_WATTS = 200.0f;          // The total power the system attempts to maintain.
const float MAX_MOTOR_WATTS = 200.0f;     // The upper bound mapping for 255 PWM.
```
If you change `MAX_MOTOR_WATTS` to 100.0f, the motor will push twice as hard (hit 255 PWM) for the exact same `requiredMotorWatts` calculation!
