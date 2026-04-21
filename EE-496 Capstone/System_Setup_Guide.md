# EE-496 Capstone Setup And Run Guide

This guide covers the full command-line workflow to build, flash, calibrate, and run the dual-crank e-bike prototype in this repository.

## 1. Repository Layout

- `XIAO_Crank_Firmware/`
  Left and right pedal firmware. The left and right pedals now use separate PlatformIO environments:
  - `xiaoble_left`
  - `xiaoble_right`
- `ESP32_Main_Firmware/`
  Central controller firmware for BLE collection, rider power math, external goal-watts BLE intake, and motor control.
- `System_Setup_Guide.md`
  This setup guide.

## 2. Hardware Overview

### Pedal Nodes

Each pedal node uses:

- A Seeed XIAO nRF52840 board
- An HX711 load-cell amplifier
- A ShangHJ load cell

The right pedal node also uses the onboard LSM6DS3 IMU for RPM.

### HX711 Wiring

Wire both pedal nodes the same way:

- `HX711 DOUT` -> `XIAO D2`
- `HX711 SCK` -> `XIAO D3`
- `HX711 VCC` -> `XIAO 3.3V`
- `HX711 GND` -> `XIAO GND`

### ESP32 Wiring

- Encoder phase A -> `GPIO 32`
- Encoder phase B -> `GPIO 33`
- L298N `ENA` -> `GPIO 14`
- L298N `IN1` -> `GPIO 27`
- L298N `IN2` -> `GPIO 26`

### Optional External Goal-Watts ESP

The main ESP32 can now connect to a third BLE node that publishes a target goal wattage.

Expected BLE identity for that external ESP:

- Local name: `GOAL_WATTS_NODE`
- Service UUID: `7D200000-E8F2-537E-4F6C-D104768A1214`
- Characteristic UUID: `7D200001-E8F2-537E-4F6C-D104768A1214`

The value is expected to be a BLE `float` representing the new target wattage.

## 3. Software Prerequisites

From the repository root:

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone"
```

Install PlatformIO if needed:

```bash
python3 -m pip install --user platformio
python3 -m platformio --version
```

If `pio` is already on your `PATH`, you can use `pio ...` directly. If not, replace `pio` with `python3 -m platformio` in the commands below.

To list serial devices before flashing:

```bash
pio device list
```

On this machine, a typical USB serial device looked like:

```bash
/dev/cu.usbmodem101
```

## 4. Important Firmware Notes

### Pedal role selection

The pedal role is no longer selected by editing `main.cpp`.

Use:

- `xiaoble_left` for the left pedal
- `xiaoble_right` for the right pedal

The current pedal PlatformIO configuration is in `XIAO_Crank_Firmware/platformio.ini`.

### Board selection note

The current `XIAO_Crank_Firmware/platformio.ini` uses:

```ini
board = xiaoblesense
```

for both pedal environments. If your left pedal is a non-Sense XIAO BLE, update the board target before flashing that node.

### GPS and Wi-Fi note

GPS and terrain logic have been removed from the main ESP32 firmware. The main ESP now focuses on:

- left crank BLE input
- right crank BLE input
- optional external goal-watts BLE input
- motor control

## 5. Build Commands

### Build both pedal firmwares

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/XIAO_Crank_Firmware"
pio run -e xiaoble_left -e xiaoble_right
```

### Build the ESP32 firmware

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/ESP32_Main_Firmware"
pio run
```

## 6. Flashing Commands

### Flash the left pedal

Plug in the left pedal XIAO, then run:

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/XIAO_Crank_Firmware"
pio run -e xiaoble_left -t upload
```

If PlatformIO does not pick the correct serial port automatically:

```bash
pio run -e xiaoble_left -t upload --upload-port /dev/cu.usbmodem101
```

### Flash the right pedal

Plug in the right pedal XIAO, then run:

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/XIAO_Crank_Firmware"
pio run -e xiaoble_right -t upload
```

If needed, specify the port:

```bash
pio run -e xiaoble_right -t upload --upload-port /dev/cu.usbmodem101
```

### Flash the ESP32 

Plug in the ESP32, then run:

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/ESP32_Main_Firmware"
pio run -e esp32dev -t upload
```

If needed, specify the port:

```bash
pio run -e esp32dev -t upload --upload-port /dev/cu.usbmodem101
```

## 7. Serial Monitor Commands

### Left or right pedal serial monitor

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/XIAO_Crank_Firmware"
pio device monitor -b 115200
```

### ESP32 serial monitor

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/ESP32_Main_Firmware"
pio device monitor -b 115200
```

If you need to bind the monitor to a specific port:

```bash
pio device monitor -b 115200 -p /dev/cu.usbmodem101
```

## 8. Load-Cell Zeroing And Calibration

The pedal firmware no longer blindly trusts boot-time tare. It now tries to zero only when the pedal appears unloaded and stable. If startup zero is skipped, the serial monitor will tell you.

### Zero the load cell

1. Open the pedal serial monitor.
2. Make sure the pedal is unloaded.
3. Send:

```text
t
```

The firmware will only zero if the signal is stable and near zero.

### Enter calibration mode

Send:

```text
c
```

Then place a known weight on the load cell and adjust:

- `+` -> increase calibration factor by `100`
- `-` -> decrease calibration factor by `100`
- `f` -> increase calibration factor by `10`
- `g` -> decrease calibration factor by `10`

When the reported force matches the known load, copy the printed calibration factor back into:

`XIAO_Crank_Firmware/src/load_cell.cpp`

Current line:

```cpp
float calibrationFactor = -2280.0f;
```

Then rebuild and reflash that pedal.

## 9. Normal Bring-Up Sequence

Use this order each time you want to run the full system:

1. Flash the left pedal with `xiaoble_left`.
2. Flash the right pedal with `xiaoble_right`.
3. Flash the ESP32 with `esp32dev`.
4. If you have an external goal-watts ESP, power it on and make sure it advertises as `GOAL_WATTS_NODE`.
5. Open the right pedal serial monitor and confirm it reports force and RPM.
6. Open the left pedal serial monitor and confirm it reports force.
7. Open the ESP32 serial monitor and confirm it discovers:
   - `CRANK_LEFT`
   - `CRANK_RIGHT`
   - optionally `GOAL_WATTS_NODE`
8. Confirm the ESP32 prints successful BLE connections and begins using pedal force/RPM data.

## 10. What Each Node Does

### Left pedal node

- Reads force from the HX711/load cell
- Filters the force reading
- Zeroes safely only when unloaded/stable
- Broadcasts BLE as `CRANK_LEFT`

### Right pedal node

- Reads force from the HX711/load cell
- Reads gyro data from the onboard IMU over I2C
- Estimates RPM from gyro magnitude
- Broadcasts BLE as `CRANK_RIGHT`

### ESP32 central

- Scans for `CRANK_LEFT` and `CRANK_RIGHT`
- Optionally scans for `GOAL_WATTS_NODE`
- Subscribes to force notifications from both pedals
- Subscribes to RPM notifications from the right pedal
- Optionally subscribes to target wattage notifications from a third BLE ESP
- Computes rider power from fresh force and RPM data
- Uses the external goal wattage when available, otherwise defaults to `200 W`
- Runs motor control

## 11. Troubleshooting

### `pio` command not found

Use:

```bash
python3 -m platformio run
```

instead of:

```bash
pio run
```

### Load cell reads zero all the time

- Check HX711 wiring on `D2` and `D3`
- Confirm the load cell amplifier is powered from `3.3V`
- Open the pedal serial monitor and send `t` with no load on the pedal
- Recalibrate using a known weight

### Force gets stuck after unplugging or bad wiring

The current firmware should now clear stale force to zero after a short timeout. If it does not, recheck the HX711 wiring and rebuild/flash the updated firmware.

### ESP32 does not connect to the pedals

- Confirm the left node advertises `CRANK_LEFT`
- Confirm the right node advertises `CRANK_RIGHT`
- Make sure both pedals are powered before the ESP32 starts scanning
- Watch the ESP32 serial output for `Found LEFT Node`, `Found RIGHT Node`, and `Successfully connected`

### External goal-watts ESP does not connect

- Confirm it advertises as `GOAL_WATTS_NODE`
- Confirm it exposes:
  - service `7D200000-E8F2-537E-4F6C-D104768A1214`
  - characteristic `7D200001-E8F2-537E-4F6C-D104768A1214`
- Confirm the characteristic contains a BLE `float`
- Watch the ESP32 serial output for `Found GOAL node` and `Successfully connected to Goal-Watts Node`

### RPM is zero on the right pedal

- Confirm the right node was flashed with `xiaoble_right`
- Confirm the board is the Sense variant with the onboard LSM6DS3 IMU
- Spin the crank and watch the right pedal serial output

## 12. Verified Build Commands

These builds completed successfully with the current repository state:

```bash
cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/XIAO_Crank_Firmware"
pio run -e xiaoble_left -e xiaoble_right

cd "/Users/lmainvielle/Desktop/EE-496/EE-496 Capstone/ESP32_Main_Firmware"
pio run
```

## 13. Power Calculation Assumption

The ESP32 now ignores stale force and RPM samples, but the power math still assumes the ShangHJ load-cell output represents tangential pedal force. If the sensor mounting measures a different force component, the watt calculation will still need a geometry correction.
