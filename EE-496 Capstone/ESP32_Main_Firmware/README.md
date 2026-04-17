# ESP32 E-Bike Controller Firmware

## Setup

Install PlatformIO and add it to your PATH (one-time):
```bash
echo 'export PATH="$HOME/Library/Python/3.9/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
```

Add the `flash` alias for one-command build+upload+monitor (one-time):
```bash
echo 'alias flash="cd \"$(pwd)\" && pio run -t upload && pio device monitor"' >> ~/.zshrc
source ~/.zshrc
```

## Commands

| Command | What it does |
|---|---|
| `pio run` | Build only |
| `pio run -t upload` | Build and flash to ESP32 |
| `pio device monitor` | Open serial monitor at 115200 baud |
| `flash` | Build + flash + open monitor (one command, requires alias above) |

## Serial Monitor Output

### GPS Status
While waiting for a GPS fix, you will see every 3 seconds:
```
Waiting for GPS fix... chars=142 sentences=0 failed=0
```

| Reading | Meaning |
|---|---|
| `chars=0` | GPS module not wired or not powered — check pins 16/17 |
| `chars>0, sentences=0` | Receiving data but no fix yet — go outside or near a window |
| `sentences>0` | Fix acquired, elevation API calls will begin |

### Elevation Output (once GPS fix is acquired)
```
Lat: 32.123456  Lon: 34.123456  Heading: 270.0
Elev now: 45.0m  Ahead: 48.0m  Delta: 3.0m
UPHILL — boost modifier: 0.07
```

## Hardware Pin Reference

| Pin | Function |
|---|---|
| GPIO 14 | Motor PWM (ENA) |
| GPIO 27 | Motor direction IN1 |
| GPIO 26 | Motor direction IN2 |
| GPIO 32 | Encoder phase A |
| GPIO 33 | Encoder phase B |
| GPIO 16 | GPS RX |
| GPIO 17 | GPS TX |

## WiFi / Secrets

Edit `include/secrets.h` to set your WiFi credentials before flashing.
