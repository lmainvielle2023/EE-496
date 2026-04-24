#include "lcd_display.h"
#include "ble_central.h"
#include "power_calc.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_ADDRESS   0x3C

static Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static unsigned long lastUpdateMs = 0;
constexpr unsigned long LCD_UPDATE_INTERVAL_MS = 500;

void initLCD() {
    Serial.println("Initializing LCD...");
    Wire.begin(21, 22);
    if (!display.begin(I2C_ADDRESS, true)) {
        Serial.println("LCD init failed — check wiring");
        return;
    }
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("  E-Bike Controller");
    display.println("   Starting up...");
    display.display();
    Serial.println("LCD Initialized.");
}

void updateLCD() {
    const unsigned long now = millis();
    if (now - lastUpdateMs < LCD_UPDATE_INTERVAL_MS) return;
    lastUpdateMs = now;

    display.clearDisplay();
    display.setTextSize(1);

    // Row 1: Rider watts
    display.setCursor(0, 0);
    display.print("Rider: ");
    display.print(riderWatts, 1);
    display.print(" W");

    // Row 2: Goal watts
    display.setCursor(0, 16);
    display.print("Goal:  ");
    display.print(getTargetGoalWatts(), 1);
    display.print(" W");

    // Row 3: RPM
    display.setCursor(0, 32);
    display.print("RPM:   ");
    display.print(getCurrentRPM(), 0);

    // Row 4: Connection status
    display.setCursor(0, 48);
    display.print("L:");
    display.print(isRegularCrankConnected() ? "OK " : "-- ");
    display.print("R:");
    display.print(isSenseCrankConnected() ? "OK " : "-- ");
    display.print("G:");
    display.print(isGoalNodeConnected() ? "OK" : "--");

    display.display();
}
