#include "terrain_predict.h"
#include "system_pins.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // Use UART 1 for GPS

void initTerrainPredict() {
    Serial.println("Initializing Terrain Predict...");

    // Setup GPS Serial (NEO-6M typically defaults to 9600 baud)
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // WiFi disabled for now - uncomment and update secrets.h when terrain API is needed
    // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.println("WiFi skipped (not configured).");
}

void updateTerrainPredict() {
    // 1. Process NMEA data from GPS UART continuously
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            // New valid GPS data parsed!
            if (gps.location.isValid()) {
                double myLat = gps.location.lat();
                double myLng = gps.location.lng();
                
                // 2. Predict Trajectory and Fetch Elevation
                // Placeholder: send GET request to HTTP API on Wi-Fi
                // e.g. String url = "https://api.opentopodata.org/v1/srtm90m?locations=" + String(predictedLat) + "," + String(predictedLng);
                // We should only poll an HTTP request every few seconds.
            }
        }
    }
}
