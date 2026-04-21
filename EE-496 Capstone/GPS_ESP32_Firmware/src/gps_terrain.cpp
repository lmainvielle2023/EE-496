#include "gps_terrain.h"
#include "gps_pins.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <math.h>

static TinyGPSPlus gps;
static HardwareSerial GPS_Serial(2);

static unsigned long lastElevCallMs = 0;
static unsigned long lastGpsLogMs = 0;
static unsigned long lastWifiRetryMs = 0;
static unsigned long initMs = 0;
static bool gpsTimeoutReached = false;

// How far ahead (metres) to look for elevation change
constexpr double LOOKAHEAD_M = 20.0;

constexpr unsigned long ELEV_INTERVAL_MS     = 3000;
constexpr unsigned long GPS_LOG_INTERVAL_MS  = 3000;
constexpr unsigned long GPS_FIX_TIMEOUT_MS   = 300000; // 5 min
constexpr unsigned long WIFI_RETRY_MS        = 5000;

// Terrain modifier constants
// These turn an elevation delta (metres) into a watt adjustment.
// +/- 2m dead-band, then linearly up to +/- MAX_WATT_ADJUST over 15m.
constexpr float ELEV_DEADBAND_M   = 2.0f;
constexpr float ELEV_SCALE_M      = 15.0f;
constexpr float MAX_WATT_ADJUST   = 100.0f; // max boost/reduction in watts

void initGPSTerrain() {
    Serial.println("Initializing GPS Terrain...");
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    initMs = millis();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiRetryMs = initMs;
    Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
}

// Returns a watt adjustment based on terrain delta.
// Positive = uphill boost, negative = downhill reduction.
static float terrainWattAdjust(double elevDeltaM) {
    if (elevDeltaM > ELEV_DEADBAND_M) {
        float raw = (float)(elevDeltaM - ELEV_DEADBAND_M) / ELEV_SCALE_M;
        return constrain(raw, 0.0f, 1.0f) * MAX_WATT_ADJUST;
    } else if (elevDeltaM < -ELEV_DEADBAND_M) {
        float raw = (float)(elevDeltaM + ELEV_DEADBAND_M) / ELEV_SCALE_M;
        return constrain(raw, -1.0f, 0.0f) * MAX_WATT_ADJUST;
    }
    return 0.0f;
}

float updateGPSTerrain(float baseGoalWatts) {
    // Feed GPS serial into TinyGPS++
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }

    // Log while waiting for fix
    if (!gps.location.isValid()) {
        if (!gpsTimeoutReached && millis() - lastGpsLogMs >= GPS_LOG_INTERVAL_MS) {
            lastGpsLogMs = millis();
            Serial.printf("Waiting for GPS fix... chars=%u sentences=%u failed=%u\n",
                gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum());
        }
        if (!gpsTimeoutReached && millis() - initMs >= GPS_FIX_TIMEOUT_MS) {
            gpsTimeoutReached = true;
            Serial.println("GPS timeout — broadcasting base goal watts until fix.");
        }
        return baseGoalWatts;
    }

    if (gpsTimeoutReached) {
        Serial.println("GPS fix acquired.");
        gpsTimeoutReached = false;
    }

    // Rate-limit the API calls
    if (millis() - lastElevCallMs < ELEV_INTERVAL_MS) {
        return baseGoalWatts;
    }

    // Retry WiFi if dropped
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - lastWifiRetryMs >= WIFI_RETRY_MS) {
            lastWifiRetryMs = millis();
            WiFi.reconnect();
        }
        return baseGoalWatts;
    }

    lastElevCallMs = millis();

    double myLat  = gps.location.lat();
    double myLng  = gps.location.lng();
    double heading = gps.course.isValid() ? gps.course.deg() : 0.0;

    // Calculate a point LOOKAHEAD_M metres ahead along heading
    double d    = LOOKAHEAD_M / 6371000.0;
    double lat1 = myLat * DEG_TO_RAD;
    double lon1 = myLng * DEG_TO_RAD;
    double brng = heading * DEG_TO_RAD;
    double lat2 = asin(sin(lat1)*cos(d) + cos(lat1)*sin(d)*cos(brng));
    double lon2 = lon1 + atan2(sin(brng)*sin(d)*cos(lat1), cos(d) - sin(lat1)*sin(lat2));
    double aheadLat = lat2 * RAD_TO_DEG;
    double aheadLon = lon2 * RAD_TO_DEG;

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    String url  = "https://api.opentopodata.org/v1/srtm90m";
    String body = "locations=";
    body += String(myLat, 6) + "," + String(myLng, 6);
    body += "|" + String(aheadLat, 6) + "," + String(aheadLon, 6);

    http.begin(client, url);
    http.setTimeout(8000);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int code = http.POST(body);

    float goalWatts = baseGoalWatts;

    if (code == 200) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, http.getString());
        if (!err) {
            double elevNow   = doc["results"][0]["elevation"].as<double>();
            double elevAhead = doc["results"][1]["elevation"].as<double>();
            double delta     = elevAhead - elevNow;

            float adjustment = terrainWattAdjust(delta);
            goalWatts = baseGoalWatts + adjustment;

            const char* terrain = (delta > ELEV_DEADBAND_M) ? "UPHILL" :
                                  (delta < -ELEV_DEADBAND_M) ? "DOWNHILL" : "FLAT";

            Serial.printf("GPS: %.6f, %.6f  Hdg: %.1f  Elev: %.1fm -> %.1fm  Delta: %.1fm  [%s]  Goal: %.1fW\n",
                myLat, myLng, heading, elevNow, elevAhead, delta, terrain, goalWatts);
        }
    } else {
        Serial.printf("Elevation API failed (HTTP %d) — using base goal watts\n", code);
    }

    http.end();
    return goalWatts;
}
