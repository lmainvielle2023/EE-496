#include "terrain_predict.h"
#include "system_pins.h"
#include "secrets.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <math.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

static unsigned long lastElevCall = 0;
#define ELEV_INTERVAL_MS 3000

void initTerrainPredict() {
    Serial.println("Initializing Terrain Predict...");

    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Trying SSID: '%s' Password: '%s'\n", WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    } else {
        Serial.println("\nWiFi failed — elevation API unavailable");
    }
}

void updateTerrainPredict() {
    while (GPS_Serial.available() > 0) {
        gps.encode(GPS_Serial.read());
    }

    if (!gps.location.isValid()) return;

    if (millis() - lastElevCall < ELEV_INTERVAL_MS) return;
    lastElevCall = millis();

    if (WiFi.status() != WL_CONNECTED) return;

    double myLat = gps.location.lat();
    double myLng = gps.location.lng();
    double heading = gps.course.isValid() ? gps.course.deg() : 0.0;

    double d    = 20.0 / 6371000.0;
    double lat1 = myLat * DEG_TO_RAD;
    double lon1 = myLng * DEG_TO_RAD;
    double brng = heading * DEG_TO_RAD;
    double lat2 = asin(sin(lat1)*cos(d) + cos(lat1)*sin(d)*cos(brng));
    double lon2 = lon1 + atan2(sin(brng)*sin(d)*cos(lat1), cos(d)-sin(lat1)*sin(lat2));
    double aheadLat = lat2 * RAD_TO_DEG;
    double aheadLon = lon2 * RAD_TO_DEG;

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    String url = "https://api.opentopodata.org/v1/srtm90m";
    String body = "locations=";
    body += String(myLat, 6) + "," + String(myLng, 6);
    body += "|" + String(aheadLat, 6) + "," + String(aheadLon, 6);

    http.begin(client, url);
    http.setTimeout(8000);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int code = http.POST(body);

    if (code == 200) {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, http.getString());
        if (!err) {
            double elevNow   = doc["results"][0]["elevation"].as<double>();
            double elevAhead = doc["results"][1]["elevation"].as<double>();
            double delta     = elevAhead - elevNow;

            Serial.printf("Lat: %.6f  Lon: %.6f  Heading: %.1f\n", myLat, myLng, heading);
            Serial.printf("Elev now: %.1fm  Ahead: %.1fm  Delta: %.1fm\n", elevNow, elevAhead, delta);

            float modifier = 0.0f;
            if (delta > 2.0) {
                modifier = constrain((float)(delta - 2.0) / 15.0f, 0.0f, 1.0f);
                Serial.printf("UPHILL — boost modifier: %.2f\n", modifier);
            } else if (delta < -2.0) {
                modifier = constrain((float)(delta + 2.0) / 15.0f, -1.0f, 0.0f);
                Serial.printf("DOWNHILL — regen modifier: %.2f\n", modifier);
            } else {
                Serial.println("FLAT — no terrain modifier");
            }
        }
    } else {
        Serial.printf("Elevation API failed — HTTP %d\n", code);
    }

    http.end();
}