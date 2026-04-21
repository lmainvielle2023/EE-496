#include "ble_goal_broadcaster.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Must match the UUIDs the main ESP32 scans for
#define GOAL_SERVICE_UUID  "7D200000-E8F2-537E-4F6C-D104768A1214"
#define GOAL_CHAR_UUID     "7D200001-E8F2-537E-4F6C-D104768A1214"

static BLEServer* pServer = nullptr;
static BLECharacteristic* pGoalChar = nullptr;
static bool deviceConnected = false;
static bool advertising = false;

class GoalServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        advertising = false;
        Serial.println("Main ESP32 connected to Goal-Watts Node.");
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Main ESP32 disconnected — restarting advertising.");
        pServer->startAdvertising();
        advertising = true;
    }
};

void initBLEGoalBroadcaster() {
    Serial.println("Initializing BLE Goal Broadcaster...");

    BLEDevice::init("GOAL_WATTS_NODE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new GoalServerCallbacks());

    BLEService* pService = pServer->createService(GOAL_SERVICE_UUID);

    pGoalChar = pService->createCharacteristic(
        GOAL_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pGoalChar->addDescriptor(new BLE2902());

    // Start with 200W default so the main ESP32 gets something immediately on connect
    float defaultWatts = 200.0f;
    pGoalChar->setValue((uint8_t*)&defaultWatts, sizeof(float));

    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(GOAL_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    advertising = true;

    Serial.println("BLE Goal Broadcaster advertising as GOAL_WATTS_NODE.");
}

void updateBLEGoalWatts(float goalWatts) {
    pGoalChar->setValue((uint8_t*)&goalWatts, sizeof(float));
    if (deviceConnected) {
        pGoalChar->notify();
    }
}
