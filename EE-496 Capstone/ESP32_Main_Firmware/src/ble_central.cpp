#include "ble_central.h"
#include "power_calc.h"
#include <BLEDevice.h>
#include <esp_bt.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// Shared Rider Power 
double riderWatts = 0.0;

// BLE UUIDs for Crank Sensors
static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID forceCharUUID("19B10001-E8F2-537E-4F6C-D104768A1214");
static BLEUUID rpmCharUUID("19B10002-E8F2-537E-4F6C-D104768A1214");

// States
static boolean doConnectLeft = false;
static boolean doConnectRight = false;
static boolean connectedLeft = false;
static boolean connectedRight = false;

static BLEAdvertisedDevice* myDeviceLeft = nullptr;
static BLEAdvertisedDevice* myDeviceRight = nullptr;

static BLEClient* pClientLeft = nullptr;
static BLEClient* pClientRight = nullptr;

static BLEScan* pBLEScan;
static unsigned long lastScanTime = 0;
#define SCAN_INTERVAL_MS 5000

// Notify Callbacks
static void notifyCallbackLeftForce(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        addLeftForce(val);
    }
}

static void notifyCallbackRightForce(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        addRightForce(val);
    }
}

static void notifyCallbackRPM(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        setRPM(val);
    }
}

// Client Callbacks
class MyClientCallbackLeft : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connectedLeft = true;
    Serial.println("Connected to LEFT Crank Node");
  }
  void onDisconnect(BLEClient* pclient) {
    connectedLeft = false;
    Serial.println("Disconnected from LEFT Crank Node");
  }
};

class MyClientCallbackRight : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connectedRight = true;
    Serial.println("Connected to RIGHT Crank Node");
  }
  void onDisconnect(BLEClient* pclient) {
    connectedRight = false;
    Serial.println("Disconnected from RIGHT Crank Node");
  }
};

bool connectToServerLeft() {
    Serial.print("Connecting to LEFT: ");
    Serial.println(myDeviceLeft->getAddress().toString().c_str());
    pClientLeft = BLEDevice::createClient();
    pClientLeft->setClientCallbacks(new MyClientCallbackLeft());
    pClientLeft->connect(myDeviceLeft);

    BLERemoteService* pRemoteService = pClientLeft->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClientLeft->disconnect();
      return false;
    }

    BLERemoteCharacteristic* pForceChar = pRemoteService->getCharacteristic(forceCharUUID);
    if (pForceChar && pForceChar->canNotify()) {
      pForceChar->registerForNotify(notifyCallbackLeftForce);
    }

    return true;
}

bool connectToServerRight() {
    Serial.print("Connecting to RIGHT: ");
    Serial.println(myDeviceRight->getAddress().toString().c_str());
    pClientRight = BLEDevice::createClient();
    pClientRight->setClientCallbacks(new MyClientCallbackRight());
    pClientRight->connect(myDeviceRight);

    BLERemoteService* pRemoteService = pClientRight->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClientRight->disconnect();
      return false;
    }

    // Force Characteristic
    BLERemoteCharacteristic* pForceChar = pRemoteService->getCharacteristic(forceCharUUID);
    if (pForceChar && pForceChar->canNotify()) {
      pForceChar->registerForNotify(notifyCallbackRightForce);
    }

    // RPM Characteristic (only on right)
    BLERemoteCharacteristic* pRpmChar = pRemoteService->getCharacteristic(rpmCharUUID);
    if (pRpmChar && pRpmChar->canNotify()) {
      pRpmChar->registerForNotify(notifyCallbackRPM);
    }

    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.haveName()) {
        std::string devName = advertisedDevice.getName();
        if (devName == "CRANK_LEFT" && !connectedLeft && !doConnectLeft) {
            Serial.print("Found LEFT Node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            if (myDeviceLeft != nullptr) delete myDeviceLeft;
            myDeviceLeft = new BLEAdvertisedDevice(advertisedDevice);
            doConnectLeft = true;
        } 
        else if (devName == "CRANK_RIGHT" && !connectedRight && !doConnectRight) {
            Serial.print("Found RIGHT Node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            if (myDeviceRight != nullptr) delete myDeviceRight;
            myDeviceRight = new BLEAdvertisedDevice(advertisedDevice);
            doConnectRight = true;
        }
      }
    }
};

void initBLECentral() {
    Serial.println("Initializing BLE Central...");
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); 
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); 
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); 
    Serial.println("BLE Central Initialized.");
}

void updateBLECentral() {
    if (doConnectLeft) {
      if (connectToServerLeft()) {
        Serial.println("Successfully connected to LEFT Node.");
      }
      doConnectLeft = false;
    }

    if (doConnectRight) {
      if (connectToServerRight()) {
        Serial.println("Successfully connected to RIGHT Node.");
      }
      doConnectRight = false;
    }
    
    // Only scan if one of them is missing, throttled to avoid blocking HTTP calls
    if ((!connectedLeft || !connectedRight) && (millis() - lastScanTime >= SCAN_INTERVAL_MS)) {
        lastScanTime = millis();
        pBLEScan->start(2, true);  // non-blocking
        pBLEScan->clearResults();
    }
}
