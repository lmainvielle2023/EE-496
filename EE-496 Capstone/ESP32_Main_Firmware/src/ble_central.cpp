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
constexpr double DEFAULT_GOAL_WATTS = 200.0;
double targetGoalWatts = DEFAULT_GOAL_WATTS;

// BLE UUIDs for Crank Sensors
static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID forceCharUUID("19B10001-E8F2-537E-4F6C-D104768A1214");
static BLEUUID rpmCharUUID("19B10002-E8F2-537E-4F6C-D104768A1214");

// BLE UUIDs for external goal-watts broadcaster
static BLEUUID goalServiceUUID("7D200000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID goalCharUUID("7D200001-E8F2-537E-4F6C-D104768A1214");

// States
static boolean doConnectLeft = false;
static boolean doConnectRight = false;
static boolean doConnectGoal = false;
static boolean connectedLeft = false;
static boolean connectedRight = false;
static boolean connectedGoal = false;

static BLEAdvertisedDevice* myDeviceLeft = nullptr;
static BLEAdvertisedDevice* myDeviceRight = nullptr;
static BLEAdvertisedDevice* myDeviceGoal = nullptr;

static BLEClient* pClientLeft = nullptr;
static BLEClient* pClientRight = nullptr;
static BLEClient* pClientGoal = nullptr;

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

static void notifyCallbackGoalWatts(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        if (isfinite(val) && val >= 0.0f) {
            targetGoalWatts = val;
        }
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

class MyClientCallbackGoal : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connectedGoal = true;
    Serial.println("Connected to Goal-Watts Node");
  }
  void onDisconnect(BLEClient* pclient) {
    connectedGoal = false;
    targetGoalWatts = DEFAULT_GOAL_WATTS;
    Serial.println("Disconnected from Goal-Watts Node");
    Serial.print("Falling back to default goal watts: ");
    Serial.println(targetGoalWatts, 1);
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

bool connectToServerGoal() {
    Serial.print("Connecting to GOAL node: ");
    Serial.println(myDeviceGoal->getAddress().toString().c_str());
    pClientGoal = BLEDevice::createClient();
    pClientGoal->setClientCallbacks(new MyClientCallbackGoal());
    pClientGoal->connect(myDeviceGoal);

    BLERemoteService* pRemoteService = pClientGoal->getService(goalServiceUUID);
    if (pRemoteService == nullptr) {
      pClientGoal->disconnect();
      return false;
    }

    BLERemoteCharacteristic* pGoalChar = pRemoteService->getCharacteristic(goalCharUUID);
    if (pGoalChar == nullptr) {
      pClientGoal->disconnect();
      return false;
    }

    if (pGoalChar->canNotify()) {
      pGoalChar->registerForNotify(notifyCallbackGoalWatts);
    }

    if (pGoalChar->canRead()) {
      std::string value = pGoalChar->readValue();
      if (value.size() == sizeof(float)) {
        float val;
        memcpy(&val, value.data(), sizeof(float));
        if (isfinite(val) && val >= 0.0f) {
          targetGoalWatts = val;
        }
      }
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
        else if (devName == "GOAL_WATTS_NODE" && !connectedGoal && !doConnectGoal) {
            Serial.print("Found GOAL node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            if (myDeviceGoal != nullptr) delete myDeviceGoal;
            myDeviceGoal = new BLEAdvertisedDevice(advertisedDevice);
            doConnectGoal = true;
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

    if (doConnectGoal) {
      if (connectToServerGoal()) {
        Serial.println("Successfully connected to Goal-Watts Node.");
      }
      doConnectGoal = false;
    }
    
    // Rescan whenever any node is missing
    if ((!connectedLeft || !connectedRight || !connectedGoal) && (millis() - lastScanTime >= SCAN_INTERVAL_MS)) {
        lastScanTime = millis();
        pBLEScan->start(2, true);  // non-blocking
        pBLEScan->clearResults();
    }
}

bool isLeftCrankConnected() {
    return connectedLeft;
}

bool isRightCrankConnected() {
    return connectedRight;
}

bool isGoalNodeConnected() {
    return connectedGoal;
}

double getTargetGoalWatts() {
    return targetGoalWatts;
}
