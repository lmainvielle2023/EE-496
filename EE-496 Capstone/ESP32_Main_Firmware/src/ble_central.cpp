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
constexpr double DEFAULT_GOAL_WATTS = 3.0;
double targetGoalWatts = DEFAULT_GOAL_WATTS;

// BLE UUIDs for Crank Sensors
static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID forceCharUUID("19B10001-E8F2-537E-4F6C-D104768A1214");
static BLEUUID rpmCharUUID("19B10002-E8F2-537E-4F6C-D104768A1214");

// BLE UUIDs for external goal-watts broadcaster
static BLEUUID goalServiceUUID("7D200000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID goalCharUUID("7D200001-E8F2-537E-4F6C-D104768A1214");

// States
static boolean doConnectRegular = false;
static boolean doConnectSense = false;
static boolean doConnectGoal = false;
static boolean connectedRegular = false;
static boolean connectedSense = false;
static boolean connectedGoal = false;

static BLEAdvertisedDevice* myDeviceRegular = nullptr;
static BLEAdvertisedDevice* myDeviceSense = nullptr;
static BLEAdvertisedDevice* myDeviceGoal = nullptr;

static BLEClient* pClientRegular = nullptr;
static BLEClient* pClientSense = nullptr;
static BLEClient* pClientGoal = nullptr;

static BLEScan* pBLEScan;
static unsigned long lastScanTime = 0;
#define SCAN_INTERVAL_MS 5000
constexpr bool BLE_SCAN_DEBUG = true;

// Notify Callbacks
static void notifyCallbackRegularForce(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        addRegularForce(val);
    }
}

static void notifyCallbackSenseForce(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        addSenseForce(val);
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
class MyClientCallbackRegular : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connectedRegular = true;
    Serial.println("Connected to REGULAR Crank Node");
  }
  void onDisconnect(BLEClient* pclient) {
    connectedRegular = false;
    Serial.println("Disconnected from REGULAR Crank Node");
  }
};

class MyClientCallbackSense : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connectedSense = true;
    Serial.println("Connected to SENSE Crank Node");
  }
  void onDisconnect(BLEClient* pclient) {
    connectedSense = false;
    Serial.println("Disconnected from SENSE Crank Node");
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

bool connectToServerRegular() {
    Serial.print("Connecting to REGULAR: ");
    Serial.println(myDeviceRegular->getAddress().toString().c_str());
    pBLEScan->stop();
    delay(100);
    pClientRegular = BLEDevice::createClient();
    pClientRegular->setClientCallbacks(new MyClientCallbackRegular());
    pClientRegular->connect(myDeviceRegular);

    BLERemoteService* pRemoteService = pClientRegular->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClientRegular->disconnect();
      return false;
    }

    BLERemoteCharacteristic* pForceChar = pRemoteService->getCharacteristic(forceCharUUID);
    if (pForceChar && pForceChar->canNotify()) {
      pForceChar->registerForNotify(notifyCallbackRegularForce);
    }

    return true;
}

bool connectToServerSense() {
    Serial.print("Connecting to SENSE: ");
    Serial.println(myDeviceSense->getAddress().toString().c_str());
    pBLEScan->stop();
    delay(100);
    pClientSense = BLEDevice::createClient();
    pClientSense->setClientCallbacks(new MyClientCallbackSense());
    pClientSense->connect(myDeviceSense);

    BLERemoteService* pRemoteService = pClientSense->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClientSense->disconnect();
      return false;
    }

    // 1. Subscribe to Force
    BLERemoteCharacteristic* pForceChar = pRemoteService->getCharacteristic(forceCharUUID);
    if (pForceChar && pForceChar->canNotify()) {
      pForceChar->registerForNotify(notifyCallbackSenseForce);
    }

    // FIX: Give the XIAO a moment to ACK the first subscription
    delay(200);

    // 2. Subscribe to RPM
    BLERemoteCharacteristic* pRpmChar = pRemoteService->getCharacteristic(rpmCharUUID);
    if (pRpmChar && pRpmChar->canNotify()) {
      pRpmChar->registerForNotify(notifyCallbackRPM);
    }
    
    // FIX: Give the XIAO another moment before hitting it with a Read Request
    delay(200);

    // 3. Read initial RPM
    if (pRpmChar && pRpmChar->canRead()) {
      std::string value = pRpmChar->readValue();
      if (value.size() == sizeof(float)) {
        float val;
        memcpy(&val, value.data(), sizeof(float));
        setRPM(val);
        Serial.print("Initial SENSE RPM read: ");
        Serial.println(val, 1);
      }
    }

    return true;
}

bool connectToServerGoal() {
    Serial.print("Connecting to GOAL node: ");
    Serial.println(myDeviceGoal->getAddress().toString().c_str());
    pBLEScan->stop();
    delay(100);
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
        if (devName == "CRANK_REGULAR" && !connectedRegular && !doConnectRegular) {
            if (BLE_SCAN_DEBUG) {
                Serial.print("BLE scan saw REGULAR: ");
                Serial.println(advertisedDevice.getAddress().toString().c_str());
            }
            Serial.print("Found REGULAR Node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            if (myDeviceRegular != nullptr) delete myDeviceRegular;
            myDeviceRegular = new BLEAdvertisedDevice(advertisedDevice);
            doConnectRegular = true;
        } 
        else if (devName == "CRANK_SENSE" && !connectedSense && !doConnectSense) {
            if (BLE_SCAN_DEBUG) {
                Serial.print("BLE scan saw SENSE: ");
                Serial.println(advertisedDevice.getAddress().toString().c_str());
            }
            Serial.print("Found SENSE Node: ");
            Serial.println(advertisedDevice.getAddress().toString().c_str());
            if (myDeviceSense != nullptr) delete myDeviceSense;
            myDeviceSense = new BLEAdvertisedDevice(advertisedDevice);
            doConnectSense = true;
        }
        else if (devName == "GOAL_WATTS_NODE" && !connectedGoal && !doConnectGoal) {
            if (BLE_SCAN_DEBUG) {
                Serial.print("BLE scan saw GOAL: ");
                Serial.println(advertisedDevice.getAddress().toString().c_str());
            }
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
    if (doConnectRegular) {
      if (connectToServerRegular()) {
        Serial.println("Successfully connected to REGULAR Node.");
      }
      doConnectRegular = false;
    }

    if (doConnectSense) {
      if (connectToServerSense()) {
        Serial.println("Successfully connected to SENSE Node.");
      }
      doConnectSense = false;
    }

    if (doConnectGoal) {
      if (connectToServerGoal()) {
        Serial.println("Successfully connected to Goal-Watts Node.");
      }
      doConnectGoal = false;
    }
    
    // Keep the crank links mandatory. The external goal node is optional.
    if ((!connectedRegular || !connectedSense) && (millis() - lastScanTime >= SCAN_INTERVAL_MS)) {
        lastScanTime = millis();
        pBLEScan->start(2, true);  // non-blocking
        pBLEScan->clearResults();
    }
}

bool isRegularCrankConnected() {
    return connectedRegular;
}

bool isSenseCrankConnected() {
    return connectedSense;
}

bool isGoalNodeConnected() {
    return connectedGoal;
}

double getTargetGoalWatts() {
    return targetGoalWatts;
}
