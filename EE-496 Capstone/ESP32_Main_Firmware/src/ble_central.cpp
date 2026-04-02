#include "ble_central.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// Shared Target Power 
double targetWatts = 0.0;

// BLE UUIDs for Crank Sensor
static BLEUUID serviceUUID("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLEUUID charUUID("19B10001-E8F2-537E-4F6C-D104768A1214");

static boolean doConnect = false;
static boolean connected = false;
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

// Scan interval
static int scanTime = 5; // In seconds
static BLEScan* pBLEScan;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if(length == sizeof(float)) {
        float val;
        memcpy(&val, pData, sizeof(float));
        targetWatts = (double)val; // Update the global used by Motor Control
        Serial.print("Received Target Watts: ");
        Serial.println(targetWatts);
    }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.println("Connected to Seeed XIAO (Crank Node)");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from Seeed XIAO (Crank Node)");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remote BLE Server.
    pClient->connect(myDevice);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      Serial.println(" - Registered for notifications (ready to read wattage!)");
    }

    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // Check if this device advertises our service UUID
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
        Serial.print("Found XIAO Sensor Node: ");
        Serial.println(advertisedDevice.getAddress().toString().c_str());
        
        BLEDevice::getScan()->stop();
        if (myDevice != nullptr) {
            delete myDevice;
        }
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
      }
    }
};

void initBLECentral() {
    Serial.println("Initializing BLE Central...");

    // Setup basic BLE stack
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); 
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); 
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99); 

    Serial.println("BLE Central Initialized.");
}

void updateBLECentral() {
    // If we found the target but haven't connected yet, try connecting
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("Successfully connected to Crank Node. Waiting for data...");
      } else {
        Serial.println("Failed to connect to Crank Node.");
      }
      doConnect = false;
    }
    
    // Only scan if we are explicitly disconnected
    if (!connected) {
        // Serial.println("Scanning for Crank Node...");
        pBLEScan->start(scanTime, false);
        pBLEScan->clearResults(); 
    }
}
