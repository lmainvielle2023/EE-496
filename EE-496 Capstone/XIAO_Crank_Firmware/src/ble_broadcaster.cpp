#include "ble_broadcaster.h"
#include <ArduinoBLE.h>

// Standard dummy UUIDs for our capstone prototype
BLEService crankService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic wattsCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

void initBLEBroadcaster() {
    Serial.println("Initializing BLE Broadcaster...");
    
    if (!BLE.begin()) {
        Serial.println("starting Bluetooth® Low Energy failed!");
        while (1);
    }
    
    // Set up local name and service
    BLE.setLocalName("CRANK_NODE");
    BLE.setAdvertisedService(crankService);
    
    // Add characteristic to the service
    crankService.addCharacteristic(wattsCharacteristic);
    
    // Add service to the BLE stack
    BLE.addService(crankService);
    
    // Set initial value for the characteristic
    wattsCharacteristic.writeValue(0.0f);
    
    // Start advertising
    BLE.advertise();
    
    Serial.println("BLE Broadcaster Initialized & Advertising.");
}

void updateBLEBroadcaster(float current_watts) {
    // Only update if connected to a central device to save power/bandwidth,
    // though the ArduinoBLE library handles this safely either way.
    BLEDevice central = BLE.central();
    if (central) {
        wattsCharacteristic.writeValue(current_watts);
    }
}
