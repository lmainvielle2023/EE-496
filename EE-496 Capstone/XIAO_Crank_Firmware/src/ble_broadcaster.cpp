#include "ble_broadcaster.h"
#include <ArduinoBLE.h>

// Standard dummy UUIDs for our capstone prototype
BLEService crankService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEFloatCharacteristic forceCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEFloatCharacteristic rpmCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

static bool isRightPedal = false;

void initBLEBroadcaster(bool isRightNode) {
    isRightPedal = isRightNode;
    Serial.println("Initializing BLE Broadcaster...");
    
    if (!BLE.begin()) {
        Serial.println("starting Bluetooth® Low Energy failed!");
        while (1);
    }
    
    // Set up local name based on node type
    if (isRightPedal) {
        BLE.setLocalName("CRANK_RIGHT");
    } else {
        BLE.setLocalName("CRANK_LEFT");
    }
    BLE.setAdvertisedService(crankService);
    
    // Add characteristics to the service
    crankService.addCharacteristic(forceCharacteristic);
    
    if (isRightPedal) {
        crankService.addCharacteristic(rpmCharacteristic);
        rpmCharacteristic.writeValue(0.0f);
    }
    
    // Add service to the BLE stack
    BLE.addService(crankService);
    
    // Set initial value for the characteristic
    forceCharacteristic.writeValue(0.0f);
    
    // Start advertising
    BLE.advertise();
    
    Serial.print("BLE Broadcaster Initialized & Advertising as ");
    Serial.println(isRightPedal ? "CRANK_RIGHT" : "CRANK_LEFT");
}

void updateBLEBroadcaster(float current_force, float current_rpm) {
    // Only update if connected to a central device to save power/bandwidth
    BLEDevice central = BLE.central();
    if (central) {
        forceCharacteristic.writeValue(current_force);
        if (isRightPedal) {
            rpmCharacteristic.writeValue(current_rpm);
        }
    }
}
