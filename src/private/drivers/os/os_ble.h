/* 
 * Copyright 2025 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEM_DRIVERS_OS_BLE
#define INCLUDE_TOTEM_DRIVERS_OS_BLE

#include <RTOS.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include "os_ble_interface.h"
#include "esp_arduino_version.h"

namespace _RoboBoard {

class ObBleDriver : public OsBleInterface, BLEServerCallbacks, BLECharacteristicCallbacks, BLEDescriptorCallbacks {
    BLEServer *server = nullptr;
public:
    void* createServer(const char *name) override {
        if (server) return nullptr;
        // Create server
        BLEDevice::init(name);
        BLEDevice::setMTU(517);
        server = BLEDevice::createServer();
        server->setCallbacks(this);
        return server;
    }
    void configAdvertisement(const char *uuid, const char *data, uint len) override {
        BLEAdvertisementData advData;
        advData.setFlags(0x06);
        advData.setCompleteServices(BLEUUID(uuid));
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        advData.setManufacturerData(std::string(data, len));
#else
        advData.setManufacturerData(String((const uint8_t*)data, len));
#endif
        BLEDevice::getAdvertising()->setAdvertisementData(advData);
    }
    void configScanResponse(const char *name) override {
        BLEAdvertisementData scanRespData;
        scanRespData.setName(name);
        BLEDevice::getAdvertising()->setScanResponse(true);
        BLEDevice::getAdvertising()->setScanResponseData(scanRespData);
    }
    Service_t addService(const char *uuid) override {
        if (!server) return nullptr;
        return server->createService(uuid);
    }
    Service_t addService(uint16_t uuid) override {
        if (!server) return nullptr;
        return server->createService(uuid);
    }
    Characteristic_t addCharacteristic(Service_t service, const char *uuid, int prop) override {
        if (!service) return nullptr;
        BLECharacteristic *bleChar = ((BLEService*)service)->createCharacteristic(uuid, prop);
        if (prop & BLECharacteristic::PROPERTY_NOTIFY) {
            BLEDescriptor *bleDescriptor = new BLE2902();
            bleDescriptor->setCallbacks(this);
            bleChar->addDescriptor(bleDescriptor);
        }
        bleChar->setCallbacks(this);
        return bleChar;
    }
    Characteristic_t addCharacteristic(Service_t service, uint16_t uuid, int prop) override {
        if (!service) return nullptr;
        BLECharacteristic *bleChar = ((BLEService*)service)->createCharacteristic(uuid, prop);
        if (prop & BLECharacteristic::PROPERTY_NOTIFY) {
            BLEDescriptor *bleDescriptor = new BLE2902();
            bleDescriptor->setCallbacks(this);
            bleChar->addDescriptor(bleDescriptor);
        }
        bleChar->setCallbacks(this);
        return bleChar;
    }
    void setCharacteristic(Characteristic_t characteristic, const char *value) override {
        ((BLECharacteristic*)characteristic)->setValue(value);
    }
    void notifyCharacteristic(Characteristic_t characteristic, uint8_t *data, uint len) override {
        ((BLECharacteristic*)characteristic)->setValue(data, len);
        ((BLECharacteristic*)characteristic)->notify();
    }
    void startService(Service_t service) override {
        ((BLEService*)service)->start();
    }
    void startAdvertising() override {
        BLEDevice::getAdvertising()->start();
    }
    void stopAdvertising() override {
        BLEDevice::getAdvertising()->start();
    }
    bool isConnected() override {
        return server->getConnectedCount() != 0;
    }
    void disconnect() override {
        if (!isConnected()) return;
        server->disconnect(server->getConnId());
    }
    int getServiceMTU(Service_t service) override {
        if (server->getConnectedCount() == 0) return 0;
        return server->getPeerMTU(server->getConnId())-3;
    }
private:
    // BLEServerCallbacks
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        server->updateConnParams(param->connect.remote_bda, 6, 6, 0, 100);
        OsBleInterfaceCallback::onConnect();
    }
    // BLEServerCallbacks
    void onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        OsBleInterfaceCallback::onDisconnect(param->disconnect.reason);
    }
    // BLEServerCallbacks
    void onMtuChanged(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override { }
    // BLECharacteristicCallbacks
    void onRead(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) override {
        OsBleInterfaceCallback::onRead(pCharacteristic);
    }
    // BLECharacteristicCallbacks
    void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t *param) override {
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        OsBleInterfaceCallback::onWrite(pCharacteristic, String(pCharacteristic->getValue().c_str(), pCharacteristic->getValue().length()));
#else
        OsBleInterfaceCallback::onWrite(pCharacteristic, pCharacteristic->getValue());
#endif
    }
    // BLEDescriptorCallbacks
    void onWrite(BLEDescriptor *pDescriptor) override {
        OsBleInterfaceCallback::onWrite(pDescriptor, pDescriptor->getValue(), pDescriptor->getLength());
    }
};

} // namespace _RoboBoard

#endif /* INCLUDE_TOTEM_DRIVERS_OS_BLE */
