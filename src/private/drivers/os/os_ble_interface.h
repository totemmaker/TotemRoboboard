/* 
 * Copyright 2025 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEM_DRIVERS_OS_BLE_INTERFACE
#define INCLUDE_TOTEM_DRIVERS_OS_BLE_INTERFACE

#include <WString.h>
#include "sys/types.h"

namespace _RoboBoard {

class OsBleInterface {
public:
    using Service_t = void*;
    using Descriptor_t = void*;
    using Characteristic_t = void*;

    virtual void* createServer(const char *name) = 0;
    virtual void configAdvertisement(const char *uuid, const char *data, uint len) = 0;
    virtual void configScanResponse(const char *name) = 0;
    virtual Service_t addService(const char *uuid) = 0;
    virtual Service_t addService(uint16_t uuid) = 0;
    virtual Characteristic_t addCharacteristic(Service_t service, const char *uuid, int prop) = 0;
    virtual Characteristic_t addCharacteristic(Service_t service, uint16_t uuid, int prop) = 0;
    virtual void setCharacteristic(Characteristic_t characteristic, const char *value) = 0;
    virtual void notifyCharacteristic(Characteristic_t characteristic, uint8_t *data, uint len) = 0;
    virtual void startService(Service_t service) = 0;
    virtual void startAdvertising() = 0;
    virtual void stopAdvertising() = 0;
    virtual bool isConnected() = 0;
    virtual void disconnect() = 0;
    virtual int getServiceMTU(Service_t service) = 0;
};

class OsBleInterfaceCallback {
public:
    static void onConnect();
    static void onDisconnect(int reason);
    static void onRead(OsBleInterface::Characteristic_t chr);
    static void onWrite(OsBleInterface::Characteristic_t chr, String value);
    static void onWrite(OsBleInterface::Descriptor_t dsc, uint8_t *data, uint len);
};

} // namespace _RoboBoard

#endif /* INCLUDE_TOTEM_DRIVERS_OS_BLE_INTERFACE */
