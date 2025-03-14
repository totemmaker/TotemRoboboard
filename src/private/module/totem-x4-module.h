/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEM_X4_MODULE
#define INCLUDE_TOTEM_X4_MODULE

#include "sys/types.h"

#include "totem-module-data.h"
#include "private/totem-can.h"

namespace _RoboBoard {

class TotemModule {
public:
    TotemModule *next = nullptr;
    using DataReceiver = void (*)(totem::ModuleData data);
    using DataReceiver2 = void (*)(void *context, totem::ModuleData data);
    using DataReceiver3 = void (*)(void *context, totem::ModuleData data, int ack);

    TotemModule(uint16_t number, uint16_t serial, void *context, DataReceiver3 receiver);
    TotemModule(uint16_t number, uint16_t serial, void *context, DataReceiver2 receiver);
    TotemModule(uint16_t number, uint16_t serial, DataReceiver receiver);
    TotemModule(uint16_t number, uint16_t serial = 0);

    ~TotemModule();

    static void processMessageCAN(uint id, uint8_t *data, uint8_t len);
    static void onMessageCAN() {
        auto packet = totem::CAN.getPacket();
        uint id = packet.id;
        if (packet.ext) id |= 0x80000000UL;
        if (packet.rtr) id |= 0x40000000UL;
        processMessageCAN(id, packet.data, packet.len);
    }
    static int sendMessageCAN(uint id, uint8_t *data, uint8_t len) {
        int ext = id & 0x80000000UL;
        int rtr = id & 0x40000000UL;
        id = ext ? (id & 0x1FFFFFFF) : (id & 0x7FF);
        return totem::CAN.writePacket(ext, id, data, len, rtr);
    }
    static void initCAN() {
        totem::CAN.addEvent(onMessageCAN);
        totem::CAN.begin(500);
    }

    bool write(const char *command) {
        return write(hashCmd(command));
    }
    bool write(const char *command, int32_t value) {
        return write(hashCmd(command), value);
    }
    template <int VSIZE>
    bool write(const char *command, const char (&value)[VSIZE]) {
        return write(hashCmd(command), value);
    }
    bool write(const char *command, const uint8_t *bytes, uint bytesCount) {
        return write(hashCmd(command), bytes, bytesCount);
    }
    bool write(const char *command, int8_t A, int8_t B, int8_t C, int8_t D) {
        return write(hashCmd(command), A, B, C, D);
    }
    bool write(const char *command, int8_t A, int8_t B, int8_t C) {
        return write(hashCmd(command), A, B, C);
    }
    bool writeWait(const char *command) {
        return writeWait(hashCmd(command));
    }
    bool writeWait(const char *command, int32_t value) {
        return writeWait(hashCmd(command), value);
    }
    template <int VSIZE>
    bool writeWait(const char *command, const char (&value)[VSIZE]) {
        return writeWait(hashCmd(command), value);
    }
    bool writeWait(const char *command, const uint8_t *bytes, uint bytesCount) {
        return writeWait(hashCmd(command), bytes, bytesCount);
    }
    bool writeWait(const char *command, int8_t A, int8_t B, int8_t C, int8_t D) {
        return writeWait(hashCmd(command), A, B, C, D);
    }
    bool writeWait(const char *command, int8_t A, int8_t B, int8_t C) {
        return writeWait(hashCmd(command), A, B, C);
    }
    
    bool read(const char *command) {
        return read(hashCmd(command));
    }
    totem::ModuleData readWait(const char *command) {
        return readWait(hashCmd(command));
    }
    bool readWait(const char *command, totem::ModuleData &result) {
        return readWait(hashCmd(command), result);
    }
    bool subscribe(const char *command, int intervalMs = 0) {
        return subscribe(hashCmd(command), intervalMs);
    }
    bool subscribeWait(const char *command, int intervalMs = 0) {
        return subscribeWait(hashCmd(command), intervalMs);
    }
    bool unsubscribe(const char *command) {
        return unsubscribe(hashCmd(command));
    }
    bool unsubscribeWait(const char *command) {
        return unsubscribeWait(hashCmd(command));
    }
    void attachOnData(DataReceiver receiver) {
        this->receiver = receiver;
    }

    bool write(uint command) {
        return moduleWrite(command, false);
    }
    bool write(uint command, int32_t value) {
        return moduleWrite(command, value, false);
    }
    template <int VSIZE>
    bool write(uint command, const char (&value)[VSIZE]) {
        return moduleWrite(command, value, VSIZE-1, false);
    }
    bool write(uint command, const uint8_t *bytes, uint bytesCount) {
        return moduleWrite(command, reinterpret_cast<const char*>(bytes), bytesCount, false);
    }
    bool write(uint command, int8_t A, int8_t B, int8_t C, int8_t D) {
        return moduleWrite(command, toValue(A, B, C, D), false);
    }
    bool write(uint command, int8_t A, int8_t B, int8_t C) {
        return moduleWrite(command, toValue(0, A, B, C), false);
    }
    bool writeWait(uint command) {
        return moduleWrite(command, true);
    }
    bool writeWait(uint command, int32_t value) {
        return moduleWrite(command, value, true);
    }
    template <int VSIZE>
    bool writeWait(uint command, const char (&value)[VSIZE]) {
        return moduleWrite(command, value, VSIZE-1, true);
    }
    bool writeWait(uint command, const uint8_t *bytes, uint bytesCount) {
        return moduleWrite(command, reinterpret_cast<const char*>(bytes), bytesCount, true);
    }
    bool writeWait(uint command, int8_t A, int8_t B, int8_t C, int8_t D) {
        return moduleWrite(command, toValue(A, B, C, D), true);
    }
    bool writeWait(uint command, int8_t A, int8_t B, int8_t C) {
        return moduleWrite(command, toValue(0, A, B, C), true);
    }

    bool read(uint command) {
        return moduleRead(command, false);
    }
    totem::ModuleData readWait(uint command) {
        response.command = command;
        response.waiting = true;
        bool succ = moduleRead(response.command, true);
        if (!succ || response.waiting) {
            totem::ModuleData data(0, (uint8_t*)"", 0);
            return data;
        }
        return response.data;
    }
    bool readWait(uint command, totem::ModuleData &result) {
        response.command = command;
        response.waiting = true;
        bool succ = moduleRead(response.command, true);
        if (!succ || response.waiting) return false;
        result = response.data;
        return true;
    }
    bool subscribe(uint command, int intervalMs = 0, bool responseReq = false) {
        return moduleSubscribe(command, intervalMs, responseReq);
    }
    bool subscribeWait(uint command, int intervalMs = 0) {
        return moduleSubscribe(command, intervalMs, true);
    }
    bool unsubscribe(uint command) {
        return subscribe(command, -1);
    }
    bool unsubscribeWait(uint command) {
        return subscribeWait(command, -1);
    }

    void setNumber(uint16_t number) {
        this->number = number;
    }
    void setSerial(uint16_t serial) {
        this->serial = serial;
    }
    uint16_t getNumber() {
        return number;
    }
    uint16_t getSerial() {
        return serial;
    }
    uint getError() {
        return requestError;
    }
    bool ping();
    
    static constexpr uint hashCmd(const char *command) {
        return fnv1a32Hash(2166136261, command);
    }
    static constexpr uint16_t hashModel(const char *model) {
        return fnv1a16Hash(2166136261, model);
    }

    void onMessageReceive(void *message);
private:
    uint16_t number;
    uint16_t serial;
    volatile struct {
        int    command = -1;
        bool succ = false;
        bool waiting = false;
    } request;
    void giveResponse(int command, bool succ);
    void prepareWait(int command);
    bool waitResponse(int timeout);
    bool isFromModule(uint16_t number, uint16_t serial);
    void onModuleData(totem::ModuleData msgData, int ack);
    uint requestError = 0;
    void setError(uint err) {
        requestError = err;
    }
    uint toValue(int8_t A, int8_t B, int8_t C, int8_t D) {
        uint value = 0;
        value |= (static_cast<uint>(A) & 0xFF) << 24;
        value |= (static_cast<uint>(B) & 0xFF) << 16;
        value |= (static_cast<uint>(C) & 0xFF) << 8;
        value |= (static_cast<uint>(D) & 0xFF);
        return value;
    }
    
    struct {
        int command;
        totem::ModuleData data;
        bool waiting;
    } response;
    void *receiverContext = nullptr;
    DataReceiver receiver = nullptr;
    DataReceiver2 receiver2 = nullptr;
    DataReceiver3 receiver3 = nullptr;

    bool moduleWrite(int command, bool responseReq);
    bool moduleWrite(int command, int value, bool responseReq);
    bool moduleWrite(int command, const char *data, uint len, bool responseReq);
    bool moduleRead(int command, bool blocking);
    bool moduleSubscribe(int command, int interval, bool responseReq);
    
    static constexpr uint fnv1a32Hash(uint hash, const char *cmd) {
        return (*cmd) ? fnv1a32Hash(((hash ^ *cmd ) * 16777619), cmd+1) : hash;
    }
    static constexpr uint fnv1a16Hash(uint hash, const char *cmd) {
        return (*cmd) ? fnv1a16Hash(((hash ^ *cmd ) * 16777619), cmd+1) : ((hash>>16) ^ (hash & ((uint)0xFFFF)));
    }

    void *mutex = nullptr;
};

// Convert string to 32bit hash
static constexpr uint CMD(const char *cmdString) {
    return TotemModule::hashCmd(cmdString);
}
// Event data object
struct Event {
    totem::ModuleData data;
    struct {
        uint8_t subscribed : 1;
        uint8_t streaming : 1;
    } state = { .subscribed = 0, .streaming = 0 };

    Event(uint cmd) : data(cmd, 0, 0) { }
    Event(uint cmd, uint8_t *ptr, uint size) : data(cmd, ptr, size) { }
};

} // namespace _RoboBoard

// Totem module with event functionality
class TotemX4Module {
    _RoboBoard::Event *eventList;
    uint16_t eventListCnt;
public:
    template <int SIZE>
    TotemX4Module(uint16_t number, uint16_t serial, _RoboBoard::Event (&eventList)[SIZE]) : 
    eventList(eventList), eventListCnt(SIZE), module(number, serial, this, onModuleDataReceive) { }
    // Register event handler function
    using EventHandler = void (*)(int evt);
    void addEvent(EventHandler evtHandler) {
        this->evtHandler = evtHandler;
    }
    // Reset module subscriptions
    void reset() {
        for (int i=0; i<eventListCnt; i++) {
            if (eventList[i].state.streaming) {
                module.unsubscribe(eventList[i].data.getHashCmd());
            }
            eventList[i].state.streaming = 0;
            eventList[i].state.subscribed = 0;
        }
    }

    static bool find(uint16_t number = 0, uint16_t serial = 0);
    static uint16_t foundNumber;
    static uint16_t foundSerial;

    _RoboBoard::TotemModule module;
protected:
    // Write 32bit int value to command
    void writeCmdInt(uint cmd, int32_t value) {
        module.write(cmd, value);
    }
    // Read command returning ModuleData object
    totem::ModuleData readCmd(uint cmd) {
        // Return event object if command is added to the list
        _RoboBoard::Event *event = findEvent(cmd);
        if (event) {
            // Subscribe if not already streaming
            if (!event->state.streaming) {
                event->state.subscribed = 1;
                module.subscribe(cmd, 0, true);
                return module.readWait(cmd);
            }
            // Return cached value
            return event->data;
        }
        else {
            // Perform simple read until result is received
            return module.readWait(cmd);
        }
    }
    // Read command returning 32bit integer
    int32_t readCmdInt(uint cmd) {
        return readCmd(cmd).getInt();
    }
    // Call registered event handler
    virtual void callEventHandler(int event) {
        if (evtHandler) evtHandler(event);
    }

private:
    // Event handler instance
    // void *eventHandlerInst = nullptr;
    EventHandler evtHandler;
    // Find event index by command
    int findEventIdx(uint cmd) {
        for (int i=0; i<eventListCnt; i++) {
            if (eventList[i].data.getHashCmd() == cmd)
                return i;
        }
        return -1;
    }
    // Find event object by command
    _RoboBoard::Event* findEvent(uint cmd) {
        int idx = findEventIdx(cmd);
        return idx == -1 ? nullptr : &eventList[idx];
    }
    // Module data handler
    static void onModuleDataReceive(void *context, totem::ModuleData moduleData, int ack) {
        TotemX4Module *instance = (TotemX4Module*)context;
        int idx = instance->findEventIdx(moduleData.getHashCmd());
        if (idx != -1) {
            // Get event object
            _RoboBoard::Event *event = &(instance->eventList[idx]);
            // Update event state and call handler
            if (ack == -1) {
                event->state.subscribed = 0;
            }
            else if (ack == 1 && event->state.subscribed) {
                event->state.streaming = 1;
            }
            else if (event->state.streaming) {
                // Save latest data and call event handler
                event->data = moduleData;
                instance->callEventHandler(((uint)instance->module.getNumber() << 16) + idx);
            }
        }
    }
};

#endif /* INCLUDE_TOTEM_X4_MODULE */
