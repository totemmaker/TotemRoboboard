/* 
 * Copyright 2025 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "os_sys.h"

uint os_sys_get_uptime() {
    return pdMS_TO_TICKS(xTaskGetTickCount());
}
void os_sys_delay_ms(uint ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

os_task_t os_task_create(const char *taskName, void (*func)(void*), void *arg, int stackSize, os_task_priority_t priority) {
    int uxPriority = 1;
    switch (priority) {
        case OS_TASK_PRIO_LOW: uxPriority = 2; break;
        case OS_TASK_PRIO_MED: uxPriority = 10; break;
        case OS_TASK_PRIO_HIGH: uxPriority = 21; break;
    }
    TaskHandle_t taskHandle = NULL;
    int pass = xTaskCreatePinnedToCore((TaskFunction_t)func, taskName, stackSize, arg, uxPriority, &taskHandle, !CONFIG_ARDUINO_RUNNING_CORE);
    return taskHandle;
}
os_task_t os_task_create_arduino(const char *taskName, void (*func)(void*), void *arg) {
    TaskHandle_t taskHandle = NULL;
    xTaskCreatePinnedToCore((TaskFunction_t)func, taskName, CONFIG_ARDUINO_LOOP_STACK_SIZE, arg, 1, &taskHandle, CONFIG_ARDUINO_RUNNING_CORE);
    return taskHandle;
}

void os_task_resume(os_task_t handle) {
    vTaskResume((TaskHandle_t)handle);
}
void os_task_suspend(os_task_t handle) {
    vTaskSuspend((TaskHandle_t)handle);
}
void os_task_notify_give(os_task_t handle) {
    xTaskNotifyGive((TaskHandle_t)handle);
}
void os_task_notify_take() {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}
void os_task_yield() {
    taskYIELD();
}

os_timer_t os_timer_create(const char *name, void (*func)(void*), void *arg, int periodMs, int autoReload) {
    return xTimerCreate(name, pdMS_TO_TICKS(periodMs), autoReload, arg, (TimerCallbackFunction_t)func);
}
void* os_timer_get_arg(os_timer_t handle) {
    return pvTimerGetTimerID((TimerHandle_t)handle);
}
void os_timer_set_arg(os_timer_t handle, void *arg) {
    vTimerSetTimerID((TimerHandle_t)handle, arg);
}
int os_timer_is_running(os_timer_t handle) {
    return xTimerIsTimerActive((TimerHandle_t)handle);
}
void os_timer_change_period(os_timer_t handle, int periodMs, int timeout) {
    if (timeout == -1) xTimerChangePeriodFromISR((TimerHandle_t)handle, periodMs, NULL);
    else xTimerChangePeriod((TimerHandle_t)handle, pdMS_TO_TICKS(periodMs), pdMS_TO_TICKS(timeout));
}
void os_timer_start(os_timer_t handle, int timeout) {
    if (timeout == -1) xTimerStartFromISR((TimerHandle_t)handle, NULL);
    else xTimerStart((TimerHandle_t)handle, pdMS_TO_TICKS(timeout));
}
void os_timer_stop(os_timer_t handle, int timeout) {
    if (timeout == -1) xTimerStopFromISR((TimerHandle_t)handle, NULL);
    else xTimerStop((TimerHandle_t)handle, pdMS_TO_TICKS(timeout));
}

#include "driver/twai.h"

static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static twai_timing_config_t twai_get_timing_config(int baud) {
    switch (baud) {
        case 25: return (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
        case 50: return (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        case 100: return (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        case 125: return (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        case 250: return (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        case 500: return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        case 800: return (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        case 1000: return (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
    }
    return (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
}

void os_twai_filter(uint code, uint mask, bool single) {
    f_config.acceptance_code = code;
    f_config.acceptance_mask = mask;
    f_config.single_filter = single;
}

esp_err_t os_twai_install(uint pinTx, uint pinRx, uint mode, uint baud, int txLen, int rxLen, uint alerts) {
    esp_err_t err = ESP_FAIL;
    // Configure TWAI parameters
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(pinTx, pinRx, mode);
    g_config.tx_queue_len = txLen;
    g_config.rx_queue_len = rxLen;
    g_config.alerts_enabled = alerts;
    // Select baud rate config
    twai_timing_config_t t_config = twai_get_timing_config(baud);
    // Initialize TWAI driver
    return twai_driver_install(&g_config, &t_config, &f_config);
}

#include "esp_arduino_version.h"
#include "esp32-hal-rmt.h"

static struct {
#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    #define RMT_SYMBOLS_OF(x) (sizeof(x) / sizeof(rmt_data_t))
    rmt_obj_t *obj;
#endif
    rmt_data_t data[24*4+1];
    uint pin;
} rmtStruct;

bool os_rmt_install(uint pinTx) {
    rmtStruct.pin = pinTx;
    rmtStruct.data[RMT_SYMBOLS_OF(rmtStruct.data)-1] = (rmt_data_t){{200, 0, 200, 0}};
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    return rmtInit(pinTx, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
#else
    rmtStruct.obj = rmtInit(pinTx, RMT_TX_MODE, RMT_MEM_64);
    rmtSetTick(rmtStruct.obj, 100);
    return rmtStruct.obj != NULL;
#endif
}

bool os_rmt_write_blocking(uint8_t buffer[4][3]) {
    const rmt_data_t bit0 = {{3, 1, 9, 0}};
    const rmt_data_t bit1 = {{6, 1, 6, 0}};
    for (int i=0,item=0; item<4; item++) {
        for (int col=0; col<3; col++) {
            for (int bit=0; bit<8; bit++,i++) {
                rmtStruct.data[i] = (((buffer[item][col]) & (1<<(7-bit)))) ? bit1 : bit0;
            }
        }
    }
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    return rmtWrite(rmtStruct.pin, rmtStruct.data, RMT_SYMBOLS_OF(rmtStruct.data), RMT_WAIT_FOR_EVER);
#else
    return rmtWriteBlocking(rmtStruct.obj, rmtStruct.data, RMT_SYMBOLS_OF(rmtStruct.data));
#endif
}
