/* 
 * Copyright 2025 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEM_DRIVERS_OS_SYS
#define INCLUDE_TOTEM_DRIVERS_OS_SYS

#include "esp_err.h"
#include "sys/types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint os_sys_get_uptime();
void os_sys_delay_ms(uint ms);

typedef enum {
    OS_TASK_PRIO_LOW,
    OS_TASK_PRIO_MED,
    OS_TASK_PRIO_HIGH,
} os_task_priority_t;
typedef void* os_task_t;

os_task_t os_task_create(const char *taskName, void (*func)(void*), void *arg, int stackSize, os_task_priority_t priority);
os_task_t os_task_create_arduino(const char *taskName, void (*func)(void*), void *arg);
void os_task_resume(os_task_t handle);
void os_task_suspend(os_task_t handle);
void os_task_notify_give(os_task_t handle);
void os_task_notify_take();
void os_task_yield();

typedef void* os_timer_t;
os_timer_t os_timer_create(const char *name, void (*func)(void*), void *arg, int periodMs, int autoReload);
void* os_timer_get_arg(os_timer_t handle);
void os_timer_set_arg(os_timer_t handle, void *arg);
int os_timer_is_running(os_timer_t handle);
void os_timer_change_period(os_timer_t handle, int periodMs, int timeout);
void os_timer_start(os_timer_t handle, int timeout);
void os_timer_stop(os_timer_t handle, int timeout);

void os_twai_filter(uint code, uint mask, bool single);
esp_err_t os_twai_install(uint pinTx, uint pinRx, uint mode, uint baud, int txLen, int rxLen, uint alerts);

bool os_rmt_install(uint pinTx);
bool os_rmt_write_blocking(uint8_t buffer[4][3]);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_TOTEM_DRIVERS_OS_SYS */
