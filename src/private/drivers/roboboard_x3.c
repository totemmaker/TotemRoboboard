/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_mac.h"

#include "esp32-hal-gpio.h"
#include "esp32-hal-adc.h"

#include "bsp/roboboard_x3.h"
#include "bsp_drivers.h"
// Macros to return peripheral port command and GPIO pin setup
#define RegPort(cmd, port) (cmd + ((port+1)*0x10))
// Amount of peripheral ports available
#define DC_CNT 4
#define SERVO_CNT bsp_servo_cnt
// Runtime states
static bool bsp_initialized = false;
static uint8_t bsp_board_revision = 0;
static uint8_t bsp_servo_cnt = 2;
static struct { bsp_evt_func_t func; void *arg; } bsp_evt_handler;
// Board interrupt handler
static void bsp_on_isr(void *arg) {
    if (bsp_evt_handler.func == NULL) return;
    uint value = 0;
    switch ((uint)arg) {
    case BSP_EVT_USB: value = x3_board_get_usb(); break;
    case BSP_EVT_BUTTON: value = x3_board_get_button(); break;
    case BSP_EVT_CHARGING: value = x3_battery_get_charging(); break;
    }
    bsp_evt_handler.func((uint)arg, 0, value, bsp_evt_handler.arg);
}

/**************************************************************************************************
 * Totem RoboBoard X3 low level control API
 **************************************************************************************************/

/// @brief Initialize BSP driver
/// @return ESP error
esp_err_t x3_board_init() {
    // Initialize GPIO pins
    pinMode(BSP_IO_3V3_EN, OUTPUT);
    digitalWrite(BSP_IO_3V3_EN, 0);
    pinMode(BSP_IO_USB_DETECT, INPUT);
    pinMode(BSP_IO_BUTTON, INPUT_PULLUP);
    pinMode(BSP_IO_BATTERY_CHARGE, INPUT);
    // Initialize interrupts
    attachInterruptArg(BSP_IO_BUTTON, bsp_on_isr, (void*)BSP_EVT_BUTTON, CHANGE);
    attachInterruptArg(BSP_IO_USB_DETECT, bsp_on_isr, (void*)BSP_EVT_USB, CHANGE);
    attachInterruptArg(BSP_IO_BATTERY_CHARGE, bsp_on_isr, (void*)BSP_EVT_CHARGING, CHANGE);
    // Initialize motors
    BSP_ERR(bsp_motor_init());
    // Read board revision
    int verVolt = analogReadMilliVolts(BSP_IO_VERSION_DETECT);
    if (verVolt < 300) bsp_board_revision = 30;
    else if (verVolt > 3000) bsp_board_revision = 30;
    else if (verVolt < 700) {
        bsp_board_revision = 31;
        bsp_servo_cnt = 4;
    }
    // Return initialization state
    bsp_initialized = true;
    return ESP_OK;
}
/// @brief Register board events function
/// @param func event function handler
/// @param arg pointer passed to handler
/// @return ESP error
esp_err_t x3_board_reg_event(bsp_evt_func_t func, void *arg) {
    bsp_evt_handler.func = func;
    bsp_evt_handler.arg = arg;
    return ESP_OK;
}

/*******************************
 * Board control functions
 ******************************/

/// @brief Turn 3.3V power rail (default - off)
/// @param state [0] off, [1] on
/// @note  Affects 3v3 pin and Qwiic
/// @return ESP error
esp_err_t x3_board_set_3V(uint state) {
    digitalWrite(BSP_IO_3V3_EN, !!state); return ESP_OK;
}
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int x3_board_get_revision() {
    return bsp_board_revision;
}
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int x3_board_get_button() {
    return !digitalRead(BSP_IO_BUTTON);
}
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int x3_board_get_usb() {
    return digitalRead(BSP_IO_USB_DETECT);
}

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [2600:4200]mV (millivolts)
int x3_battery_get_voltage() {
    // Read pin voltage in mV
    uint pinVoltage = 0;
    // Oversample
    for (int i=0; i<16; i++) {
        pinVoltage += analogReadMilliVolts(BSP_IO_BATTERY_VOLTAGE);
    }
    pinVoltage /= 16;
    // Covert pin voltage to battery voltage
    return pinVoltage * 2;
}
/// @brief Read battery current (v3.1 only)
/// @return [-2000:2000]mA (milliAmps). [-] discharging
int x3_battery_get_current() {
    // Only supported from v3.1
    if (bsp_board_revision < 31) return 0;
    // Read pin voltage in mV
    int pinVoltage = 0;
    // Oversample
    for (int i=0; i<16; i++) {
        pinVoltage += analogReadMilliVolts(BSP_IO_BATTERY_CURRENT);
    }
    pinVoltage /= 16;
    // Convert pin voltage to battery current
    return (pinVoltage + 180 - 1500) * -10 / 2;
}
/// @brief Is battery charging
/// @return [0] not charging, [1] charging
int x3_battery_get_charging() {
    return !digitalRead(BSP_IO_BATTERY_CHARGE);
}

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t x3_dc_spin(int portID, int power) {
    return bsp_motor_set(portID, MOTOR_DC_MODE_POWER, power);
}
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% braking power, [0] coast
/// @return ESP error
esp_err_t x3_dc_brake(int portID, uint power) {
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    if (power > 100) return ESP_ERR_INVALID_ARG;
    return bsp_motor_set(portID, MOTOR_DC_MODE_BRAKE, power);
}
/// @brief Output audible tone to motor
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [0:20000]Hz tone frequency
/// @return ESP error
esp_err_t x3_dc_tone(int portID, uint frequency) {
    return bsp_motor_set(portID, MOTOR_DC_MODE_TONE, frequency);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x3_dc_set_enable(int portID, uint enable) {
    return bsp_motor_set(portID, MOTOR_DC_ENABLE, !!enable);
}
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t x3_dc_set_decay(int portID, uint decay) {
    return bsp_motor_set(portID, MOTOR_DC_DECAY, decay);
}
/// @brief Change PWM frequency for motor port (default - 20000)
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t x3_dc_set_frequency(int portID, uint frequency) {
    return bsp_motor_set(portID, MOTOR_DC_FREQUENCY, frequency);
}
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int x3_dc_get_decay(uint portID) {
    return bsp_motor_get(portID, MOTOR_DC_DECAY);
}
/// @brief Get configured PWM frequency of motor port
/// @param portID [0:3] port number
/// @return [1:250000]Hz PWM frequency
int x3_dc_get_frequency(uint portID) {
    return bsp_motor_get(portID, MOTOR_DC_FREQUENCY);
}

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:3] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t x3_servo_spin(int portID, uint pulse) {
    return bsp_motor_set(portID, MOTOR_SERVO_PULSE, pulse);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x3_servo_set_enable(int portID, uint enable) {
    return bsp_motor_set(portID, MOTOR_SERVO_ENABLE, !!enable);
}
/// @brief Change PWM period for AB ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_AB(uint period) {
    return bsp_motor_set(0, MOTOR_SERVO_PERIOD, period);
}
/// @brief Change PWM period for CD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_CD(uint period) {
    return bsp_motor_set(2, MOTOR_SERVO_PERIOD, period);
}
/// @brief Change PWM period for ABCD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_ABCD(uint period) {
    BSP_ERR(x3_servo_set_period_AB(period));
    BSP_ERR(x3_servo_set_period_CD(period));
    return ESP_OK;
}
/// @brief Get configured PWM period of AB ports
/// @return [1:65535]us PWM period
int x3_servo_get_period_AB() {
    return bsp_motor_get(0, MOTOR_SERVO_PERIOD);
}
/// @brief Get configured PWM period of CD ports
/// @return [1:65535]us PWM period
int x3_servo_get_period_CD() {
    return bsp_motor_get(2, MOTOR_SERVO_PERIOD);
}
/// @brief Read servo motor position
/// @param portID [0:3] port number
/// @return [0:period]us position pulse
int x3_servo_get_pulse(uint portID) {
    return bsp_motor_get(portID, MOTOR_SERVO_PULSE);
}
/// @brief Get number of servo ports
/// @return [2,4] servo port count
int x3_servo_get_port_cnt() {
    return bsp_servo_cnt;
}
