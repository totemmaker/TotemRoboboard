/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_ROBOBOARD_X3_H
#define INCLUDE_BSP_ROBOBOARD_X3_H

#include "esp_err.h"
#include "sys/types.h"
#include "hal/gpio_types.h"

/**************************************************************************************************
 * Totem RoboBoard X3 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_IO_I2C_SDA           (GPIO_NUM_15) //MTDO
#define BSP_IO_I2C_SCL           (GPIO_NUM_5)

/* GPIO */
#define BSP_IO_IO26              (GPIO_NUM_26)
#define BSP_IO_IO32              (GPIO_NUM_32)
#define BSP_IO_IO33              (GPIO_NUM_33)

/* DC motor */
#define BSP_IO_MOTORA_INA        (GPIO_NUM_22)
#define BSP_IO_MOTORA_INB        (GPIO_NUM_12) // MTDI
#define BSP_IO_MOTORB_INA        (GPIO_NUM_23)
#define BSP_IO_MOTORB_INB        (GPIO_NUM_19)
#define BSP_IO_MOTORC_INA        (GPIO_NUM_18)
#define BSP_IO_MOTORC_INB        (GPIO_NUM_21)
#define BSP_IO_MOTORD_INA        (GPIO_NUM_4)
#define BSP_IO_MOTORD_INB        (GPIO_NUM_2)

/* Servo motor */
#define BSP_IO_SERVOA_IN         (GPIO_NUM_25)
#define BSP_IO_SERVOB_IN         (GPIO_NUM_14) // MTMS
#define BSP_IO_SERVOC_IN         (GPIO_NUM_16) // v3.1 only
#define BSP_IO_SERVOD_IN         (GPIO_NUM_17) // v3.1 only

/* Others */
#define BSP_IO_BUTTON            (GPIO_NUM_0)
#define BSP_IO_RGB               (GPIO_NUM_13) // MTCK
#define BSP_IO_3V3_EN            (GPIO_NUM_27)
#define BSP_IO_VERSION_DETECT    (GPIO_NUM_34) // VDET_1
#define BSP_IO_BATTERY_CHARGE    (GPIO_NUM_35) // VDET_2
#define BSP_IO_BATTERY_CURRENT   (GPIO_NUM_36) // SENSOR_VP
#define BSP_IO_BATTERY_VOLTAGE   (GPIO_NUM_37) // SENSOR_CAPP
#define BSP_IO_USB_DETECT        (GPIO_NUM_38) // SENSOR_CAPN
#define BSP_IO_IMU_INT           (GPIO_NUM_39) // SENSOR_VN

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * Totem RoboBoard X3 low level control API
 **************************************************************************************************/
// Board events list
enum {
    // USB cable plug/unplug event (called from ISR)
    BSP_EVT_USB,
    // Button press/release event (called from ISR)
    BSP_EVT_BUTTON,
    // Charging start/stop event (called from ISR)
    BSP_EVT_CHARGING,
};
// Board events function
typedef void (*bsp_evt_func_t)(uint evt, uint portID, uint value, void *arg);
// Names for A, B, C, D ports
enum {
    BSP_PORT_ALL = -1, // All ports
    BSP_PORT_A = 0, // Port A
    BSP_PORT_B = 1, // Port B
    BSP_PORT_C = 2, // Port C
    BSP_PORT_D = 3, // Port D
};

/// @brief Initialize BSP driver
/// @return ESP error
esp_err_t x3_board_init();
/// @brief Register board events function
/// @param func event function handler
/// @param arg pointer passed to handler
/// @return ESP error
esp_err_t x3_board_reg_event(bsp_evt_func_t func, void *arg);

/*******************************
 * Board control functions
 ******************************/

/// @brief Turn 3.3V power rail (default - off)
/// @param state [0] off, [1] on
/// @note  Affects 3v3 pin and Qwiic
/// @return ESP error
esp_err_t x3_board_set_3V(uint state);
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int x3_board_get_revision();
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int x3_board_get_button();
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int x3_board_get_usb();

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [2600:4200]mV (millivolts)
int x3_battery_get_voltage();
/// @brief Read battery current (v3.1 only)
/// @return [-2000:2000]mA (milliAmps). [-] discharging
int x3_battery_get_current();
/// @brief Is battery charging
/// @return [0] not charging, [1] charging
int x3_battery_get_charging();

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t x3_dc_spin(int portID, int power);
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% braking power, [0] coast
/// @return ESP error
esp_err_t x3_dc_brake(int portID, uint power);
/// @brief Output audible tone to motor
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [0:20000]Hz tone frequency
/// @return ESP error
esp_err_t x3_dc_tone(int portID, uint frequency);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x3_dc_set_enable(int portID, uint enable);
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t x3_dc_set_decay(int portID, uint decay);
/// @brief Change PWM frequency for motor port (default - 20000)
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t x3_dc_set_frequency(int portID, uint frequency);
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int x3_dc_get_decay(uint portID);
/// @brief Get configured PWM frequency of motor port
/// @param portID [0:3] port number
/// @return [1:250000]Hz PWM frequency
int x3_dc_get_frequency(uint portID);

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:3] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t x3_servo_spin(int portID, uint pulse);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x3_servo_set_enable(int portID, uint enable);
/// @brief Change PWM period for AB ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_AB(uint period);
/// @brief Change PWM period for CD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_CD(uint period);
/// @brief Change PWM period for ABCD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x3_servo_set_period_ABCD(uint period);
/// @brief Get configured PWM period of AB ports
/// @return [1:65535]us PWM period
int x3_servo_get_period_AB();
/// @brief Get configured PWM period of CD ports
/// @return [1:65535]us PWM period
int x3_servo_get_period_CD();
/// @brief Read servo motor position
/// @param portID [0:3] port number
/// @return [0:period]us position pulse
int x3_servo_get_pulse(uint portID);
/// @brief Get number of servo ports
/// @return [2,4] servo port count
int x3_servo_get_port_cnt();

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_ROBOBOARD_X3_H*/
