/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_ROBOBOARD_X4_H
#define INCLUDE_BSP_ROBOBOARD_X4_H

#include "esp_err.h"
#include "sys/types.h"
#include "hal/gpio_types.h"

/**************************************************************************************************
 * Totem RoboBoard X4 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_IO_I2C_SDA           (GPIO_NUM_21)
#define BSP_IO_I2C_SCL           (GPIO_NUM_22)

/* CAN */
#define BSP_IO_CAN_TX            (GPIO_NUM_17)
#define BSP_IO_CAN_RX            (GPIO_NUM_34)
#define BSP_IO_CAN_EN            (GPIO_NUM_5)  // v1.1 only

/* GPIO */
#define BSP_IO_GPIOA             (GPIO_NUM_14) // v1.1 only
#define BSP_IO_GPIOB             (GPIO_NUM_23) // v1.1 only
#define BSP_IO_GPIOC             (GPIO_NUM_25) // v1.1 only
#define BSP_IO_GPIOD             (GPIO_NUM_26) // v1.1 only

/* Driver */
#define BSP_IO_DRIVER_DFU        (GPIO_NUM_4)
#define BSP_IO_DRIVER_RESET      (GPIO_NUM_15)
#define BSP_IO_DRIVER_UART_RX    (GPIO_NUM_16)
#define BSP_IO_DRIVER_UART_TX    (GPIO_NUM_27)

/* Others */
#define BSP_IO_LED               (GPIO_NUM_13)
#define BSP_IO_BUTTON            (GPIO_NUM_18)
#define BSP_IO_POWER_DETECT      (GPIO_NUM_19)
#define BSP_IO_USB_DETECT        (GPIO_NUM_39)
#define BSP_IO_BATTERY_VOLTAGE   (GPIO_NUM_36)
#define BSP_IO_ACCEL_INT         (GPIO_NUM_35) // v1.1 only
#define BSP_IO_ACCEL_INT_v10     (GPIO_NUM_14) // v1.0 only

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * Totem RoboBoard X4 low level control API
 **************************************************************************************************/
// Board events list
enum {
    // USB cable plug/unplug event (called from ISR)
    BSP_EVT_USB,
    // DC power adapter plug/unplug event (called from ISR)
    BSP_EVT_POWER,
    // Button press/release event (called from ISR)
    BSP_EVT_BUTTON,
    // Servo position pulse change (during speed control)
    BSP_EVT_SERVO_PULSE,
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
esp_err_t x4_board_init();
/// @brief Register board events function
/// @param func event function handler
/// @param arg pointer passed to handler
/// @return ESP error
esp_err_t x4_board_reg_event(bsp_evt_func_t func, void *arg);

/*******************************
 * Board control functions
 ******************************/

/// @brief Turn LED (pin 13) (default - off)
/// @param state [0] off, [1] on
/// @return ESP error
esp_err_t x4_board_set_led(uint state);
/// @brief Turn 5V power rail (default - on) (v1.1 only)
/// @param state [0] off, [1] on
/// @note  Affects servo (+) and RGB power
/// @return ESP error
esp_err_t x4_board_set_5V(uint state);
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int x4_board_get_revision();
/// @brief Get motor driver firmware version
/// @return Format: [123] -> v1.23
int x4_board_get_firmware();
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int x4_board_get_button();
/// @brief Is power adapter plugged in (v1.0 only)
/// @return [0] unplugged, [1] plugged in
int x4_board_get_power();
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int x4_board_get_usb();

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [8400:12600]mV (millivolts)
int x4_battery_get_voltage();

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t x4_dc_spin(int portID, int power);
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% braking power, [0] coast
/// @return ESP error
esp_err_t x4_dc_brake(int portID, uint power);
/// @brief Output audible tone to AB ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t x4_dc_tone_AB(uint frequency, uint duration);
/// @brief Output audible tone to CD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t x4_dc_tone_CD(uint frequency, uint duration);
/// @brief Output audible tone to ABCD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t x4_dc_tone_ABCD(uint frequency, uint duration);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x4_dc_set_enable(int portID, uint enable);
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t x4_dc_set_decay(int portID, uint decay);
/// @brief Change PWM frequency for AB ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t x4_dc_set_frequency_AB(uint frequency);
/// @brief Change PWM frequency for CD ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t x4_dc_set_frequency_CD(uint frequency);
/// @brief Change PWM frequency for ABCD ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t x4_dc_set_frequency_ABCD(uint frequency);
/// @brief Change group AB mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t x4_dc_set_mode_AB(uint mode);
/// @brief Change group CD mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t x4_dc_set_mode_CD(uint mode);
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int x4_dc_get_decay(uint portID);
/// @brief Get configured PWM frequency of AB ports
/// @return [1:250000]Hz PWM frequency
int x4_dc_get_frequency_AB();
/// @brief Get configured PWM frequency of CD ports
/// @return [1:250000]Hz PWM frequency
int x4_dc_get_frequency_CD();

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t x4_servo_spin(int portID, uint pulse);
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param duration [0] max speed, spin speed in duration (ms) (overrides default speed)
/// @note  Duration: amount of time to spin from current to target position
/// @return ESP error
esp_err_t x4_servo_spin_duration(int portID, uint pulse, uint duration);
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param ppp [0] max speed, spin speed in PPP unit (overrides default speed)
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t x4_servo_spin_ppp(int portID, uint pulse, uint ppp);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t x4_servo_set_enable(int portID, uint enable);
/// @brief Configure constant motor speed (default - disabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param ppp [0] disabled (max speed), spin speed in PPP unit
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t x4_servo_set_speed_ppp(int portID, uint ppp);
/// @brief Change PWM period for ABC ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t x4_servo_set_period_ABC(uint period);
/// @brief Get configured PWM period of ABC ports
/// @return [1:65535]us PWM period
int x4_servo_get_period_ABC();
/// @brief Read servo motor position
/// @param portID [0:2] port number
/// @return [0:period]us position pulse
int x4_servo_get_pulse(uint portID);

/*******************************
 * RGB control functions
 ******************************/

/// @brief Change RGB light color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t x4_rgb_color(int ledID, uint hex);
/// @brief Prepare RGB light fade color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit fade color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t x4_rgb_fade_color(int ledID, uint hex);
/// @brief Run fade animation
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param duration animation duration (ms)
/// @return ESP error
esp_err_t x4_rgb_fade_start(int ledID, uint duration);
/// @brief Toggle LED output (default - enabled)
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param enable [0] LED is disabled, [1] LED is enabled
/// @return ESP error
esp_err_t x4_rgb_set_enable(int ledID, uint enable);

/*******************************
 * GPIO pin control functions. RoboBoard X4 v1.0 only!
 * Pins are connected to peripheral driver (STM32) so
 * additional functions are required for interaction.
 * For later versions use standard GPIO API!
 ******************************/

/// @brief Read digital state of GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @return [0] LOW, [1] HIGH
uint x4_gpio_digital_read(uint pinID);
/// @brief Write digital state to GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param state [0] LOW, [1] HIGH
void x4_gpio_digital_write(uint pinID, uint state);
/// @brief Configure GPIO mode
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param mode [0] pd, [1] pu, [2] float, [3] out, [4] analog
void x4_gpio_mode(uint pinID, uint mode);
/// @brief Read analog value of GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @return [0:1023] ADC measurement
uint x4_gpio_analog_read(uint pinID);
/// @brief Write analog value (PWM) to GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param value [0:20] duty cycle
void x4_gpio_analog_write(uint pinID, uint value);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_ROBOBOARD_X4_H */
