/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_ROBOBOARD
#define INCLUDE_ROBOBOARD

#include <Arduino.h>

#ifdef __cplusplus

#include "private/totem-battery.h"
#include "private/totem-board.h"
#include "private/totem-button.h"
#include "private/totem-can.h"
#include "private/totem-color.h"
#include "private/totem-drivetrain.h"
#include "private/totem-imu.h"
#include "private/totem-led.h"
#include "private/totem-dc.h"
#include "private/totem-servo.h"
#include "private/totem-rgb.h"

// Create secondary loop task
void addLoop(void (*loopFunc)(void));

// RoboBoard X3 servo SIG pins (can be used as GPIO)
static const uint8_t SIGA = 25;
static const uint8_t SIGB = 14;
static const uint8_t SIGC = 16;
static const uint8_t SIGD = 17;
// RoboBoard X4 v1.1 GPIO pins (for v1.0 use X410_ functions!)
static const uint8_t GPIOA = 14;
static const uint8_t GPIOB = 23;
static const uint8_t GPIOC = 25;
static const uint8_t GPIOD = 26;

// Legacy RoboBoard X4 (revision 1.0) GPIO control functions
// For later boards use standard Arduino functions
void X410_digitalWrite(uint pin, uint value);
int X410_digitalRead(uint pin);
void X410_analogWrite(uint pin, int value);
int X410_analogRead(uint pin);
void X410_pinMode(uint pin, uint mode);

#ifndef REQUIRE_TOTEM_PREFIX
using namespace totem;
#endif // REQUIRE_TOTEM_PREFIX

#endif // __cplusplus

#endif // INCLUDE_ROBOBOARD
