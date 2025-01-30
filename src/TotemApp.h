/* 
 * Copyright 2025 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEMAPP
#define INCLUDE_TOTEMAPP

#include "private/totem-board.h"
#include "private/totem-totemapp.h"
#include "private/drivers/os/os_ble.h"

namespace _RoboBoard {

struct TotemAppBuilder {
static TotemAppClass& getInstance() {
        static ObBleDriver _bleDriver;
        static TotemAppClass _totemApp(&_bleDriver);
        return _totemApp;
    }
};

} // namespace _RoboBoard

inline _RoboBoard::TotemAppClass& __returnTotemApp() {
    Board.begin(); // Make sure board is initialized
    return _RoboBoard::TotemAppBuilder::getInstance();
}

#define TotemApp __returnTotemApp()

#endif /* INCLUDE_TOTEMAPP */
