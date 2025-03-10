/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_TOTEM_BUTTON
#define INCLUDE_TOTEM_BUTTON

#include "sys/types.h"

namespace _RoboBoard {

struct ButtonStatus {
    uint8_t pin;
    const uint8_t pressState;
    void (*evtFunc)();
    void (*evtFuncArg)(int state);
    uint eventPending;
    uint eventHistory;
    uint lastState;
    uint lastTime;
    uint debounceTime;
    uint longPressTime;
};

} // namespace _RoboBoard

namespace totem {

class IOButton {

public:

    /// @brief Create button control interface
    /// @param pin GPIO pin number
    /// @param pressState button pin state when pressed [LOW:HIGH]
    IOButton(uint8_t pin, uint8_t pressState = 0);
    
    ///////////////////////
    //  Read button state
    ///////////////////////

    /// @brief Is button pressed in
    /// @return [true:false] is pressed
    bool isPressed();
    /// @brief Is button not pressed
    /// @return [true:false] not pressed
    bool isReleased();
    // Is pressed for a certain period of time (long press)
    // `ms`: time after button press (milliseconds)
    bool isPressedFor(uint ms);
    /// @brief Is released for a certain period of time (cooldown)
    /// @return time after button release (milliseconds)
    bool isReleasedFor(uint ms);
    /// @brief Is pressed for 500ms
    /// @return [true:false] is long press
    bool isLongPress();
    /// @brief Read button state from pin
    /// @return [true:false] is pressed
    int getState();
    
    ///////////////////////
    //  Read button event
    ///////////////////////
    /*
     * Check current or previous button state. Returns true only once.
     * Convenient to use inside loop.
    */

    /// @brief Was pressed earlier?
    /// @return [true:false]
    bool wasPressed();
    /// @brief Was released earlier?
    /// @return [true:false]
    bool wasReleased();
    /// @brief Was clicked single time?
    /// @return [true:false]
    bool wasClick();
    /// @brief Was pressed for 500ms?
    /// @return [true:false]
    bool wasLongPress();
    /// @brief Was double clicked?
    /// @return [true:false]
    bool wasDoubleClick();
    /// @brief Get time passed after last button state change (pressed or released)
    /// @return milliseconds
    uint getStateTime();

    ///////////////////////
    // Detect button event
    ///////////////////////

    /// @brief Wait for button click
    /// @param timeout max wait time. 0 - indefinitely
    /// @return true - button clicked. false - timeout
    bool waitClick(uint timeout = 0);
    /// @brief Wait for button double click
    /// @param timeout max wait time. 0 - indefinitely
    /// @return true - button double clicked. false - timeout
    bool waitDoubleClick(uint timeout = 0);
    /// @brief Wait for button long press
    /// @param timeout max wait time. 0 - indefinitely
    /// @return true - button long pressed. false - timeout
    bool waitLongPress(uint timeout = 0);

    /// @brief Button events
    enum {
        evtRelease,
        evtPress,
        evtClick,
        evtDoubleClick,
        evtLongPress,
    };

    /// @brief Register an event for button state change (press or release)
    /// @param buttonEvt button event function name
    void addEvent(void (*buttonEvt)(int evt));

    void addEvent(void (*buttonEvt)());

    /// @brief Set debounce time (default 20ms)
    /// @param time milliseconds
    void setDebounceTime(uint time);
    /// @brief Set long press time (default 500ms)
    /// @param time milliseconds
    void setLongPressTime(uint time);
private:
    _RoboBoard::ButtonStatus status;
};

extern IOButton Button;

} // namespace totem

#endif /* INCLUDE_TOTEM_BUTTON */
