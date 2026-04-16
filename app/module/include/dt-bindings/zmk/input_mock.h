/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <dt-bindings/input/input-event-codes.h>

/*
 * Convenience macros for zmk,input-mock events.
 *
 * Each press/release expands to three input events (5 elements each):
 *   ABS_X (column), ABS_Y (row), BTN_TOUCH (pressed/released + sync + delay)
 *
 * Usage in DTS:
 *   events = <
 *       ZMK_INPUT_MOCK_PRESS(0,0,500)
 *       ZMK_INPUT_MOCK_RELEASE(0,0,10)
 *   >;
 */

#define ZMK_INPUT_MOCK_PRESS(row, col, ms) \
    INPUT_EV_ABS INPUT_ABS_X (col) 0 0 \
    INPUT_EV_ABS INPUT_ABS_Y (row) 0 0 \
    INPUT_EV_KEY INPUT_BTN_TOUCH 1 1 (ms)

#define ZMK_INPUT_MOCK_RELEASE(row, col, ms) \
    INPUT_EV_ABS INPUT_ABS_X (col) 0 0 \
    INPUT_EV_ABS INPUT_ABS_Y (row) 0 0 \
    INPUT_EV_KEY INPUT_BTN_TOUCH 0 1 (ms)
