/*
 * Copyright (c) 2023-2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_sideband_behaviors

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct isbb_entry {
    struct zmk_behavior_binding binding;
    uint8_t row;
    uint8_t column;
};

struct isbb_state {
    uint32_t pending_row;
    uint32_t pending_col;
    bool pending_pressed;
    bool pending_pressed_seen;
};

struct isbb_config {
    struct isbb_entry *entries;
    size_t entries_len;
    struct isbb_state *state;
};

static struct isbb_entry *isbb_find_entry(const struct isbb_config *cfg, uint32_t row,
                                          uint32_t column) {
    for (size_t e = 0; e < cfg->entries_len; e++) {
        struct isbb_entry *candidate = &cfg->entries[e];
        if (candidate->row == row && candidate->column == column) {
            return candidate;
        }
    }
    return NULL;
}

static void isbb_input_cb(struct input_event *evt, void *user_data) {
    const struct device *dev = user_data;
    const struct isbb_config *cfg = dev->config;
    struct isbb_state *state = cfg->state;

    switch (evt->type) {
    case INPUT_EV_ABS:
        switch (evt->code) {
        case INPUT_ABS_X:
            state->pending_col = evt->value;
            break;
        case INPUT_ABS_Y:
            state->pending_row = evt->value;
            break;
        default:
            break;
        }
        break;
    case INPUT_EV_KEY:
        if (evt->code == INPUT_BTN_TOUCH) {
            state->pending_pressed = evt->value;
            state->pending_pressed_seen = true;
        }
        break;
    default:
        break;
    }

    if (!evt->sync || !state->pending_pressed_seen) {
        return;
    }
    state->pending_pressed_seen = false;

    struct isbb_entry *entry = isbb_find_entry(cfg, state->pending_row, state->pending_col);
    if (!entry) {
        return;
    }

    struct zmk_behavior_binding_event event = {
        .position = INT32_MAX,
        .timestamp = k_uptime_get(),
    };

    if (state->pending_pressed) {
        behavior_keymap_binding_pressed(&entry->binding, event);
    } else {
        behavior_keymap_binding_released(&entry->binding, event);
    }
}

static int isbb_init(const struct device *dev) { return 0; }

#define ENTRY(e)                                                                                   \
    {                                                                                              \
        .row = DT_PROP(e, row),                                                                    \
        .column = DT_PROP(e, column),                                                              \
        .binding = ZMK_KEYMAP_EXTRACT_BINDING(0, e),                                               \
    }

#define ISBB_INST(n)                                                                               \
    static struct isbb_entry isbb_entries_##n[] = {                                                \
        DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, ENTRY, (, ))};                                    \
    static struct isbb_state isbb_state_##n;                                                       \
    static const struct isbb_config isbb_config_##n = {                                            \
        .entries = isbb_entries_##n,                                                               \
        .entries_len = ARRAY_SIZE(isbb_entries_##n),                                               \
        .state = &isbb_state_##n,                                                                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, isbb_init, NULL, NULL, &isbb_config_##n, POST_KERNEL,                 \
                          CONFIG_ZMK_INPUT_SIDEBAND_BEHAVIORS_INIT_PRIORITY, NULL);                \
    INPUT_CALLBACK_DEFINE_NAMED(DEVICE_DT_GET(DT_INST_PHANDLE(n, input)), isbb_input_cb,           \
                                (void *)DEVICE_DT_INST_GET(n), isbb_cb_##n);

DT_INST_FOREACH_STATUS_OKAY(ISBB_INST)
