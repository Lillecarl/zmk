/*
 * Copyright (c) 2020-2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_composite

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct input_composite_child {
    const struct device *composite;
    uint32_t row_offset;
    uint32_t col_offset;
    uint32_t pending_row;
    uint32_t pending_col;
    bool pending_pressed;
};

struct input_composite_config {
    struct input_composite_child *children;
    size_t children_len;
};

static void input_composite_child_cb(struct input_event *evt, void *user_data) {
    struct input_composite_child *child = user_data;

    switch (evt->type) {
    case INPUT_EV_ABS:
        switch (evt->code) {
        case INPUT_ABS_X:
            child->pending_col = evt->value;
            break;
        case INPUT_ABS_Y:
            child->pending_row = evt->value;
            break;
        default:
            break;
        }
        break;
    case INPUT_EV_KEY:
        if (evt->code == INPUT_BTN_TOUCH) {
            child->pending_pressed = evt->value;
        }
        break;
    default:
        break;
    }

    if (!evt->sync) {
        return;
    }

    const uint32_t row = child->pending_row + child->row_offset;
    const uint32_t col = child->pending_col + child->col_offset;

    input_report_abs(child->composite, INPUT_ABS_X, col, false, K_FOREVER);
    input_report_abs(child->composite, INPUT_ABS_Y, row, false, K_FOREVER);
    input_report_key(child->composite, INPUT_BTN_TOUCH, child->pending_pressed, true, K_FOREVER);
}

static int input_composite_init(const struct device *dev) {
    const struct input_composite_config *cfg = dev->config;

    for (size_t i = 0; i < cfg->children_len; i++) {
        cfg->children[i].composite = dev;
    }

    return 0;
}

#define CHILD_STATE_INIT(child_node)                                                               \
    {                                                                                              \
        .row_offset = DT_PROP(child_node, row_offset),                                             \
        .col_offset = DT_PROP_OR(child_node, col_offset, DT_PROP(child_node, column_offset)),     \
    }

#define CHILD_CALLBACK_DEFINE(child_node, inst)                                                    \
    INPUT_CALLBACK_DEFINE_NAMED(                                                                   \
        DEVICE_DT_GET(DT_PHANDLE(child_node, input)), input_composite_child_cb,                    \
        &input_composite_children_##inst[DT_NODE_CHILD_IDX(child_node)],                           \
        child_node##_input_composite_cb);

#define INPUT_COMPOSITE_INIT(n)                                                                    \
    static struct input_composite_child input_composite_children_##n[] = {                         \
        DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, CHILD_STATE_INIT, (, ))};                         \
                                                                                                   \
    DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, CHILD_CALLBACK_DEFINE, n)                           \
                                                                                                   \
    static const struct input_composite_config input_composite_config_##n = {                      \
        .children = input_composite_children_##n,                                                  \
        .children_len = ARRAY_SIZE(input_composite_children_##n),                                  \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, input_composite_init, NULL, NULL, &input_composite_config_##n,        \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_COMPOSITE_INIT);
