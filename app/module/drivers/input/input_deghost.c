/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_deghost

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <dt-bindings/zmk/matrix_transform.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define KEY_STATUS_FORCE_NOT_RECHECK        (1 << 2)
#define KEY_STATUS_REPORTED_AS_PRESSED_MASK (1 << 1)
#define KEY_STATUS_SEEN_AS_PRESSED_MASK     (1 << 0)
#define KEY_SEEN_AS_PRESSED(v)              ((v) & KEY_STATUS_SEEN_AS_PRESSED_MASK)
#define KEY_WAS_GHOSTING(v)                 ((v) == KEY_STATUS_SEEN_AS_PRESSED_MASK)

struct input_deghost_config {
    size_t rows;
    size_t cols;
    const uint8_t *transform_filled;
    size_t transform_filled_len;
    uint8_t *key_status;
};

struct input_deghost_data {
    const struct device *dev;
    uint32_t pending_row;
    uint32_t pending_col;
    bool pending_pressed;
};

#define KEY(cfg, row, col) ((cfg)->key_status[(row) * (cfg)->cols + (col)])

static bool key_exists(const struct input_deghost_config *cfg, uint32_t row, uint32_t col) {
    if (cfg->transform_filled_len == 0) {
        return true;
    }
    const size_t idx = row * cfg->cols + col;
    if (idx >= cfg->transform_filled_len) {
        return false;
    }
    return cfg->transform_filled[idx] != 0;
}

static void emit(const struct device *dev, uint32_t row, uint32_t col, bool pressed) {
    input_report_abs(dev, INPUT_ABS_X, col, false, K_FOREVER);
    input_report_abs(dev, INPUT_ABS_Y, row, false, K_FOREVER);
    input_report_key(dev, INPUT_BTN_TOUCH, pressed, true, K_FOREVER);
}

static void process(const struct device *dev, uint32_t row, uint32_t col, bool pressed) {
    const struct input_deghost_config *cfg = dev->config;

    if (!key_exists(cfg, row, col)) {
        return;
    }

    if (pressed) {
        KEY(cfg, row, col) |= KEY_STATUS_SEEN_AS_PRESSED_MASK;
        bool ghosting = false;
        for (uint32_t orow = 0; orow < cfg->rows && !ghosting; orow++) {
            if (orow == row || !key_exists(cfg, orow, col)) {
                continue;
            }
            const int other_row_pressed = KEY_SEEN_AS_PRESSED(KEY(cfg, orow, col));
            for (uint32_t ocol = 0; ocol < cfg->cols; ocol++) {
                if (ocol == col || !key_exists(cfg, row, ocol) ||
                    !key_exists(cfg, orow, ocol)) {
                    continue;
                }
                const int pressed_in_rectangle = 1 + other_row_pressed +
                                                 KEY_SEEN_AS_PRESSED(KEY(cfg, row, ocol)) +
                                                 KEY_SEEN_AS_PRESSED(KEY(cfg, orow, ocol));
                if (pressed_in_rectangle > 2) {
                    ghosting = true;
                    break;
                }
            }
        }
        if (!ghosting) {
            KEY(cfg, row, col) |= KEY_STATUS_REPORTED_AS_PRESSED_MASK;
            emit(dev, row, col, true);
        }
    } else {
        if (KEY(cfg, row, col) & KEY_STATUS_REPORTED_AS_PRESSED_MASK) {
            KEY(cfg, row, col) = 0;
            emit(dev, row, col, false);
        } else {
            KEY(cfg, row, col) = 0;
        }
        for (uint32_t orow = 0; orow < cfg->rows; orow++) {
            if (orow == row || !key_exists(cfg, orow, col)) {
                continue;
            }
            const int other_row_pressed = KEY_SEEN_AS_PRESSED(KEY(cfg, orow, col));
            int check_orow_col = KEY_WAS_GHOSTING(KEY(cfg, orow, col));
            for (uint32_t ocol = 0; ocol < cfg->cols; ocol++) {
                if (ocol == col || !key_exists(cfg, row, ocol) ||
                    !key_exists(cfg, orow, ocol)) {
                    continue;
                }
                const int pressed_in_rectangle = 0 + other_row_pressed +
                                                 KEY_SEEN_AS_PRESSED(KEY(cfg, row, ocol)) +
                                                 KEY_SEEN_AS_PRESSED(KEY(cfg, orow, ocol));
                if (pressed_in_rectangle == 2) {
                    if (KEY_WAS_GHOSTING(KEY(cfg, row, ocol))) {
                        process(dev, row, ocol, true);
                        // Next KEY_WAS_GHOSTING() check on this cell must return false.
                        KEY(cfg, row, ocol) |= KEY_STATUS_FORCE_NOT_RECHECK;
                    }
                    if (KEY_WAS_GHOSTING(KEY(cfg, orow, ocol))) {
                        process(dev, orow, ocol, true);
                    }
                    if (check_orow_col) {
                        process(dev, orow, col, true);
                        check_orow_col = 0;
                    }
                }
            }
        }
        for (uint32_t ocol = 0; ocol < cfg->cols; ocol++) {
            KEY(cfg, row, ocol) &= ~KEY_STATUS_FORCE_NOT_RECHECK;
        }
    }
}

static void input_deghost_cb(struct input_event *evt, void *user_data) {
    struct input_deghost_data *data = user_data;

    switch (evt->type) {
    case INPUT_EV_ABS:
        switch (evt->code) {
        case INPUT_ABS_X:
            data->pending_col = evt->value;
            break;
        case INPUT_ABS_Y:
            data->pending_row = evt->value;
            break;
        default:
            break;
        }
        break;
    case INPUT_EV_KEY:
        if (evt->code == INPUT_BTN_TOUCH) {
            data->pending_pressed = evt->value;
        }
        break;
    default:
        break;
    }

    if (!evt->sync) {
        return;
    }

    process(data->dev, data->pending_row, data->pending_col, data->pending_pressed);
}

static int input_deghost_init(const struct device *dev) {
    struct input_deghost_data *data = dev->data;
    data->dev = dev;
    return 0;
}

#define _TRANSFORM_ENTRY(i, transform_node)                                                        \
    [(KT_ROW(DT_PROP_BY_IDX(transform_node, map, i)) * DT_PROP(transform_node, columns)) +         \
     KT_COL(DT_PROP_BY_IDX(transform_node, map, i))] = 1

#define INPUT_DEGHOST_ROWS(n)                                                                      \
    DT_INST_PROP_OR(n, rows, DT_PROP_OR(DT_INST_PHANDLE(n, transform), rows, 0))
#define INPUT_DEGHOST_COLS(n)                                                                      \
    DT_INST_PROP_OR(n, columns, DT_PROP_OR(DT_INST_PHANDLE(n, transform), columns, 0))

#define INPUT_DEGHOST_INIT(n)                                                                      \
    BUILD_ASSERT(INPUT_DEGHOST_ROWS(n) > 0,                                                        \
                 "zmk,input-deghost needs `rows` or a `transform` that provides rows");            \
    BUILD_ASSERT(INPUT_DEGHOST_COLS(n) > 0,                                                        \
                 "zmk,input-deghost needs `columns` or a `transform` that provides columns");      \
                                                                                                   \
    static const uint8_t input_deghost_transform_##n[] = {COND_CODE_1(                             \
        DT_INST_NODE_HAS_PROP(n, transform),                                                       \
        (LISTIFY(DT_PROP_LEN(DT_INST_PHANDLE(n, transform), map), _TRANSFORM_ENTRY, (, ),          \
                 DT_INST_PHANDLE(n, transform))),                                                  \
        ())};                                                                                      \
                                                                                                   \
    static uint8_t input_deghost_key_status_##n[INPUT_DEGHOST_ROWS(n) * INPUT_DEGHOST_COLS(n)];    \
                                                                                                   \
    static struct input_deghost_data input_deghost_data_##n;                                       \
                                                                                                   \
    static const struct input_deghost_config input_deghost_config_##n = {                          \
        .rows = INPUT_DEGHOST_ROWS(n),                                                             \
        .cols = INPUT_DEGHOST_COLS(n),                                                             \
        .transform_filled = input_deghost_transform_##n,                                           \
        .transform_filled_len = ARRAY_SIZE(input_deghost_transform_##n),                           \
        .key_status = input_deghost_key_status_##n,                                                \
    };                                                                                             \
                                                                                                   \
    INPUT_CALLBACK_DEFINE_NAMED(DEVICE_DT_GET(DT_INST_PHANDLE(n, input)), input_deghost_cb,        \
                                &input_deghost_data_##n, input_deghost_cb_##n);                    \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, input_deghost_init, NULL, &input_deghost_data_##n,                    \
                          &input_deghost_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,      \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_DEGHOST_INIT);
