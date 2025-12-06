/*
 * Copyright (c) 2020-2021 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "kscan_gpio.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_kscan_gpio_adc

#define INST_ADCS_LEN(n) DT_INST_PROP_LEN(n, adc_gpios)
#define INST_COLS_LEN(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_ROWS_LEN(n) 8
#define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COLS_LEN(n))
#define INST_INPUTS_LEN(n) INST_COLS_LEN(n)

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n)                                                                  \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n)                                                                \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

#define USE_POLLING IS_ENABLED(CONFIG_ZMK_KSCAN_MATRIX_POLLING)

#define KSCAN_GPIO_ADC_CFG_INIT(idx, inst_idx)                                                     \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), adc_gpios, idx)
#define KSCAN_GPIO_COL_CFG_INIT(idx, inst_idx)                                                     \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), col_gpios, idx)

struct kscan_matrix_irq_callback {
    const struct device *dev;
    struct gpio_callback callback;
};

struct kscan_matrix_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    const struct device *adc;
    // struct adc_channel_cfg acc;
    // struct adc_sequence as;
    /** Timestamp of the current or scheduled scan. */
    int64_t scan_time;
    /**
     * Current state of the matrix as a flattened 2D array of length
     * (config->rows * config->cols)
     */
    struct zmk_debounce_state *matrix_state;
};

struct kscan_matrix_config {
    struct kscan_gpio_list outputs;
    struct kscan_gpio_list adc_gpios;
    struct zmk_debounce_config debounce_config;
    uint8_t io_channel;
    const struct gpio_dt_spec mux_enable;
    size_t rows;
    size_t cols;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
};

/**
 * Get the index into a matrix state array from a row and column.
 */
static int state_index_rc(const struct kscan_matrix_config *config, const int row, const int col) {
    __ASSERT(row < config->rows, "Invalid row %i", row);
    __ASSERT(col < config->cols, "Invalid column %i", col);

    return (col * config->rows) + row;
}

/**
 * Get the index into a matrix state array from input/output pin indices.
 */
static int state_index_io(const struct kscan_matrix_config *config, const int input_idx,
                          const int output_idx) {
    return state_index_rc(config, output_idx, input_idx);
}

static int kscan_matrix_set_all_outputs(const struct device *dev, const int value) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;

        int err = gpio_pin_set_dt(gpio, value);
        if (err) {
            LOG_ERR("Failed to set output %i to %i: %i", i, value, err);
            return err;
        }
    }

    return 0;
}


static void kscan_matrix_read_continue(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;
    struct kscan_matrix_data *data = dev->data;

    data->scan_time += config->debounce_scan_period_ms;

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void kscan_matrix_read_end(const struct device *dev) {
#if USE_INTERRUPTS
    // Return to waiting for an interrupt.
    kscan_matrix_interrupt_enable(dev);
#else
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    data->scan_time += config->poll_period_ms;

    // Return to polling slowly.
    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
#endif
}

static void mux_set_row(const struct device *dev, int row) {
    const struct kscan_matrix_config *config = dev->config;
    // TODO: Rows and mux indeces may not map one to one
    gpio_pin_set_dt(&config->adc_gpios.gpios[0].spec, row & 1);
    gpio_pin_set_dt(&config->adc_gpios.gpios[1].spec, row & 2);
    gpio_pin_set_dt(&config->adc_gpios.gpios[2].spec, row & 4);
}

static int kscan_matrix_read(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    // Scan the matrix.
    for (int i = 0; i < config->outputs.len; i++) {
        const struct kscan_gpio *out_gpio = &config->outputs.gpios[i];

        int err = gpio_pin_set_dt(&out_gpio->spec, 1);
        if (err) {
            LOG_ERR("Failed to set output %i active: %i", out_gpio->index, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BEFORE_INPUTS);
#endif
        struct kscan_gpio_port_state state = {0};

	// TODO: Maybe dynamically determine row number if fewer than 8
        for (int row = 0; row < config->rows; row++) {
		mux_set_row(dev, row);
		// TODO: Read ADC value and threshold with 2.9V
		// TODO: Advanced check how many keys are under threshold
        //     const struct kscan_gpio *in_gpio = &data->inputs.gpios[j];

        //     const int index = state_index_io(config, in_gpio->index, out_gpio->index);
        //     const int active = kscan_gpio_pin_get(in_gpio, &state);
        //     if (active < 0) {
        //         LOG_ERR("Failed to read port %s: %i", in_gpio->spec.port->name, active);
        //         return active;
        //     }

        //     zmk_debounce_update(&data->matrix_state[index], active, config->debounce_scan_period_ms,
        //                         &config->debounce_config);
        }

        err = gpio_pin_set_dt(&out_gpio->spec, 0);
        if (err) {
            LOG_ERR("Failed to set output %i inactive: %i", out_gpio->index, err);
            return err;
        }

#if CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_MATRIX_WAIT_BETWEEN_OUTPUTS);
#endif
    }

    // Process the new state.
    bool continue_scan = false;

    for (int r = 0; r < config->rows; r++) {
        for (int c = 0; c < config->cols; c++) {
            const int index = state_index_rc(config, r, c);
            struct zmk_debounce_state *state = &data->matrix_state[index];

            if (zmk_debounce_get_changed(state)) {
                const bool pressed = zmk_debounce_is_pressed(state);

                LOG_DBG("Sending event at %i,%i state %s", r, c, pressed ? "on" : "off");
                data->callback(dev, r, c, pressed);
            }

            continue_scan = continue_scan || zmk_debounce_is_active(state);
        }
    }

    if (continue_scan) {
        // At least one key is pressed or the debouncer has not yet decided if
        // it is pressed. Poll quickly until everything is released.
        kscan_matrix_read_continue(dev);
    } else {
        // All keys are released. Return to normal.
        kscan_matrix_read_end(dev);
    }

    return 0;
}

static void kscan_matrix_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_matrix_data *data = CONTAINER_OF(dwork, struct kscan_matrix_data, work);
    kscan_matrix_read(data->dev);
}

static int kscan_matrix_configure(const struct device *dev, const kscan_callback_t callback) {
    struct kscan_matrix_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    return 0;
}

static int kscan_matrix_enable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    data->scan_time = k_uptime_get();

    // Read will automatically start interrupts/polling once done.
    return kscan_matrix_read(dev);
}

static int kscan_matrix_disable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

    return 0;
}

static int kscan_matrix_init_input_inst(const struct device *dev, const struct kscan_gpio *gpio) {
    if (!device_is_ready(gpio->spec.port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->spec.port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&gpio->spec, GPIO_INPUT);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for input", gpio->spec.pin,
                gpio->spec.port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for input", gpio->spec.pin, gpio->spec.port->name);

    return 0;
}

static int kscan_matrix_init_inputs(const struct device *dev) {
    const struct kscan_matrix_data *data = dev->data;

    // TODO: Init ADC

    return 0;
}

static int kscan_matrix_init_output_inst(const struct device *dev,
                                         const struct gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for output", gpio->pin, gpio->port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for output", gpio->pin, gpio->port->name);

    return 0;
}

static int kscan_matrix_init_outputs(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;
        int err = kscan_matrix_init_output_inst(dev, gpio);
        if (err) {
            return err;
        }
    }

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_matrix_disconnect_inputs(const struct device *dev) {
    const struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;

    gpio_pin_set_dt(&config->mux_enable, 1);

    // TODO: Disable ADC

    return 0;
}

static int kscan_matrix_disconnect_outputs(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;

    for (int i = 0; i < config->outputs.len; i++) {
        const struct gpio_dt_spec *gpio = &config->outputs.gpios[i].spec;
        int err = gpio_pin_configure_dt(gpio, GPIO_DISCONNECTED);
        if (err) {
            return err;
        }
    }

    return 0;
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_matrix_setup_pins(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;
    const struct gpio_dt_spec *mux_enable = &config->mux_enable;
    kscan_matrix_init_inputs(dev);
    kscan_matrix_init_outputs(dev);
    kscan_matrix_set_all_outputs(dev, 0);

    if (!device_is_ready(mux_enable->port)) {
        LOG_ERR("mux_enable GPIO is not ready");
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(mux_enable, GPIO_OUTPUT);
    if (err) {
        LOG_ERR("Unable to configure mux_enable pin for output");
        return err;
    }
    gpio_pin_set_dt(&config->mux_enable, 0);
    return 0;
}

static int kscan_matrix_init(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;

    data->dev = dev;

    k_work_init_delayable(&data->work, kscan_matrix_work_handler);

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif

#else
    kscan_matrix_setup_pins(dev);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_matrix_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        kscan_matrix_disconnect_inputs(dev);
        kscan_matrix_disconnect_outputs(dev);

        return kscan_matrix_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        kscan_matrix_setup_pins(dev);
        return kscan_matrix_enable(dev);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static const struct kscan_driver_api kscan_matrix_api = {
    .config = kscan_matrix_configure,
    .enable_callback = kscan_matrix_enable,
    .disable_callback = kscan_matrix_disable,
};

#define KSCAN_MATRIX_INIT(n)                                                                       \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                \
                 "ZMK_KSCAN_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                              \
                 "ZMK_KSCAN_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
                                                                                                   \
    static struct kscan_gpio kscan_adc_gpios_##n[] = {                                             \
        LISTIFY(INST_ADCS_LEN(n), KSCAN_GPIO_ADC_CFG_INIT, (, ), n)};                              \
                                                                                                   \
    static struct kscan_gpio kscan_matrix_cols_##n[] = {                                           \
        LISTIFY(INST_COLS_LEN(n), KSCAN_GPIO_COL_CFG_INIT, (, ), n)};                              \
                                                                                                   \
    static struct zmk_debounce_state kscan_matrix_state_##n[INST_MATRIX_LEN(n)];                   \
                                                                                                   \
    static struct kscan_matrix_data kscan_matrix_data_##n = {                                      \
        /*.adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(DT_DRV_INST(n))),*/                                 \
        .matrix_state = kscan_matrix_state_##n                                                     \
    };                                                                                             \
                                                                                                   \
    static const struct kscan_matrix_config kscan_matrix_config_##n = {                            \
        .io_channel = DT_IO_CHANNELS_INPUT(DT_DRV_INST(n)),                                        \
        .mux_enable = GPIO_DT_SPEC_INST_GET(n, mux_enable_gpios),                                  \
        .cols = ARRAY_SIZE(kscan_matrix_cols_##n),                                                 \
        .adc_gpios =                                                                               \
            KSCAN_GPIO_LIST(kscan_adc_gpios_##n),                                                  \
        .outputs =                                                                                 \
            KSCAN_GPIO_LIST(kscan_matrix_cols_##n),                                                \
        .debounce_config =                                                                         \
            {                                                                                      \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                    \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                \
            },                                                                                     \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                       \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
    };                                                                                             \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_matrix_pm_action);                                           \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &kscan_matrix_init, PM_DEVICE_DT_INST_GET(n), &kscan_matrix_data_##n, \
                          &kscan_matrix_config_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,       \
                          &kscan_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_MATRIX_INIT);
