/*
 * Copyright (c) 2020-2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_input_gpio_direct

#if CONFIG_ZMK_INPUT_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_INPUT_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n)                                                                  \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_INPUT_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_INPUT_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n)                                                                \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

#define USE_POLLING IS_ENABLED(CONFIG_ZMK_INPUT_DIRECT_POLLING)
#define USE_INTERRUPTS (!USE_POLLING)

#define COND_INTERRUPTS(code) COND_CODE_1(CONFIG_ZMK_INPUT_DIRECT_POLLING, (), code)

#define INST_INPUTS_LEN(n)                                                                         \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, input_gpios), (DT_INST_PROP_LEN(n, input_gpios)),         \
                (DT_INST_PROP_LEN(n, input_keys)))

struct input_direct_gpio {
    struct gpio_dt_spec spec;
    size_t index;
};

#define INPUT_DIRECT_GPIO_GET_BY_IDX(node_id, prop, idx)                                           \
    ((struct input_direct_gpio){.spec = GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx), .index = idx})

#define INPUT_DIRECT_INPUT_CFG_INIT(idx, inst_idx)                                                 \
    INPUT_DIRECT_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), input_gpios, idx)
#define INPUT_DIRECT_KEY_CFG_INIT(idx, inst_idx)                                                   \
    INPUT_DIRECT_GPIO_GET_BY_IDX(DT_INST_PROP_BY_IDX(inst_idx, input_keys, idx), gpios, 0)

struct input_direct_gpio_list {
    struct input_direct_gpio *gpios;
    size_t len;
};

#define INPUT_DIRECT_GPIO_LIST(gpio_array)                                                         \
    ((struct input_direct_gpio_list){.gpios = gpio_array, .len = ARRAY_SIZE(gpio_array)})

struct input_direct_port_state {
    const struct device *port;
    gpio_port_value_t value;
};

static int compare_ports(const void *a, const void *b) {
    const struct input_direct_gpio *gpio_a = a;
    const struct input_direct_gpio *gpio_b = b;
    return gpio_a->spec.port - gpio_b->spec.port;
}

static void input_direct_gpio_list_sort_by_port(struct input_direct_gpio_list *list) {
    qsort(list->gpios, list->len, sizeof(list->gpios[0]), compare_ports);
}

static int input_direct_gpio_pin_get(const struct input_direct_gpio *gpio,
                                     struct input_direct_port_state *state) {
    if (gpio->spec.port != state->port) {
        state->port = gpio->spec.port;
        const int err = gpio_port_get(state->port, &state->value);
        if (err) {
            return err;
        }
    }
    return (state->value & BIT(gpio->spec.pin)) != 0;
}

struct input_direct_irq_callback {
    const struct device *dev;
    struct gpio_callback callback;
};

struct input_direct_data {
    const struct device *dev;
    struct input_direct_gpio_list inputs;
    struct k_work_delayable work;
#if USE_INTERRUPTS
    struct input_direct_irq_callback *irqs;
#endif
    int64_t scan_time;
    struct zmk_debounce_state *pin_state;
};

struct input_direct_config {
    struct zmk_debounce_config debounce_config;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
    bool toggle_mode;
};

#if USE_INTERRUPTS
static int input_direct_interrupt_configure(const struct device *dev, const gpio_flags_t flags) {
    const struct input_direct_data *data = dev->data;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct gpio_dt_spec *gpio = &data->inputs.gpios[i].spec;

        int err = gpio_pin_interrupt_configure_dt(gpio, flags);
        if (err) {
            LOG_ERR("Unable to configure interrupt for pin %u on %s", gpio->pin, gpio->port->name);
            return err;
        }
    }

    return 0;
}

static int input_direct_interrupt_enable(const struct device *dev) {
    return input_direct_interrupt_configure(dev, GPIO_INT_LEVEL_ACTIVE);
}

static int input_direct_interrupt_disable(const struct device *dev) {
    return input_direct_interrupt_configure(dev, GPIO_INT_DISABLE);
}

static void input_direct_irq_callback_handler(const struct device *port, struct gpio_callback *cb,
                                              const gpio_port_pins_t pin) {
    struct input_direct_irq_callback *irq_data =
        CONTAINER_OF(cb, struct input_direct_irq_callback, callback);
    struct input_direct_data *data = irq_data->dev->data;

    input_direct_interrupt_disable(data->dev);

    data->scan_time = k_uptime_get();

    k_work_reschedule(&data->work, K_NO_WAIT);
}
#endif

static gpio_flags_t input_direct_get_extra_flags(const struct gpio_dt_spec *gpio, bool active) {
    if (!active) {
        return ((BIT(0) & gpio->dt_flags) ? GPIO_PULL_UP : GPIO_PULL_DOWN);
    }
    return 0;
}

static int input_direct_set_flags(const struct input_direct_gpio_list *inputs,
                                  const struct gpio_dt_spec *active_gpio) {
    for (int i = 0; i < inputs->len; i++) {
        const bool active = &inputs->gpios[i].spec == active_gpio;
        const gpio_flags_t extra_flags =
            GPIO_INPUT | input_direct_get_extra_flags(&inputs->gpios[i].spec, active);

        int err = gpio_pin_configure_dt(&inputs->gpios[i].spec, extra_flags);
        if (err) {
            LOG_ERR("Unable to configure flags on pin %d on %s", inputs->gpios[i].spec.pin,
                    inputs->gpios[i].spec.port->name);
            return err;
        }
    }
    return 0;
}

static void input_direct_read_continue(const struct device *dev) {
    const struct input_direct_config *config = dev->config;
    struct input_direct_data *data = dev->data;

    data->scan_time += config->debounce_scan_period_ms;

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void input_direct_read_end(const struct device *dev) {
#if USE_INTERRUPTS
    input_direct_interrupt_enable(dev);
#else
    struct input_direct_data *data = dev->data;
    const struct input_direct_config *config = dev->config;

    data->scan_time += config->poll_period_ms;

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
#endif
}

static int input_direct_read(const struct device *dev) {
    struct input_direct_data *data = dev->data;
    const struct input_direct_config *config = dev->config;

    struct input_direct_port_state state = {0};

    for (int i = 0; i < data->inputs.len; i++) {
        const struct input_direct_gpio *gpio = &data->inputs.gpios[i];

        const int active = input_direct_gpio_pin_get(gpio, &state);
        if (active < 0) {
            LOG_ERR("Failed to read port %s: %i", gpio->spec.port->name, active);
            return active;
        }

        zmk_debounce_update(&data->pin_state[gpio->index], active, config->debounce_scan_period_ms,
                            &config->debounce_config);
    }

    bool continue_scan = false;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct input_direct_gpio *gpio = &data->inputs.gpios[i];
        struct zmk_debounce_state *deb_state = &data->pin_state[gpio->index];

        if (zmk_debounce_get_changed(deb_state)) {
            const bool pressed = zmk_debounce_is_pressed(deb_state);

            LOG_DBG("Sending event at 0,%i state %s", gpio->index, pressed ? "on" : "off");
            input_report_abs(dev, INPUT_ABS_X, gpio->index, false, K_FOREVER);
            input_report_abs(dev, INPUT_ABS_Y, 0, false, K_FOREVER);
            input_report_key(dev, INPUT_BTN_TOUCH, pressed, true, K_FOREVER);

            if (config->toggle_mode && pressed) {
                input_direct_set_flags(&data->inputs, &gpio->spec);
            }
        }

        continue_scan = continue_scan || zmk_debounce_is_active(deb_state);
    }

    if (continue_scan) {
        input_direct_read_continue(dev);
    } else {
        input_direct_read_end(dev);
    }

    return 0;
}

static void input_direct_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct input_direct_data *data = CONTAINER_OF(dwork, struct input_direct_data, work);
    input_direct_read(data->dev);
}

static int input_direct_enable(const struct device *dev) {
    struct input_direct_data *data = dev->data;

    data->scan_time = k_uptime_get();

    return input_direct_read(dev);
}

static int input_direct_init_input_inst(const struct device *dev,
                                        const struct input_direct_gpio *gpio, bool toggle_mode) {
    if (!device_is_ready(gpio->spec.port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->spec.port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(
        &gpio->spec,
        GPIO_INPUT | (toggle_mode ? input_direct_get_extra_flags(&gpio->spec, false) : 0));
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for input", gpio->spec.pin,
                gpio->spec.port->name);
        return err;
    }

    LOG_DBG("Configured pin %u on %s for input", gpio->spec.pin, gpio->spec.port->name);

#if USE_INTERRUPTS
    struct input_direct_data *data = dev->data;
    struct input_direct_irq_callback *irq = &data->irqs[gpio->index];

    irq->dev = dev;
    gpio_init_callback(&irq->callback, input_direct_irq_callback_handler, BIT(gpio->spec.pin));
    err = gpio_add_callback(gpio->spec.port, &irq->callback);
    if (err) {
        LOG_ERR("Error adding the callback to the input device: %i", err);
        return err;
    }
#endif

    return 0;
}

static int input_direct_init_inputs(const struct device *dev) {
    const struct input_direct_data *data = dev->data;
    const struct input_direct_config *config = dev->config;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct input_direct_gpio *gpio = &data->inputs.gpios[i];
        int err = input_direct_init_input_inst(dev, gpio, config->toggle_mode);
        if (err) {
            return err;
        }
    }

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int input_direct_disconnect_inputs(const struct device *dev) {
    const struct input_direct_data *data = dev->data;

    for (int i = 0; i < data->inputs.len; i++) {
        const struct gpio_dt_spec *gpio = &data->inputs.gpios[i].spec;
        int err = gpio_pin_configure_dt(gpio, GPIO_DISCONNECTED);
        if (err) {
            return err;
        }
    }

    return 0;
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static int input_direct_init(const struct device *dev) {
    struct input_direct_data *data = dev->data;

    data->dev = dev;

    input_direct_gpio_list_sort_by_port(&data->inputs);

    k_work_init_delayable(&data->work, input_direct_work_handler);

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);

    // A `wakeup-source` DT property only marks this device wakeup-*capable*;
    // it must be wakeup-*enabled* for zmk_pm_suspend_devices() to leave it
    // armed (its GPIO interrupt/SENSE configured) across deep sleep
    // (sys_poweroff), so a key press can wake the SoC. The classic kscan
    // stack gets this enable from physical_layouts.c, but that hook keys off
    // the layout's `kscan` device and never reaches an input-stack source --
    // wrappers like input-deghost/input-composite sit between the layout and
    // this GPIO source. So arm ourselves here when declared a wakeup source.
    if (pm_device_wakeup_is_capable(dev)) {
        pm_device_wakeup_enable(dev, true);
    }

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#else
    // Arm through the PM hook rather than calling input_direct_init_inputs/
    // input_direct_enable directly: init_suspended() above set the PM state
    // to SUSPENDED, and arming behind its back leaves it there while the
    // hardware is live. A later pm_device_action_run(SUSPEND) -- e.g. a
    // System OFF entry that must NOT wake on keys -- then returns -EALREADY
    // without ever running the hook, and SENSE stays armed.
    pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
#endif

#else
    input_direct_init_inputs(dev);
    input_direct_enable(dev);
#endif

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int input_direct_disable(const struct device *dev) {
    struct input_direct_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

#if USE_INTERRUPTS
    return input_direct_interrupt_disable(dev);
#else
    ARG_UNUSED(data);
    return 0;
#endif
}

static int input_direct_pm_action(const struct device *dev, enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        input_direct_disconnect_inputs(dev);
        return input_direct_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        input_direct_init_inputs(dev);
        return input_direct_enable(dev);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

#define INPUT_DIRECT_INIT(n)                                                                       \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                \
                 "ZMK_INPUT_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                              \
                 "ZMK_INPUT_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
                                                                                                   \
    static struct input_direct_gpio input_direct_inputs_##n[] = {                                  \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, input_gpios),                                         \
                    (LISTIFY(INST_INPUTS_LEN(n), INPUT_DIRECT_INPUT_CFG_INIT, (, ), n)),           \
                    (LISTIFY(INST_INPUTS_LEN(n), INPUT_DIRECT_KEY_CFG_INIT, (, ), n)))};           \
                                                                                                   \
    static struct zmk_debounce_state input_direct_state_##n[INST_INPUTS_LEN(n)];                   \
                                                                                                   \
    COND_INTERRUPTS(                                                                               \
        (static struct input_direct_irq_callback input_direct_irqs_##n[INST_INPUTS_LEN(n)];))      \
                                                                                                   \
    static struct input_direct_data input_direct_data_##n = {                                      \
        .inputs = INPUT_DIRECT_GPIO_LIST(input_direct_inputs_##n),                                 \
        .pin_state = input_direct_state_##n,                                                       \
        COND_INTERRUPTS((.irqs = input_direct_irqs_##n, ))};                                       \
                                                                                                   \
    static const struct input_direct_config input_direct_config_##n = {                            \
        .debounce_config =                                                                         \
            {                                                                                      \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                    \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                \
            },                                                                                     \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                       \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
        .toggle_mode = DT_INST_PROP(n, toggle_mode),                                               \
    };                                                                                             \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, input_direct_pm_action);                                           \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &input_direct_init, PM_DEVICE_DT_INST_GET(n), &input_direct_data_##n, \
                          &input_direct_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(INPUT_DIRECT_INIT);
