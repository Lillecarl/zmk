/*
 * Copyright (c) 2020-2021 The ZMK Contributors
 * Copyright (c) 2024 Framework Computer (Ported logic)
 *
 * SPDX-License-Identifier: MIT
 */

#include "kscan_gpio.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h> // Nuclear debugging
#include <zmk/debounce.h>

#define DT_DRV_COMPAT zmk_kscan_gpio_adc

// --- CONFIGURATION ---
// Default Thresholds (mV)
#define THRESHOLD_DEFAULT_MV 2900
#define THRESHOLD_1_PRESS_MV 1000

// --- MACROS ---
#define INST_ADCS_LEN(n) DT_INST_PROP_LEN(n, adc_gpios)
#define INST_COLS_LEN(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_ROWS_LEN(n) 8
#define INST_MATRIX_LEN(n) (INST_ROWS_LEN(n) * INST_COLS_LEN(n))

#define KSCAN_GPIO_ADC_CFG_INIT(idx, inst_idx) \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), adc_gpios, idx)
#define KSCAN_GPIO_COL_CFG_INIT(idx, inst_idx) \
    KSCAN_GPIO_GET_BY_IDX(DT_DRV_INST(inst_idx), col_gpios, idx)

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n) DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n) DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

// --- STRUCTURES ---
struct kscan_matrix_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    
    struct adc_channel_cfg adc_cfg;
    struct adc_sequence adc_seq;
    int16_t adc_buffer;

    int64_t scan_time;
    struct zmk_debounce_state *matrix_state;
    
    uint32_t scan_count; 
};

struct kscan_matrix_config {
    struct kscan_gpio_list outputs;
    struct kscan_gpio_list adc_gpios;
    struct zmk_debounce_config debounce_config;
    
    const struct device *adc_dev;
    uint8_t io_channel;
    
    const struct gpio_dt_spec mux_enable;
    size_t rows;
    size_t cols;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
};

// --- HELPERS ---
static int state_index_rc(const struct kscan_matrix_config *config, const int row, const int col) {
    return (col * config->rows) + row;
}

static void mux_set_row(const struct device *dev, int row) {
    const struct kscan_matrix_config *config = dev->config;
    int index = 0;
    // Remap logic from QMK
    switch (row) {
        case 0: index = 2; break;
        case 1: index = 0; break;
        case 2: index = 1; break;
        default: index = row; break;
    }
    // Set A, B, C
    gpio_pin_set_dt(&config->adc_gpios.gpios[0].spec, (index & 0x1) > 0);
    gpio_pin_set_dt(&config->adc_gpios.gpios[1].spec, (index & 0x2) > 0);
    gpio_pin_set_dt(&config->adc_gpios.gpios[2].spec, (index & 0x4) > 0);
}

static int read_adc_mv(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    
    int err = adc_read(config->adc_dev, &data->adc_seq);
    if (err < 0) return 3300; 
    return (data->adc_buffer * 3300) / 4096;
}

static void kscan_matrix_read_continue(const struct device *dev) {
    const struct kscan_matrix_config *config = dev->config;
    struct kscan_matrix_data *data = dev->data;
    data->scan_time += config->debounce_scan_period_ms;
    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void kscan_matrix_read_end(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    data->scan_time += config->poll_period_ms;
    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

// --- SCANNING ---
static int kscan_matrix_read(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    
    if (!device_is_ready(config->adc_dev)) return -ENODEV;

    data->scan_count++;
    uint16_t voltages[8][16]; 
    bool continue_scan = false;

    // --- DIAGNOSTIC MODE (Every ~200 scans / 2 seconds) ---
    bool diag_mode = (data->scan_count % 200) == 0;
    
    if (diag_mode) {
        printk("\n--- DIAGNOSTIC START ---\n");
        // Force Mux Enable Active
        // 0 = Low = Active (Assuming GPIO_ACTIVE_HIGH)
        gpio_pin_set_dt(&config->mux_enable, 0);
        printk("Step 1: Mux Enabled (Pin %d set to 0)\n", config->mux_enable.pin);
    }

    // --- NORMAL SCAN ---
    for (int col_idx = 0; col_idx < config->outputs.len; col_idx++) {
        const struct kscan_gpio *out_gpio = &config->outputs.gpios[col_idx];

        // DRIVE COLUMN LOW (Active)
        gpio_pin_set_dt(&out_gpio->spec, 0); 
        k_busy_wait(10); 

        // Diagnostic: Log Col 0 State
        if (diag_mode && col_idx == 0) {
            printk("Step 2: Col 0 Driven Low (Pin %d set to 0)\n", out_gpio->spec.pin);
        }

        for (int row_idx = 0; row_idx < config->rows; row_idx++) {
            mux_set_row(dev, row_idx);
            k_busy_wait(10); 

            int mv = read_adc_mv(dev);
            
            // Diagnostic: Log Row 0, Col 0 Voltage
            if (diag_mode && col_idx == 0 && row_idx == 0) {
                printk("Step 3: Read ADC (Row 0, Col 0) -> %d mV\n", mv);
                if (mv > 3000) printk("        -> RESULT: HIGH (No key press detected)\n");
                else           printk("        -> RESULT: LOW (Key Press Detected!)\n");
            }

            if (row_idx < 8 && col_idx < 16) {
                voltages[row_idx][col_idx] = (uint16_t)mv;
            }
        }
        
        // DRIVE COLUMN HIGH (Inactive)
        gpio_pin_set_dt(&out_gpio->spec, 1); 
        k_busy_wait(5);
    }
    
    if (diag_mode) {
        printk("--- DIAGNOSTIC END ---\n\n");
    }

    // --- PROCESSING ---
    for (int r = 0; r < config->rows; r++) {
        int pressed_in_row = 0;
        for (int c = 0; c < config->cols; c++) {
            if (voltages[r][c] < THRESHOLD_DEFAULT_MV) pressed_in_row++;
        }

        int threshold = THRESHOLD_DEFAULT_MV;
        if (pressed_in_row == 1) threshold = THRESHOLD_1_PRESS_MV;
        // Simplified threshold logic for debug

        for (int c = 0; c < config->cols; c++) {
            const int index = state_index_rc(config, r, c);
            bool pressed = (voltages[r][c] < threshold);
            struct zmk_debounce_state *state = &data->matrix_state[index];
            zmk_debounce_update(state, pressed, config->debounce_scan_period_ms, &config->debounce_config);

            if (zmk_debounce_get_changed(state)) {
                bool final_pressed = zmk_debounce_is_pressed(state);
                printk("EVENT: R:%d C:%d | V:%d | %s\n", r, c, voltages[r][c], final_pressed ? "ON" : "OFF");
                data->callback(dev, r, c, final_pressed);
            }
            continue_scan = continue_scan || zmk_debounce_is_active(state);
        }
    }

    if (continue_scan) kscan_matrix_read_continue(dev);
    else kscan_matrix_read_end(dev);

    return 0;
}

static void kscan_matrix_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_matrix_data *data = CONTAINER_OF(dwork, struct kscan_matrix_data, work);
    kscan_matrix_read(data->dev);
}

// --- INITIALIZATION ---
static int kscan_matrix_init(const struct device *dev) {
    printk("KSCAN: Starting Initialization...\n");
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    int err;

    data->dev = dev;
    k_work_init_delayable(&data->work, kscan_matrix_work_handler);

    // 1. MUX ENABLE
    if (!device_is_ready(config->mux_enable.port)) return -ENODEV;
    printk("KSCAN CFG: Mux Enable on Pin %d\n", config->mux_enable.pin);
    // Config as Output, Default High (Inactive)
    gpio_pin_configure_dt(&config->mux_enable, GPIO_OUTPUT_INACTIVE);

    // 2. MUX ADDRESS
    printk("KSCAN CFG: Mux Addrs: ");
    for (int i = 0; i < config->adc_gpios.len; i++) {
        printk("%d ", config->adc_gpios.gpios[i].spec.pin);
        if (!device_is_ready(config->adc_gpios.gpios[i].spec.port)) return -ENODEV;
        gpio_pin_configure_dt(&config->adc_gpios.gpios[i].spec, GPIO_OUTPUT_LOW);
    }
    printk("\n");

    // 3. COLUMNS
    printk("KSCAN CFG: Cols Start Pin %d, Total: %d\n", 
           config->outputs.gpios[0].spec.pin, config->outputs.len);
    for (int i = 0; i < config->outputs.len; i++) {
        if (!device_is_ready(config->outputs.gpios[i].spec.port)) return -ENODEV;
        // Config as Output, Default High (Inactive)
        gpio_pin_configure_dt(&config->outputs.gpios[i].spec, GPIO_OUTPUT_INACTIVE);
    }

    // 4. ADC
    if (!device_is_ready(config->adc_dev)) {
        printk("KSCAN ERROR: ADC Not Ready\n");
        return -ENODEV;
    }
    printk("KSCAN CFG: ADC OK\n");

    // ADC Setup
    data->adc_cfg.gain = ADC_GAIN_1;
    data->adc_cfg.reference = ADC_REF_INTERNAL;
    data->adc_cfg.acquisition_time = ADC_ACQ_TIME_DEFAULT;
    data->adc_cfg.channel_id = config->io_channel;
    data->adc_seq.channels = BIT(config->io_channel);
    data->adc_seq.buffer = &data->adc_buffer;
    data->adc_seq.buffer_size = sizeof(data->adc_buffer);
    data->adc_seq.resolution = 12;

    err = adc_channel_setup(config->adc_dev, &data->adc_cfg);
    if (err) return err;

    printk("KSCAN: Init Complete\n");
    return 0;
}

static int kscan_matrix_configure(const struct device *dev, const kscan_callback_t callback) {
    struct kscan_matrix_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int kscan_matrix_enable(const struct device *dev) {
    printk("KSCAN: Enable Called\n");
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    data->scan_time = k_uptime_get();
    
    // Enable Mux (Low = Active)
    gpio_pin_set_dt(&config->mux_enable, 0); 
    k_sleep(K_MSEC(100));
    return kscan_matrix_read(dev);
}

static int kscan_matrix_disable(const struct device *dev) {
    struct kscan_matrix_data *data = dev->data;
    const struct kscan_matrix_config *config = dev->config;
    k_work_cancel_delayable(&data->work);
    gpio_pin_set_dt(&config->mux_enable, 1);
    return 0;
}

static const struct kscan_driver_api kscan_matrix_api = {
    .config = kscan_matrix_configure,
    .enable_callback = kscan_matrix_enable,
    .disable_callback = kscan_matrix_disable,
};

// Boilerplate macros
#define KSCAN_MATRIX_INIT(n) \
    static struct kscan_gpio kscan_mux_gpios_##n[] = { LISTIFY(INST_ADCS_LEN(n), KSCAN_GPIO_ADC_CFG_INIT, (, ), n) }; \
    static struct kscan_gpio kscan_matrix_cols_##n[] = { LISTIFY(INST_COLS_LEN(n), KSCAN_GPIO_COL_CFG_INIT, (, ), n) }; \
    static struct zmk_debounce_state kscan_matrix_state_##n[INST_MATRIX_LEN(n)]; \
    static struct kscan_matrix_data kscan_matrix_data_##n = { .matrix_state = kscan_matrix_state_##n }; \
    static const struct kscan_matrix_config kscan_matrix_config_##n = { \
        .io_channel = DT_IO_CHANNELS_INPUT(DT_DRV_INST(n)), \
        .adc_dev = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(DT_DRV_INST(n))), \
        .mux_enable = GPIO_DT_SPEC_INST_GET(n, mux_enable_gpios), \
        .cols = ARRAY_SIZE(kscan_matrix_cols_##n), \
        .rows = INST_ROWS_LEN(n), \
        .adc_gpios = KSCAN_GPIO_LIST(kscan_mux_gpios_##n), \
        .outputs = KSCAN_GPIO_LIST(kscan_matrix_cols_##n), \
        .debounce_config = { .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n), .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n) }, \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms), \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms), \
    }; \
    DEVICE_DT_INST_DEFINE(n, &kscan_matrix_init, NULL, &kscan_matrix_data_##n, \
                          &kscan_matrix_config_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY, \
                          &kscan_matrix_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_MATRIX_INIT);
