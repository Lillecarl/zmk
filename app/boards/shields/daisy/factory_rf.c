/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Daisy factory RF test modes (protocol group 0xB) -- CE/FCC certification.
 *
 * Wraps Nordic's radio_test driver (rf/radio_test.c, vendored from the NCS
 * sample) so the modes Lite-On's BT-test SOP asks for are reachable from the
 * product firmware over the factory HID interface, with no image swap. See
 * factory.h for the wire contract and CE-FCC-certification/README.md for the
 * lab procedure.
 *
 * The hard part is ownership of the radio. On nRF54L the BLE controller and
 * radio_test both want RADIO, TIMER10 and DPPI, so RF_ENTER shuts the whole
 * Bluetooth stack down (bt_disable -> hci_driver_close -> ll_deinit) before
 * taking them. That is why entering is one-way: nothing here gives the radio
 * back, and recovery is a reboot. USB is untouched, so the factory interface
 * carrying these commands stays up for the whole session.
 *
 * Compiled only when CONFIG_DAISY_FACTORY_RF=y.
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <hal/nrf_clock.h>
#include <hal/nrf_radio.h>

#include <zmk/ble.h>

#include "factory.h"
#include "factory_state.h"
#include "rf/radio_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(daisy_factory, CONFIG_ZMK_LOG_LEVEL);

/* The status payload has to fit one frame, and the host decodes it by fixed
 * offsets (aster's proto::RfStatus). Catch a field added on one side only. */
BUILD_ASSERT(sizeof(struct daisy_factory_rf_status) == 24,
             "RF_STATUS payload changed; update aster's RfStatus decoder too");
BUILD_ASSERT(offsetof(struct daisy_factory_rf_status, tx_packets) == 19,
             "tx_packets must stay after the prefix the dongle shares");
BUILD_ASSERT(offsetof(struct daisy_factory_rf_status, payload_len) == 23,
             "payload_len must stay after tx_packets, at the dongle's offset");
BUILD_ASSERT(sizeof(struct daisy_factory_rf_status) <= DAISY_FACTORY_PAYLOAD_SIZE,
             "RF_STATUS payload does not fit in a factory frame");

/* Radio-test configuration for the currently selected mode. Must have static
 * lifetime: radio_test_init() keeps a pointer into params.rx.cb, and the
 * radio ISR is handed this struct as its context. */
static struct radio_test_config rf_config;

/* Mirrors the last accepted RF_START so RF_STATUS can echo it back (the
 * radio_test config is a union, so it can't be read back field by field). */
static struct daisy_factory_rf_status rf_status;

/* Set once RF_ENTER has succeeded; RF_START/RF_STOP need it. */
static bool rf_entered;

/* The driver's TX counter as this run started, so RF_STATUS can report a
 * per-run count. Needed because the driver zeroes the counter when a
 * modulated-TX run starts but NOT when a duty-cycled one does (an upstream
 * inconsistency: the reset lives in radio_modulated_tx_carrier only). Without
 * this, a duty-cycle run reports the previous run's packets plus its own. */
static uint32_t rf_tx_baseline;

/* Held for the lifetime of RF mode so the HFXO stays up. */
static struct onoff_client rf_clk_cli;

/* How long to wait for the HF crystal in rf_clock_init(). Startup is well
 * under a millisecond; this only exists so a completion that never arrives
 * fails the command instead of hanging the thread. */
#define RF_HFXO_TIMEOUT_MS 500

/* Called by the radio-test driver when a counted MODULATED_TX or RX run
 * finishes on its own. Runs in the radio ISR (TX) or on the system workqueue
 * (RX); both only touch this one flag.
 *
 * NOTE the driver calls params.modulated_tx.cb unconditionally, without a
 * NULL check, so every modulated-TX start must install it. */
static void rf_test_finished(void) {
    rf_status.running = 0;
}

/* Dump what the HFXO hardware actually looks like when the onoff request has
 * not completed. Answers the only question that matters at that point: did the
 * crystal start (so the completion path is what broke), or is it not running at
 * all -- and if the started event is latched with the interrupt masked or the
 * NVIC line disabled, the tear-down of the link layer took the clock driver's
 * interrupt with it. */
static void rf_clock_diag(void) {
    nrf_clock_hfclk_t type = 0;
    bool running = nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &type);

    LOG_ERR("rf: HFXO diag: running=%d high_acc=%d started_evt=%d int_unmasked=%d nvic_en=%d",
            running, type == NRF_CLOCK_HFCLK_HIGH_ACCURACY,
            nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED),
            nrf_clock_int_enable_check(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK) != 0,
            irq_is_enabled(DT_IRQN(DT_NODELABEL(clock))));
}

/* Last resort: drive the crystal from the HAL and poll STATUS, bypassing the
 * clock driver's onoff bookkeeping and its interrupt entirely.
 *
 * Legitimate here in a way it would not be anywhere else in the firmware: RF
 * mode has already torn the Bluetooth stack down, entering is one-way, and
 * nothing else in the system contends for the HFXO afterwards. It is also what
 * Nordic's bare-metal radio_test does. */
static uint8_t rf_clock_start_direct(void) {
    nrf_clock_hfclk_t type = 0;

    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);

    const k_timepoint_t deadline = sys_timepoint_calc(K_MSEC(RF_HFXO_TIMEOUT_MS));
    while (!(nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &type) &&
             type == NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
        if (sys_timepoint_expired(deadline)) {
            LOG_ERR("rf: HFXO did not start when driven directly either");
            return DAISY_FACTORY_ERR_HW;
        }
        k_msleep(1);
    }

    LOG_WRN("rf: HFXO started directly (clock driver's request never completed)");
    return DAISY_FACTORY_OK;
}

/* Bring the HF crystal up and hold it for the whole RF session.
 *
 * Deliberately not factory_hid.c's pert_hfxo_prewarm(): that one exists to
 * hand the running BLE link layer a clock it can already see, and by the time
 * we get here there is no link layer left. This is the radio_test sample's own
 * clock_init(), which is what the lab's measurements were taken against. */
static uint8_t rf_clock_init(void) {
    nrf_clock_hfclk_t type = 0;

    /* Fast path, and on this board the normal one: the crystal is already
     * running (USB holds it for as long as the factory interface is live).
     *
     * It has to be checked first because the request below can never complete
     * in that state -- the hardware produces no fresh XOSTARTED event for an
     * XO that is already running, and the driver's onoff completion hangs off
     * exactly that event. Measured right after bt_disable():
     * running=1 high_acc=1 started_evt=0 int_unmasked=0. */
    if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &type) &&
        type == NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
        LOG_INF("rf: HFXO already running");
        goto pll;
    }

    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (mgr == NULL) {
        LOG_ERR("rf: no HF clock manager");
        return DAISY_FACTORY_ERR_HW;
    }

    sys_notify_init_spinwait(&rf_clk_cli.notify);
    int err = onoff_request(mgr, &rf_clk_cli);
    if (err < 0) {
        LOG_ERR("rf: HF clock request failed: %d", err);
        return DAISY_FACTORY_ERR_HW;
    }

    /* The request is completed from the clock ISR, so this poll normally ends
     * within the crystal's startup time. Bound it anyway: an unbounded poll is
     * an infinite spin if that completion never arrives -- which is exactly
     * what happens if something upstream (ll_deinit releasing the radio) left
     * the clock driver unable to finish, and a spin here takes the whole
     * device down with no log output to say why. */
    const k_timepoint_t deadline = sys_timepoint_calc(K_MSEC(RF_HFXO_TIMEOUT_MS));
    int res = 0;

    while ((err = sys_notify_fetch_result(&rf_clk_cli.notify, &res)) != 0) {
        if (sys_timepoint_expired(deadline)) {
            LOG_ERR("rf: HF clock did not start within %d ms", RF_HFXO_TIMEOUT_MS);
            rf_clock_diag();
            (void)onoff_cancel_or_release(mgr, &rf_clk_cli);

            uint8_t status = rf_clock_start_direct();
            if (status != DAISY_FACTORY_OK) {
                return status;
            }
            /* Crystal is up; carry on to the PLL workaround below. */
            res = 0;
            break;
        }
        /* Sleep rather than spin: the caller is a preemptible thread, so this
         * keeps the log thread (and everything else) running. */
        k_msleep(1);
    }
    if (res) {
        LOG_ERR("rf: HF clock could not be started: %d", res);
        return DAISY_FACTORY_ERR_HW;
    }

pll:
#if defined(NRF54LM20A_XXAA)
    /* MLTPAN-39: the radio's PLL must be started explicitly on this part or
     * TX produces nothing decodable on air. Same workaround the radio_test
     * sample applies right after its clock init. */
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

    LOG_INF("rf: HF clock running");
    return DAISY_FACTORY_OK;
}

/* Take the radio: quiesce BLE, tear the stack down, start the clock, init the
 * radio-test driver. One-way; recovery is DAISY_FACTORY_CMD_REBOOT. */
uint8_t daisy_factory_rf_enter(void) {
    if (rf_entered) {
        return DAISY_FACTORY_OK;
    }

    /* Gate: this ships in production firmware and kills BLE until the next
     * reboot, so it must not be one stray HID report away on a user's
     * keyboard. The operator opts in with FACTORY_MODE_SET first. */
    if (!daisy_factory_mode_active()) {
        LOG_WRN("rf: RF_ENTER refused, factory mode is not active");
        return DAISY_FACTORY_ERR_LOCKED;
    }

    int err;

#if IS_ENABLED(CONFIG_ZMK_BLE)
    /* Stop advertising and drop every link before pulling the stack out from
     * under them, so hosts see a clean disconnect rather than a timeout. */
    zmk_ble_adv_enabled_set(false);
    for (uint8_t i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (zmk_ble_profile_is_connected(i)) {
            (void)zmk_ble_prof_disconnect(i);
        }
    }
    /* The disconnects complete asynchronously; give the controller a moment
     * to put them on air. bt_disable() resets it regardless, so this is about
     * politeness to the peer, not correctness here -- kept short because the
     * whole handler runs on the system workqueue before the response report
     * goes out, and the host is waiting on it. */
    k_msleep(50);

    LOG_INF("rf: disabling Bluetooth");
    err = bt_disable();
    if (err) {
        LOG_ERR("rf: bt_disable failed: %d", err);
        return DAISY_FACTORY_ERR_HW;
    }
    LOG_INF("rf: Bluetooth stack down, radio released");
#endif

    uint8_t status = rf_clock_init();
    if (status != DAISY_FACTORY_OK) {
        return status;
    }

    memset(&rf_config, 0, sizeof(rf_config));
    err = radio_test_init(&rf_config);
    if (err) {
        LOG_ERR("rf: radio_test_init failed: %d", err);
        return DAISY_FACTORY_ERR_HW;
    }

    rf_entered = true;
    rf_status.entered = 1;
    LOG_INF("rf: RF test mode entered (reboot to leave)");
    return DAISY_FACTORY_OK;
}

/* Map the wire enums onto the radio-test driver's. Kept explicit rather than
 * cast: factory.h pins the wire values, radio_test.h is upstream's, and the
 * two must be free to drift. */
static bool rf_mode_to_driver(uint8_t wire, enum radio_test_mode *out) {
    switch (wire) {
    case DAISY_FACTORY_RF_MODE_UNMODULATED_TX:
        *out = UNMODULATED_TX;
        return true;
    case DAISY_FACTORY_RF_MODE_MODULATED_TX:
        *out = MODULATED_TX;
        return true;
    case DAISY_FACTORY_RF_MODE_RX:
        *out = RX;
        return true;
    case DAISY_FACTORY_RF_MODE_TX_SWEEP:
        *out = TX_SWEEP;
        return true;
    case DAISY_FACTORY_RF_MODE_RX_SWEEP:
        *out = RX_SWEEP;
        return true;
    case DAISY_FACTORY_RF_MODE_MODULATED_TX_DUTY_CYCLE:
        *out = MODULATED_TX_DUTY_CYCLE;
        return true;
    default:
        return false;
    }
}

static bool rf_rate_to_driver(uint8_t wire, nrf_radio_mode_t *out) {
    switch (wire) {
    case DAISY_FACTORY_RF_RATE_BLE_1M:
        *out = NRF_RADIO_MODE_BLE_1MBIT;
        return true;
    case DAISY_FACTORY_RF_RATE_BLE_2M:
        *out = NRF_RADIO_MODE_BLE_2MBIT;
        return true;
    default:
        return false;
    }
}

static bool rf_pattern_to_driver(uint8_t wire, enum transmit_pattern *out) {
    switch (wire) {
    case DAISY_FACTORY_RF_PAT_RANDOM:
        *out = TRANSMIT_PATTERN_RANDOM;
        return true;
    case DAISY_FACTORY_RF_PAT_11110000:
        *out = TRANSMIT_PATTERN_11110000;
        return true;
    case DAISY_FACTORY_RF_PAT_11001100:
        *out = TRANSMIT_PATTERN_11001100;
        return true;
    default:
        return false;
    }
}

/* RF_START payload layout, see factory.h. The 14th byte (payload_len) is
 * optional; without it the transmitted length is the one every firmware before
 * that field used. */
#define RF_START_REQ_LEN 13
#define RF_START_REQ_LEN_WITH_PAYLOAD_LEN 14
/* Transmitted test-data bytes per packet when payload_len is absent or 0.
 * What every firmware before that field sent, unconditionally. */
#define RF_PAYLOAD_LEN_DEFAULT 255

/* Configure and start a test mode. Any running test is cancelled first, so
 * the host can move between modes without an explicit RF_STOP. */
uint8_t daisy_factory_rf_start(const uint8_t *req, uint8_t req_len) {
    if (!rf_entered) {
        return DAISY_FACTORY_ERR_HW;
    }
    if (req_len < RF_START_REQ_LEN) {
        return DAISY_FACTORY_ERR_BAD_LENGTH;
    }

    enum radio_test_mode mode;
    nrf_radio_mode_t rate;
    enum transmit_pattern pattern;
    if (!rf_mode_to_driver(req[0], &mode) || !rf_rate_to_driver(req[1], &rate) ||
        !rf_pattern_to_driver(req[2], &pattern)) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }

    const int8_t tx_power = (int8_t)req[3];
    const uint8_t chan_start = req[4];
    const uint8_t chan_end = req[5];
    const uint16_t dwell_ms = sys_get_le16(&req[6]);
    const uint8_t duty = req[8];
    const uint32_t packets = sys_get_le32(&req[9]);
    /* Absent and 0 both mean the default. 0 is the unset value on the
     * dongle, whose config report has no length to make a field optional; the
     * two agree so one host encoder serves both. */
    uint8_t payload_len = (req_len >= RF_START_REQ_LEN_WITH_PAYLOAD_LEN) ? req[13] : 0;
    if (payload_len == 0) {
        payload_len = RF_PAYLOAD_LEN_DEFAULT;
    }

    if (chan_start > DAISY_FACTORY_RF_CHANNEL_MAX || chan_end > DAISY_FACTORY_RF_CHANNEL_MAX) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }
    const bool is_sweep = (mode == TX_SWEEP || mode == RX_SWEEP);
    if (is_sweep) {
        if (chan_end < chan_start) {
            return DAISY_FACTORY_ERR_BAD_ARG;
        }
        if (dwell_ms > DAISY_FACTORY_RF_DWELL_MS_MAX) {
            return DAISY_FACTORY_ERR_BAD_ARG;
        }
    }
    if (mode == MODULATED_TX_DUTY_CYCLE &&
        (duty < DAISY_FACTORY_RF_DUTY_MIN || duty > DAISY_FACTORY_RF_DUTY_MAX)) {
        return DAISY_FACTORY_ERR_BAD_ARG;
    }

    /* Stop whatever is on air before rewriting the config the running test's
     * ISR is reading. */
    radio_test_cancel(rf_config.type);

    memset(&rf_config, 0, sizeof(rf_config));
    rf_config.type = mode;
    rf_config.mode = rate;

    switch (mode) {
    case UNMODULATED_TX:
        rf_config.params.unmodulated_tx.txpower = tx_power;
        rf_config.params.unmodulated_tx.channel = chan_start;
        break;
    case MODULATED_TX:
        rf_config.params.modulated_tx.txpower = tx_power;
        rf_config.params.modulated_tx.pattern = pattern;
        rf_config.params.modulated_tx.channel = chan_start;
        rf_config.params.modulated_tx.packets_num = packets;
        rf_config.params.modulated_tx.payload_len = payload_len;
        rf_config.params.modulated_tx.cb = rf_test_finished;
        break;
    case RX:
        rf_config.params.rx.pattern = pattern;
        rf_config.params.rx.channel = chan_start;
        rf_config.params.rx.packets_num = packets;
        rf_config.params.rx.payload_len = payload_len;
        rf_config.params.rx.cb = rf_test_finished;
        break;
    case TX_SWEEP:
        rf_config.params.tx_sweep.txpower = tx_power;
        rf_config.params.tx_sweep.channel_start = chan_start;
        rf_config.params.tx_sweep.channel_end = chan_end;
        rf_config.params.tx_sweep.delay_ms = dwell_ms;
        break;
    case RX_SWEEP:
        rf_config.params.rx_sweep.channel_start = chan_start;
        rf_config.params.rx_sweep.channel_end = chan_end;
        rf_config.params.rx_sweep.delay_ms = dwell_ms;
        /* Sets the access address, so it has to match the transmitter's, as for
         * plain RX. Upstream had no field for it here; see the comment on
         * rx_sweep.pattern in rf/radio_test.h. */
        rf_config.params.rx_sweep.pattern = pattern;
        break;
    case MODULATED_TX_DUTY_CYCLE:
        rf_config.params.modulated_tx_duty_cycle.txpower = tx_power;
        rf_config.params.modulated_tx_duty_cycle.pattern = pattern;
        rf_config.params.modulated_tx_duty_cycle.channel = chan_start;
        rf_config.params.modulated_tx_duty_cycle.duty_cycle = duty;
        rf_config.params.modulated_tx_duty_cycle.payload_len = payload_len;
        break;
    default:
        return DAISY_FACTORY_ERR_BAD_ARG;
    }

    /* Publish the echo before starting: a counted run can finish (and clear
     * `running`) inside radio_test_start on the ISR, so this must not
     * overwrite it afterwards. */
    rf_status.mode = req[0];
    rf_status.rate = req[1];
    rf_status.pattern = req[2];
    rf_status.tx_power = tx_power;
    rf_status.chan_start = chan_start;
    rf_status.chan_end = chan_end;
    rf_status.dwell_ms = dwell_ms;
    rf_status.duty = duty;
    rf_status.packets = packets;
    rf_status.payload_len = payload_len;
    rf_status.running = 1;

    /* Where this run's TX count starts from. MODULATED_TX is zeroed by the
     * driver inside radio_test_start, so 0 is exact; for the duty-cycled mode
     * the counter carries over, and reading it here is safe because the
     * previous test was already cancelled above, so nothing is incrementing
     * it. */
    rf_tx_baseline = (mode == MODULATED_TX) ? 0 : radio_tx_packets_get();

    radio_test_start(&rf_config);

    LOG_INF("rf: started mode %u on ch %u (%u MHz), %d dBm, %u packets of %u bytes", req[0],
            chan_start, 2400U + chan_start, tx_power, packets, payload_len);
    return DAISY_FACTORY_OK;
}

/* Stop the running test. -> [rx_packets u32 LE]. */
uint8_t daisy_factory_rf_stop(uint8_t *payload, uint8_t *out_len) {
    if (!rf_entered) {
        return DAISY_FACTORY_ERR_HW;
    }

    radio_test_cancel(rf_config.type);
    rf_status.running = 0;

    struct radio_rx_stats stats = {0};
    radio_rx_stats_get(&stats);

    LOG_INF("rf: stopped, %u packets received", stats.packet_cnt);
    sys_put_le32(stats.packet_cnt, payload);
    *out_len = sizeof(uint32_t);
    return DAISY_FACTORY_OK;
}

/* Read the RF test state; valid before RF_ENTER (reports entered = 0). */
uint8_t daisy_factory_rf_status(uint8_t *payload, uint8_t *out_len) {
    if (rf_entered) {
        struct radio_rx_stats stats = {0};
        radio_rx_stats_get(&stats);
        rf_status.rx_packets = stats.packet_cnt;

        /* Only the modulated modes drive the driver's TX counter (they are the
         * ones that enable the END interrupt). For anything else -- an
         * unmodulated carrier, RX, a sweep -- the counter still holds whatever
         * the last modulated run left in it, so report 0 rather than a stale
         * number that looks like a carrier is sending packets it cannot
         * send. */
        const bool counts_tx = rf_status.mode == DAISY_FACTORY_RF_MODE_MODULATED_TX ||
                               rf_status.mode == DAISY_FACTORY_RF_MODE_MODULATED_TX_DUTY_CYCLE;
        rf_status.tx_packets = counts_tx ? radio_tx_packets_get() - rf_tx_baseline : 0;
    }

    memcpy(payload, &rf_status, sizeof(rf_status));
    *out_len = sizeof(rf_status);
    return DAISY_FACTORY_OK;
}
