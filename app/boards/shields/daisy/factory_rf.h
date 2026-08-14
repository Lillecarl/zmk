/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Handlers for factory protocol group 0xB (RF test modes), implemented in
 * factory_rf.c and dispatched from factory_hid.c. Compiled only when
 * CONFIG_DAISY_FACTORY_RF=y; the dispatcher answers UNSUPPORTED otherwise.
 *
 * Each returns a daisy_factory_status code. See factory.h for the wire
 * contract, including the channel-numbering difference versus group 0xA.
 */

#ifndef DAISY_FACTORY_RF_H
#define DAISY_FACTORY_RF_H

#include <stdint.h>

/* RF_ENTER: quiesce and shut down BLE, start the HFXO, take the radio.
 * One-way -- recovery is DAISY_FACTORY_CMD_REBOOT. Idempotent.
 * LOCKED unless factory mode is active. */
uint8_t daisy_factory_rf_enter(void);

/* RF_START: configure and start a test mode; cancels any running test first.
 * `req`/`req_len` are the request payload (13 bytes, see factory.h). */
uint8_t daisy_factory_rf_start(const uint8_t *req, uint8_t req_len);

/* RF_STOP: stop the running test. Writes [rx_packets u32 LE]. */
uint8_t daisy_factory_rf_stop(uint8_t *payload, uint8_t *out_len);

/* RF_STATUS: write a struct daisy_factory_rf_status. Valid before RF_ENTER. */
uint8_t daisy_factory_rf_status(uint8_t *payload, uint8_t *out_len);

#endif /* DAISY_FACTORY_RF_H */
