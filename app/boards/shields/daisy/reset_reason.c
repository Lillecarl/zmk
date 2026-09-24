/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Clear the SoC's reset reason register once the application is up.
 *
 * RESET.RESETREAS is cumulative: bits stay set across resets until software
 * clears them (nRF54LM20A datasheet 5.8.11.1). Zephyr only clears it right
 * before System OFF, so a wake-from-OFF bit (GPIO DETECT, VBUS) or the DIF bit
 * left by a debug probe otherwise survives every later warm reboot.
 *
 * MCUboot's IO-based recovery entry (hold the pairing button through a reset)
 * refuses to enter recovery while any reset reason it does not allow is set,
 * and wake-from-OFF is deliberately not allowed. With stale bits in place the
 * button is dead after every soft reset; clearing here leaves the next reset
 * with only its own reason (SREQ, watchdog, ...), which the bootloader honors.
 *
 * Runs last in the APPLICATION init level so every reader of the register
 * (sleep_debug.c logs it at CONFIG_APPLICATION_INIT_PRIORITY) has seen it.
 */

#include <zephyr/init.h>

#include <helpers/nrfx_reset_reason.h>

static int daisy_reset_reason_clear(void) {
    nrfx_reset_reason_clear(UINT32_MAX);
    return 0;
}

SYS_INIT(daisy_reset_reason_clear, APPLICATION, 99);
