/*
 * DEBUG (sleep/wake investigation) — TEMPORARY.
 *
 * Logs the SoC reset reason at boot so the *next* boot tells us how the
 * previous session ended:
 *   - OFF (GPIO)  -> woke cleanly from System OFF via a wakeup GPIO (matrix).
 *                    Arming works; chase the problem elsewhere.
 *   - RESETPIN    -> you power-cycled / J-Link reset it; not a real wake.
 *   - LOCKUP/DOG  -> it crashed rather than slept.
 *   - (empty)     -> power-on / brown-out reset.
 *
 * z_sys_poweroff() clears RESETREAS right before entering OFF, so after a
 * genuine deep-sleep + keypress wake only the OFF/GPIO bit should be set.
 *
 * Remove this file (and its CMakeLists entry) when the investigation is done.
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <helpers/nrfx_reset_reason.h>

LOG_MODULE_REGISTER(sleep_debug, CONFIG_ZMK_LOG_LEVEL);

static int sleep_debug_report_reset_reason(void) {
    uint32_t reason = nrfx_reset_reason_get();

    LOG_WRN("sleep_debug: RESETREAS = 0x%08x%s%s%s%s%s%s", reason,
            (reason & NRFX_RESET_REASON_OFF_MASK) ? " OFF/GPIO-wake" : "",
#if NRFX_RESET_REASON_HAS_VBUS
            (reason & NRFX_RESET_REASON_VBUS_MASK) ? " OFF/VBUS-wake" : "",
#else
            "",
#endif
            (reason & NRFX_RESET_REASON_RESETPIN_MASK) ? " RESETPIN" : "",
            (reason & NRFX_RESET_REASON_SREQ_MASK) ? " SREQ(soft)" : "",
            (reason & NRFX_RESET_REASON_DOG_MASK) ? " WATCHDOG" : "",
            (reason & NRFX_RESET_REASON_LOCKUP_MASK) ? " LOCKUP" : "");

    if (reason == 0) {
        LOG_WRN("sleep_debug: RESETREAS empty -> power-on / brown-out reset");
    }

    return 0;
}

SYS_INIT(sleep_debug_report_reset_reason, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
