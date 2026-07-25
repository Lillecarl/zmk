/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Crash breadcrumb: capture where a fatal error (hard fault, kernel oops,
 * failed __ASSERT) happened into __noinit RAM, which survives the warm
 * reboot CONFIG_RESET_ON_FATAL_ERROR performs, and expose it to the factory
 * tool (`aster v1 --crash-info`). This is the only post-mortem channel on a
 * sealed unit: the console/RTT need a debug probe, and the log buffer dies
 * with the reboot.
 *
 * The strong k_sys_fatal_error_handler here replaces the weak default, so
 * this file also owns the reboot-on-fatal behavior.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>

#include "crash_info.h"

LOG_MODULE_REGISTER(daisy_crash, CONFIG_ZMK_LOG_LEVEL);

/* Arbitrary tag telling a real record apart from power-on RAM garbage. */
#define CRASH_INFO_MAGIC 0xC7A54171u

static __noinit struct daisy_crash_info crash_info;
static __noinit uint32_t crash_info_magic;

/* A failed __ASSERT records its origin here, then falls through into
 * k_panic() -> k_sys_fatal_error_handler below, which fills in the rest.
 * Overrides the weak default (which just aborts the thread). */
void assert_post_action(const char *file, unsigned int line) {
    /* On back-to-back asserts keep the first record: it is the root cause. */
    if (crash_info_magic != CRASH_INFO_MAGIC) {
        /* Keep the basename only: __FILE__ is a long build-tree path and the
         * wire field is small. */
        const char *base = strrchr(file, '/');
        base = (base != NULL) ? base + 1 : file;
        memset(&crash_info, 0, sizeof(crash_info));
        strncpy(crash_info.file, base, sizeof(crash_info.file) - 1);
        crash_info.line = line;
    }
    k_panic();
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) {
    if (crash_info_magic != CRASH_INFO_MAGIC) {
        /* Not preceded by assert_post_action: plain fault, no file/line. */
        memset(&crash_info, 0, sizeof(crash_info));
    }
    crash_info.reason = reason;
    if (esf != NULL) {
        crash_info.pc = esf->basic.pc;
        crash_info.lr = esf->basic.lr;
    }
    const char *name = k_thread_name_get(k_current_get());
    strncpy(crash_info.thread, name != NULL ? name : "", sizeof(crash_info.thread) - 1);
    crash_info_magic = CRASH_INFO_MAGIC;

    /* Warm reset keeps __noinit RAM intact. */
    sys_reboot(SYS_REBOOT_WARM);
    CODE_UNREACHABLE;
}

bool daisy_crash_info_get(struct daisy_crash_info *out) {
    if (crash_info_magic != CRASH_INFO_MAGIC) {
        return false;
    }
    *out = crash_info;
    return true;
}

void daisy_crash_info_clear(void) { crash_info_magic = 0; }

/* Surface a stored crash once per boot in the log too (visible when a
 * console is attached; harmless otherwise). */
static int daisy_crash_info_report(void) {
    if (crash_info_magic == CRASH_INFO_MAGIC) {
        LOG_ERR("previous boot died: reason %u pc 0x%08x lr 0x%08x %s:%u (thread %s)",
                crash_info.reason, crash_info.pc, crash_info.lr,
                crash_info.file[0] ? crash_info.file : "?", crash_info.line, crash_info.thread);
    }
    return 0;
}
SYS_INIT(daisy_crash_info_report, APPLICATION, 99);
