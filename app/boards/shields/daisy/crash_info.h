/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef DAISY_CRASH_INFO_H
#define DAISY_CRASH_INFO_H

#include <stdbool.h>
#include <stdint.h>

/* One recorded fatal error; see crash_info.c. Field meanings match
 * struct daisy_factory_crash_info in factory.h (which adds the wire
 * framing); keep the two in sync. */
struct daisy_crash_info {
    uint32_t reason; /* k_fatal_error_reason */
    uint32_t pc;     /* faulting PC (0 if no exception frame) */
    uint32_t lr;     /* link register at the fault */
    uint32_t line;   /* assert line (0 unless the fault was an assert) */
    char file[12];   /* assert file, truncated; "" unless an assert */
    char thread[8];  /* running thread name, truncated */
};

/* Copy the stored crash record into *out. Returns false when none exists. */
bool daisy_crash_info_get(struct daisy_crash_info *out);

/* Drop the stored record (e.g. after the host has read it). */
void daisy_crash_info_clear(void);

#endif /* DAISY_CRASH_INFO_H */
