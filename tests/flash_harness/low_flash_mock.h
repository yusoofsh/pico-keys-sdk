/*
 * This file is part of the Pico Keys SDK distribution (https://github.com/polhenarejos/pico-keys-sdk).
 * Copyright (c) 2022 Pol Henarejos.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Mock pico-sdk flash hardware for the low_flash boot-stage integration
 * harness (host, x86_64 Linux).
 *
 * The fake flash is a private anonymous mapping placed at the real RP2040 XIP
 * address (0x10000000) with MAP_FIXED_NOREPLACE, so the production code's
 * direct pointer reads work unmodified. The mock implements NOR semantics
 * (programming can only clear bits) and records every erase and program call
 * with its offset, length, alignment, interrupt state and multicore lockout
 * depth, so tests assert on what the boot stage actually did to the hardware.
 */

#ifndef _LOW_FLASH_MOCK_H_
#define _LOW_FLASH_MOCK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MOCK_XIP_BASE 0x10000000u
#define MOCK_MAP_SIZE (16u * 1024u * 1024u) // the whole RP2040 XIP window
#define MOCK_MARKER_OFFSET 0x100000u        // the fixed physical marker sector

/* One recorded hardware interaction, in call order.
 * op: 'C' = flash_do_cmd (offs = command byte), 'E' = erase, 'P' = program,
 * 'B' = flash_set_bounds (offs = start, count = end). */
typedef struct {
    char op;
    uint32_t offs;
    size_t count;
    bool irq_off;
    int lockout_depth;
} mock_call_t;

#define MOCK_MAX_CALLS 64

extern mock_call_t mock_calls[MOCK_MAX_CALLS];
extern int mock_ncalls;

/* Map the fake flash at the XIP address. Returns 0 on success. */
int mock_map_flash(void);

/* Blank the whole chip, set the JEDEC capacity byte the mock chip reports and
 * clear the call log, the violation counter and the lockout counters. */
void mock_reset(uint8_t jedec_capacity_e);

/* Clear the call log only (used between two boots on the same flash). */
void mock_log_reset(void);

/* When set, flash_range_program records the call but the bytes do not stick,
 * which models a failed page program for the readback scenario. */
void mock_set_program_drop(bool drop);

/* Direct access to the fake flash image (offsets from XIP_BASE). */
const uint8_t *mock_flash(void);
uint8_t *mock_flash_mutable(void);

/* Counters the scenarios assert on. */
int mock_api_violations(void); // alignment, out-of-chip or bad-length calls
int mock_lockout_calls(void);  // multicore_lockout_start/end calls
int mock_erase_calls(void);
int mock_program_calls(void);

#endif
