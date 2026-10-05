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
 * Boot-stage scenarios driving the UNMODIFIED production low_flash.c against
 * the mock NOR flash. Each variant executable (see tests/CMakeLists.txt) runs
 * the scenarios matching its compile-time PICO_FLASH_SIZE_BYTES and
 * PICO_FLASH_SIZE_LIMIT_BYTES and asserts on the recorded flash operations.
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "byte_array.h"
#include "flash_layout.h"
#include "low_flash_mock.h"
#include "picokeys.h"
#include "serial.h"

extern void low_flash_task(void);
extern void low_flash_init_core1(void);
extern bool low_flash_storage_locked(void);

#define MARKER_OFFSET MOCK_MARKER_OFFSET
#define MARKER_SECTOR (mock_flash() + MARKER_OFFSET)

static const uint8_t OTHER_UID[FLASH_MARKER_UID_SIZE] = { 9, 9, 9, 9, 9, 9, 9, 9 };

/* Same wrapper low_flash.c uses so both sides produce the same on-flash CRC.
 * crc32c is provided by the mock with the SDK signature. */
extern uint32_t crc32c(const_byte_array_t data);
static uint32_t harness_crc32(const uint8_t *data, size_t len) {
    return crc32c(CONST_BYTE_ARRAY(data, len));
}

static uint32_t fnv1a(const uint8_t *p, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void program_valid_marker(void) {
    build_marker_page(pico_serial.id, mock_flash_mutable() + MARKER_OFFSET, harness_crc32);
}

static const mock_call_t *single_op(char op) {
    int found = 0;
    const mock_call_t *call = NULL;
    for (int i = 0; i < mock_ncalls; i++) {
        if (mock_calls[i].op == op) {
            found++;
            call = &mock_calls[i];
        }
    }
    assert(found == 1);
    return call;
}

/* Name the scenario before it runs so a failing assert identifies itself. */
#define RUN_SCENARIO(scenario, ...) \
    do { \
        printf("RUN %s\n", #scenario); \
        scenario(__VA_ARGS__); \
    } while (0)

/* After any low_flash_init(): the marker stage runs before core1 exists, so no
 * multicore lockout may be taken and every erase/program must have run with
 * interrupts disabled and at lockout depth 0. */
static void assert_boot_safety(void) {
    assert(mock_lockout_calls() == 0);
    for (int i = 0; i < mock_ncalls; i++) {
        if (mock_calls[i].op == 'E' || mock_calls[i].op == 'P') {
            assert(mock_calls[i].irq_off);
            assert(mock_calls[i].lockout_depth == 0);
        }
    }
}

/* Boot-stage order: the JEDEC read comes before every erase/program, and
 * flash_set_bounds comes after all marker handling. */
static void assert_boot_order(void) {
    assert(mock_ncalls >= 2);
    assert(mock_calls[0].op == 'C'); // JEDEC read before any flash mutation
    assert(mock_calls[0].offs == 0x9F);
    for (int i = 0; i < mock_ncalls; i++) {
        if (mock_calls[i].op == 'B') {
            assert(i == mock_ncalls - 1); // bounds last, after all marker handling
        }
    }
}

static const mock_call_t *assert_single_bounds(uint32_t exp_start, uint32_t exp_end) {
    const mock_call_t *b = single_op('B');
    assert(b->offs == exp_start);
    assert(b->count == exp_end);
    return b;
}

static void assert_zero_writes(void) {
    assert(mock_erase_calls() == 0);
    assert(mock_program_calls() == 0);
}

static void assert_page_is_valid_marker(void) {
    assert(classify_marker_sector(MARKER_SECTOR, pico_serial.id, harness_crc32) == FLASH_MARKER_VALID);
    for (int i = 24; i < FLASH_MARKER_PAGE_SIZE; i++) {
        assert(MARKER_SECTOR[i] == 0xFF);
    }
    for (int i = FLASH_MARKER_PAGE_SIZE; i < FLASH_LAYOUT_SECTOR_SIZE; i++) {
        assert(MARKER_SECTOR[i] == 0xFF);
    }
}

/* Valid marker: zero writes at any offset, bounds handed out once. */
static __attribute__((unused)) void scenario_valid_marker(uint8_t jedec_e, uint32_t exp_start, uint32_t exp_end) {
    mock_reset(jedec_e);
    program_valid_marker();
    low_flash_init();
    assert_boot_safety();
    assert_boot_order();
    assert_zero_writes();
    assert_single_bounds(exp_start, exp_end);
    assert(!low_flash_storage_locked());
}

/* Blank marker sector: exactly one full-page program, no erase, correct
 * marker afterwards, and a second boot performs zero writes. */
static __attribute__((unused)) void scenario_blank_marker(uint8_t jedec_e, uint32_t exp_start, uint32_t exp_end) {
    mock_reset(jedec_e); // whole sector is 0xFF
    low_flash_init();
    assert_boot_safety();
    assert_boot_order();
    assert(mock_erase_calls() == 0);
    const mock_call_t *p = single_op('P');
    assert(p->offs == MARKER_OFFSET);
    assert(p->count == FLASH_MARKER_PAGE_SIZE); // full page, never sizeof(pointer)
    assert_single_bounds(exp_start, exp_end);
    assert(!low_flash_storage_locked());
    assert_page_is_valid_marker();

    // Second boot on the resulting flash: the marker is valid, zero writes.
    mock_log_reset();
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert_single_bounds(exp_start, exp_end);
    assert(!low_flash_storage_locked());
}

/* Legacy-truncated marker (the exact 4-byte artifact of the old
 * sizeof(pointer) program): one erase, then one full-page program. */
static __attribute__((unused)) void scenario_legacy_truncated(uint8_t jedec_e, uint32_t exp_start, uint32_t exp_end) {
    mock_reset(jedec_e);
    mock_flash_mutable()[MARKER_OFFSET + 0] = 0x53; // "SYEK":
    mock_flash_mutable()[MARKER_OFFSET + 1] = 0x59; // little-endian low half
    mock_flash_mutable()[MARKER_OFFSET + 2] = 0x45; // of the "PICOKEYS" magic,
    mock_flash_mutable()[MARKER_OFFSET + 3] = 0x4B; // what the old bug programmed
    low_flash_init();
    assert_boot_safety();
    assert_boot_order();
    assert(mock_erase_calls() == 1 && mock_program_calls() == 1);
    assert(mock_ncalls == 4);
    assert(mock_calls[1].op == 'E' && mock_calls[1].offs == MARKER_OFFSET && mock_calls[1].count == FLASH_LAYOUT_SECTOR_SIZE);
    assert(mock_calls[2].op == 'P' && mock_calls[2].offs == MARKER_OFFSET && mock_calls[2].count == FLASH_MARKER_PAGE_SIZE);
    assert_single_bounds(exp_start, exp_end);
    assert(!low_flash_storage_locked());
    assert_page_is_valid_marker();

    // Second boot: repaired, zero writes.
    mock_log_reset();
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert_single_bounds(exp_start, exp_end);
}

typedef enum {
    FOREIGN_RANDOM,    // random non-0xFF bytes
    FOREIGN_BAD_CRC,   // correct page, one flipped CRC bit
    FOREIGN_OTHER_UID, // valid marker for a different board
    FOREIGN_DIRTY_PAGE // valid marker, non-0xFF byte later in the page
} foreign_kind_t;

/* Foreign content: no erase, no program, storage locked, nothing changed. */
static __attribute__((unused)) void scenario_foreign_marker(uint8_t jedec_e, foreign_kind_t kind) {
    mock_reset(jedec_e);
    uint8_t *sector = mock_flash_mutable() + MARKER_OFFSET;
    switch (kind) {
    case FOREIGN_RANDOM:
        memset(sector, 0x5A, FLASH_LAYOUT_SECTOR_SIZE);
        break;
    case FOREIGN_BAD_CRC:
        program_valid_marker();
        sector[23] ^= 0x01; // one bit of the stored CRC
        break;
    case FOREIGN_OTHER_UID:
        build_marker_page(OTHER_UID, sector, harness_crc32);
        break;
    case FOREIGN_DIRTY_PAGE:
        program_valid_marker();
        sector[100] = 0x00;
        break;
    }
    uint32_t hash_before = fnv1a(mock_flash(), MOCK_MAP_SIZE);
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert(low_flash_storage_locked());
    for (int i = 0; i < mock_ncalls; i++) {
        assert(mock_calls[i].op != 'B'); // no usable bounds while locked
    }
    // Marker sector plus data region byte-identical to the pre-boot contents.
    assert(fnv1a(mock_flash(), MOCK_MAP_SIZE) == hash_before);
}

/* The page program did not stick: storage locked, no bounds, no further ops. */
static __attribute__((unused)) void scenario_readback_failure(uint8_t jedec_e) {
    mock_reset(jedec_e);
    mock_set_program_drop(true);
    low_flash_init();
    mock_set_program_drop(false);
    assert_boot_safety();
    assert_boot_order();
    assert(mock_erase_calls() == 0);
    const mock_call_t *p = single_op('P');
    assert(p->offs == MARKER_OFFSET && p->count == FLASH_MARKER_PAGE_SIZE);
    assert(low_flash_storage_locked());
    for (int i = 0; i < mock_ncalls; i++) {
        assert(mock_calls[i].op != 'B');
        if (i > (int)(p - mock_calls)) {
            assert(mock_calls[i].op != 'E' && mock_calls[i].op != 'P');
        }
    }
    for (int i = 0; i < FLASH_LAYOUT_SECTOR_SIZE; i++) {
        assert(MARKER_SECTOR[i] == 0xFF); // the failed program never landed
    }
}

/* A boot that ends storage-locked: writes refused, flash untouched. */
static __attribute__((unused)) void assert_locked_boot(uint8_t jedec_e) {
    mock_reset(jedec_e);
    uint32_t hash_before = fnv1a(mock_flash(), MOCK_MAP_SIZE);
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert(low_flash_storage_locked());
    for (int i = 0; i < mock_ncalls; i++) {
        assert(mock_calls[i].op != 'B');
    }
    assert(fnv1a(mock_flash(), MOCK_MAP_SIZE) == hash_before);
}

/* Storage-locked state keeps booting but refuses every later write. */
static __attribute__((unused)) void scenario_locked_writes(void) {
    assert(low_flash_storage_locked()); // reached after a foreign-marker boot
    const uintptr_t addr = MOCK_XIP_BASE + 0x101000u + 0x4000u; // inside the data region
    uint8_t buf[16];
    memset(buf, 0xA5, sizeof(buf));
    uint32_t hash_before = fnv1a(mock_flash(), MOCK_MAP_SIZE);

    assert(flash_program_block(addr, CONST_BYTE_ARRAY(buf, sizeof(buf))) == PICOKEYS_ERR_BLOCKED);
    assert(flash_program_halfword(addr, 0x1234) == PICOKEYS_ERR_BLOCKED);
    assert(flash_program_word(addr, 0xDEADBEEFu) == PICOKEYS_ERR_BLOCKED);
    assert(flash_program_uintptr(addr, (uintptr_t)0x11223344u) == PICOKEYS_ERR_BLOCKED);

    low_flash_init_core1();
    for (int i = 0; i < 8; i++) {
        low_flash_task();
    }
    assert_zero_writes();
    assert(mock_lockout_calls() == 0);
    assert(fnv1a(mock_flash(), MOCK_MAP_SIZE) == hash_before);
    assert(low_flash_storage_locked());
}

/* Capped RP2040 build: the data region is [0x10101000, 0x10200000) whatever the
 * detected capacity is, the marker sector stays outside it and nothing is ever
 * written into the unused upper half. */
static __attribute__((unused)) void scenario_capped_geometry(uint8_t jedec_e) {
    mock_reset(jedec_e);
    program_valid_marker(); // keep the ops log clean of marker writes
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert_single_bounds(0x10101000u, 0x10200000u);
    assert(!low_flash_storage_locked());
}

static int run_scenarios(void) {
    int tests = 0;
#if defined(PICO_FLASH_SIZE_LIMIT_BYTES) && PICO_FLASH_SIZE_LIMIT_BYTES == 0x200000 && PICO_FLASH_SIZE_BYTES == 0x400000
    // Full suite for the Pipico preset (4 MiB board, 2 MiB cap).
    RUN_SCENARIO(scenario_valid_marker, 0x16, 0x10101000u, 0x10200000u); tests++;
    RUN_SCENARIO(scenario_blank_marker, 0x16, 0x10101000u, 0x10200000u); tests++;
    RUN_SCENARIO(scenario_legacy_truncated, 0x16, 0x10101000u, 0x10200000u); tests++;
    RUN_SCENARIO(scenario_capped_geometry, 0x16); tests++;
    RUN_SCENARIO(scenario_capped_geometry, 0x17); tests++;
    RUN_SCENARIO(scenario_capped_geometry, 0x18); tests++;
    // Locking scenarios last: storage_locked never resets within a process.
    RUN_SCENARIO(scenario_readback_failure, 0x16); tests++;
    RUN_SCENARIO(scenario_foreign_marker, 0x16, FOREIGN_RANDOM); tests++;
    RUN_SCENARIO(scenario_foreign_marker, 0x16, FOREIGN_BAD_CRC); tests++;
    RUN_SCENARIO(scenario_foreign_marker, 0x16, FOREIGN_OTHER_UID); tests++;
    RUN_SCENARIO(scenario_foreign_marker, 0x16, FOREIGN_DIRTY_PAGE); tests++;
    RUN_SCENARIO(scenario_locked_writes); tests++;
    RUN_SCENARIO(assert_locked_boot, 0x00); tests++; // invalid JEDEC bytes
    RUN_SCENARIO(assert_locked_boot, 0xFF); tests++;
    RUN_SCENARIO(assert_locked_boot, 0x14); tests++; // 1 MiB chip: cap 2 MiB exceeds the chip
    RUN_SCENARIO(assert_locked_boot, 0x13); tests++; // 512 KiB chip: ditto
#elif defined(PICO_FLASH_SIZE_LIMIT_BYTES) && PICO_FLASH_SIZE_LIMIT_BYTES == 0x200000 && PICO_FLASH_SIZE_BYTES == 0x200000
    // 2 MiB board with a 2 MiB cap: 16 MiB chip clamps to the same layout.
    RUN_SCENARIO(scenario_blank_marker, 0x18, 0x10101000u, 0x10200000u); tests++;
    RUN_SCENARIO(scenario_capped_geometry, 0x15); tests++;
    RUN_SCENARIO(scenario_capped_geometry, 0x18); tests++;
#elif defined(PICO_FLASH_SIZE_LIMIT_BYTES) && PICO_FLASH_SIZE_LIMIT_BYTES == 0x200800
    RUN_SCENARIO(assert_locked_boot, 0x16); tests++; // misaligned cap: explicit error, no writes
#elif defined(PICO_FLASH_SIZE_LIMIT_BYTES) && PICO_FLASH_SIZE_LIMIT_BYTES == 0x100000
    RUN_SCENARIO(assert_locked_boot, 0x16); tests++; // too-small cap: no data region above the marker
#elif defined(PICO_FLASH_SIZE_LIMIT_BYTES)
#error "unknown harness variant"
#elif PICO_FLASH_SIZE_BYTES == 0x200000
    // Uncapped 2 MiB board: upstream layout, fixed marker handling.
    RUN_SCENARIO(scenario_blank_marker, 0x15, 0x10100000u, 0x10200000u); tests++;
    RUN_SCENARIO(scenario_valid_marker, 0x15, 0x10100000u, 0x10200000u); tests++; // upstream bounds
    RUN_SCENARIO(scenario_valid_marker, 0x16, 0x10100000u, 0x10200000u); tests++; // clamped, not [0x10200000,0x10400000)
#elif PICO_FLASH_SIZE_BYTES == 0x400000
    RUN_SCENARIO(scenario_valid_marker, 0x16, 0x10200000u, 0x10400000u); tests++;
#elif PICO_FLASH_SIZE_BYTES == 0x1000000
    RUN_SCENARIO(scenario_valid_marker, 0x18, 0x10800000u, 0x11000000u); tests++;
#elif PICO_FLASH_SIZE_BYTES == 0x100000
    // 1 MiB chip: the marker sector lies beyond the chip, marker handling is
    // skipped entirely even on a blank sector, and the layout is the upstream
    // upper-half range.
    printf("RUN uncapped_1mib_marker_beyond_chip\n");
    mock_reset(0x14); // blank marker sector
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes(); // no marker read-modify-write beyond the chip
    assert_single_bounds(0x10080000u, 0x10100000u);
    assert(!low_flash_storage_locked());
    tests++;
#elif PICO_FLASH_SIZE_BYTES == 0x80000
    printf("RUN uncapped_512kib_marker_beyond_chip\n");
    mock_reset(0x13);
    low_flash_init();
    assert_boot_safety();
    assert_zero_writes();
    assert_single_bounds(0x10040000u, 0x10080000u);
    assert(!low_flash_storage_locked());
    tests++;
#else
#error "unknown harness variant"
#endif
    return tests;
}

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0); // keep scenario names visible when an assert aborts
    if (mock_map_flash() != 0) {
        return 2;
    }
    int tests = run_scenarios();
    // Global property across every scenario: every recorded erase/program was
    // sector/page aligned, inside the build-time flash size, and never a
    // 4- or 8-byte program.
    assert(mock_api_violations() == 0);
    printf("low_flash harness: %d scenario(s) passed, %d recorded call(s), 0 API violations\n",
           tests, mock_ncalls);
    return 0;
}
