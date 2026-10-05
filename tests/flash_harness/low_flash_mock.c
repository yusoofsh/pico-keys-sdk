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
 * harness. See low_flash_mock.h for the contract.
 */
#define _GNU_SOURCE
#include "low_flash_mock.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "byte_array.h"
#include "serial.h"
#include "apdu.h"

mock_call_t mock_calls[MOCK_MAX_CALLS];
int mock_ncalls = 0;

static uint8_t *flash_mem = NULL;
static uint8_t mock_jedec_e = 0x16; // 4 MiB
static bool mock_program_drop = false;
static int api_violations = 0;
static int irq_disabled = 0;
static int lockout_depth = 0;
static int lockout_calls = 0;

/* Symbols low_flash.c expects from serial.c / crypto_utils.c. The fs state
 * (FLASH_SIZE_BYTES, pool bounds, num_files) is the production one from
 * src/fs/flash.c, which the harness links unmodified. */
picokey_serial_t pico_serial = { .id = { 1, 2, 3, 4, 5, 6, 7, 8 } };

/* file.c references the APDU state through apdu.h globals; the harness never
 * processes APDUs, but the storage-locked startup must not touch them anyway. */
static uint8_t apdu_rdata[672];
struct apdu apdu = { .rdata = apdu_rdata };

/* Host copy of the SDK crc32c() (src/crypto_utils.c): standard reflected
 * CRC-32, poly 0xEDB88320, byte-identical to the firmware implementation. */
uint32_t crc32c(const_byte_array_t data) {
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < data.len; i++) {
        crc ^= data.data[i];
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static void record(char op, uint32_t offs, size_t count) {
    if (mock_ncalls < MOCK_MAX_CALLS) {
        mock_calls[mock_ncalls++] = (mock_call_t){ op, offs, count, irq_disabled > 0, lockout_depth };
    }
}

int mock_map_flash(void) {
    if (flash_mem != NULL) {
        return 0;
    }
    flash_mem = mmap((void *)(uintptr_t)MOCK_XIP_BASE, MOCK_MAP_SIZE, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (flash_mem != (uint8_t *)(uintptr_t)MOCK_XIP_BASE) {
        perror("mmap fake flash at the XIP address");
        return -1;
    }
    memset(flash_mem, 0xFF, MOCK_CHIP_BYTES);
    if (MOCK_MAP_SIZE > MOCK_CHIP_BYTES) {
        // Reads beyond the build-time chip must fault like they would on real
        // hardware; keep them out of the mapping entirely.
        if (mprotect(flash_mem + MOCK_CHIP_BYTES, MOCK_MAP_SIZE - MOCK_CHIP_BYTES, PROT_NONE) != 0) {
            perror("mprotect beyond-chip XIP window");
            return -1;
        }
    }
    return 0;
}

void mock_reset(uint8_t jedec_capacity_e) {
    memset(flash_mem, 0xFF, MOCK_CHIP_BYTES);
    mock_jedec_e = jedec_capacity_e;
    mock_program_drop = false;
    api_violations = 0;
    irq_disabled = 0;
    lockout_depth = 0;
    lockout_calls = 0;
    mock_log_reset();
}

void mock_log_reset(void) {
    mock_ncalls = 0;
}

void mock_set_program_drop(bool drop) {
    mock_program_drop = drop;
}

const uint8_t *mock_flash(void) {
    return flash_mem;
}

uint8_t *mock_flash_mutable(void) {
    return flash_mem;
}

int mock_api_violations(void) {
    return api_violations;
}

int mock_lockout_calls(void) {
    return lockout_calls;
}

int mock_erase_calls(void) {
    int n = 0;
    for (int i = 0; i < mock_ncalls; i++) {
        n += mock_calls[i].op == 'E';
    }
    return n;
}

int mock_program_calls(void) {
    int n = 0;
    for (int i = 0; i < mock_ncalls; i++) {
        n += mock_calls[i].op == 'P';
    }
    return n;
}

/* ---- mocked pico-sdk flash hardware ---- */

void flash_range_erase(uint32_t offs, size_t count) {
    record('E', offs, count);
    if ((offs % 4096u) != 0 || (count % 4096u) != 0 ||
        (uint64_t)offs + count > PICO_FLASH_SIZE_BYTES) {
        api_violations++;
        return;
    }
    memset(flash_mem + offs, 0xFF, count);
}

void flash_range_program(uint32_t offs, const uint8_t *data, size_t count) {
#ifdef MOCK_ARM32_POINTERS
    // phymarker_write used to pass sizeof(uint8_t *): 4 on Cortex-M0+, 8 on
    // x86_64. Model the 32-bit target so the old bug shows up as a recorded
    // 4-byte program instead of hiding behind the 64-bit host size.
    if (count == sizeof(void *)) {
        count = 4;
    }
#endif
    record('P', offs, count);
    if (count == 4 || count == 8) {
        api_violations++; // the sizeof(pointer) marker bug, never acceptable
    }
    if ((offs % 256u) != 0 || (count % 256u) != 0 ||
        (uint64_t)offs + count > PICO_FLASH_SIZE_BYTES) {
        api_violations++;
    }
    if (mock_program_drop || (uint64_t)offs + count > MOCK_MAP_SIZE) {
        return;
    }
    // NOR semantics: programming can only clear bits.
    for (size_t i = 0; i < count; i++) {
        flash_mem[offs + i] &= data[i];
    }
}

void flash_do_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count) {
    record('C', txbuf[0], count);
    if (txbuf[0] == 0x9F && count >= 4) {
        rxbuf[1] = 0xEF;
        rxbuf[2] = 0x40;
        rxbuf[3] = mock_jedec_e;
    }
}

/* ---- mocked pico-sdk support functions ---- */

/* The production flash_set_bounds body (src/fs/flash.c) is compiled under the
 * alias pico_flash_set_bounds (tests/CMakeLists.txt), so the harness records
 * every bounds publication and then lets the real pool math run. */
void pico_flash_set_bounds(uintptr_t start, uintptr_t end);
void flash_set_bounds(uintptr_t start, uintptr_t end) {
    record('B', (uint32_t)start, (size_t)end);
    pico_flash_set_bounds(start, end);
}

uint32_t save_and_disable_interrupts(void) {
    irq_disabled++;
    return 0;
}

void restore_interrupts(uint32_t ints) {
    (void)ints;
    irq_disabled--;
}

bool multicore_lockout_start_timeout_us(uint64_t us) {
    (void)us;
    lockout_calls++;
    lockout_depth++;
    return true;
}

bool multicore_lockout_end_timeout_us(uint64_t us) {
    (void)us;
    lockout_calls++;
    lockout_depth--;
    return true;
}

void multicore_lockout_victim_init(void) {}
unsigned get_core_num(void) { return 0; }
uint32_t board_millis(void) { return 0; }

void reset_usb_boot(uint32_t a, uint32_t b) {
    // The boot stage must never reboot the device; a locked device keeps booting.
    (void)a; (void)b;
    fprintf(stderr, "FATAL: reset_usb_boot called from the boot stage\n");
    abort();
}

int rom_load_partition_table(uint8_t *wa, uint32_t sz, bool force) {
    (void)wa; (void)sz; (void)force;
    return 0;
}

int rom_get_partition_table_info(uint32_t *out, uint32_t words, uint32_t flags) {
    (void)out; (void)words; (void)flags;
    return 0;
}
