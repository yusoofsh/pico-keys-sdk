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

#include "puf.h"

#include <stdbool.h>
#include <stddef.h>

const uint8_t *puf_key = NULL;
puf_status_t puf_status = PUF_STATUS_OFF;

#if defined(PICO_RP2040) && !defined(ENABLE_EMULATION)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "picokeys.h"
#include "file.h"
#include "flash.h"
#include "random.h"
#include "led/led.h"
#include "puf_fe.h"
#include "mbedtls/platform_util.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/structs/watchdog.h"

#define EF_PUF_HELPER 0xE5F0

// The lower half of SCRATCH_X. Nothing is linked there (.scratch_x is empty),
// the heap ends below it and core1's stack occupies only the upper half; the
// bootrom does not touch the bank on a flash boot. Nothing ever writes it, so
// after a warm reset it still holds the last power-up pattern.
#define PUF_SRAM_BASE 0x20040000u

extern uintptr_t __phymarker_start;
extern char __scratch_x_end__[], __StackOneBottom[], __HeapLimit[], __flash_binary_end[];

static uint8_t key_buf[PUF_ROOT_KEY_SIZE];

static bool layout_ok(uintptr_t scratch_addr) {
    const uintptr_t lo = PUF_SRAM_BASE, hi = PUF_SRAM_BASE + PUF_SAMPLE_BYTES;
    return (uintptr_t)__scratch_x_end__ <= lo && (uintptr_t)__HeapLimit <= lo &&
           (uintptr_t)__StackOneBottom >= hi && (uintptr_t)__flash_binary_end <= scratch_addr;
}

// A real power-on: the chip-level reset came from POR and the watchdog did
// not reboot it since (watchdog reboots keep the chip_reset bits).
static bool power_on_reset(void) {
    return (vreg_and_chip_reset_hw->chip_reset & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS) != 0 &&
           watchdog_hw->reason == 0;
}

static void rng(uint8_t *buf, size_t len) {
    random_fill_buffer(BYTE_ARRAY(buf, len));
}

// The enrollment scratch lives in its own sector just below the storage
// marker, outside the file system, so erasing it really destroys the raw
// samples. Runs before core1 exists: disabling interrupts is enough.
static void scratch_erase(uintptr_t addr) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(addr - XIP_BASE, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

#define SCRATCH_PROGRAM_SIZE (((sizeof(puf_scratch_t) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)
_Static_assert(SCRATCH_PROGRAM_SIZE <= FLASH_SECTOR_SIZE, "PUF scratch must fit one sector");

static void scratch_write(uintptr_t addr, const puf_scratch_t *scratch) {
    uint8_t *page = malloc(SCRATCH_PROGRAM_SIZE);
    if (page == NULL) {
        return;
    }
    memset(page, 0xFF, SCRATCH_PROGRAM_SIZE);
    memcpy(page, scratch, sizeof(*scratch));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(addr - XIP_BASE, FLASH_SECTOR_SIZE);
    flash_range_program(addr - XIP_BASE, page, SCRATCH_PROGRAM_SIZE);
    restore_interrupts(ints);
    mbedtls_platform_zeroize(page, SCRATCH_PROGRAM_SIZE);
    free(page);
}

void puf_init(void) {
    if (low_flash_storage_locked()) {
        return;
    }
    const uintptr_t scratch_addr = __phymarker_start - FLASH_SECTOR_SIZE;
    file_t *ef = file_search(EF_PUF_HELPER);
    const bool helper_present = file_has_data(ef);
    puf_scratch_t *scratch = malloc(sizeof(*scratch));
    puf_helper_t *helper = calloc(1, sizeof(*helper));
    if (!layout_ok(scratch_addr) || scratch == NULL || helper == NULL) {
        printf("PUF: unavailable (layout or memory)\n");
        puf_status = helper_present ? PUF_STATUS_FAILED : PUF_STATUS_OFF;
        free(scratch);
        free(helper);
        return;
    }
    memcpy(scratch, (const void *)scratch_addr, sizeof(*scratch));
    if (helper_present && file_get_size(ef) == sizeof(*helper)) {
        memcpy(helper, file_get_data(ef), sizeof(*helper));
    }

    puf_actions_t act;
    puf_boot_result_t r = puf_boot_step((const uint8_t *)PUF_SRAM_BASE, power_on_reset(),
                                        scratch, helper, helper_present, rng, key_buf, &act);
    if (act.scratch_erase) {
        scratch_erase(scratch_addr);
    }
    if (act.scratch_write) {
        scratch_write(scratch_addr, scratch);
    }
    if (act.helper_delete) {
        file_delete(ef);
    }
    if (act.helper_write) {
        file_t *out = file_new(EF_PUF_HELPER);
        if (out) {
            file_put_data(out, CONST_BYTE_ARRAY((const uint8_t *)helper, sizeof(*helper)));
            flash_commit();
        }
    }
    mbedtls_platform_zeroize(scratch, sizeof(*scratch));
    free(scratch);
    free(helper);

    if (r == PUF_BOOT_KEY) {
        puf_key = key_buf;
        puf_status = PUF_STATUS_READY;
    }
    else if (r == PUF_BOOT_FAILED) {
        mbedtls_platform_zeroize(key_buf, sizeof(key_buf));
        puf_status = PUF_STATUS_FAILED;
    }
    else {
        puf_status = PUF_STATUS_ENROLLING;
    }
    printf("PUF: status %d\n", (int)puf_status);
}

#define MODE_PUF_FAILED ((MAX_BTNESS << LED_BTNESS_SHIFT) | (LED_COLOR_RED << LED_COLOR_SHIFT) | (100 << LED_ON_SHIFT) | (100 << LED_OFF_SHIFT))

void puf_halt_if_failed(void) {
    if (puf_status != PUF_STATUS_FAILED) {
        return;
    }
    printf("PUF: root key not reproduced, power cycle the device\n");
    led_set_mode(MODE_PUF_FAILED);
    for (;;) {
        led_blinking_task();
    }
}

#else

void puf_init(void) {
}

void puf_halt_if_failed(void) {
}

#endif
