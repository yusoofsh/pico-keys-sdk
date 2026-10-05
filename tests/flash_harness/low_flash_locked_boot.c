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
 * Storage-locked startup continuation, one fresh process per locking cause.
 *
 * Production main() runs low_flash_init() and then, unconditionally,
 * file_scan_flash() (which includes journal recovery) before usb_init(). When
 * the boot stage locked the storage, low_flash_init() returns without
 * publishing bounds, so the continuation used to read address 0 through
 * end_rom_pool and journal recovery used to underflow its sector to
 * 0xfffff000 - both SIGSEGV in this harness with production objects.
 *
 * The production storage_locked flag latches for the life of the process, so
 * a shared process cannot prove that each cause locks by itself. Run with no
 * argument this executable execs itself once per locking cause; run with a
 * cause name the fresh process sets up that cause, boots the unmodified
 * production low_flash.c and then runs the real startup continuation (the
 * unmodified production file_scan_flash and the main-loop flash task). An
 * out-of-bounds access (address 0, 0xfffff000, or a read beyond the
 * build-time chip, which the mock keeps PROT_NONE) faults and kills the
 * child; the parent fails the suite on any child that does not exit 0, so a
 * pass proves no out-of-bounds read happened.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "picokeys.h"
#include "file.h"
#include "flash.h"
#include "flash_layout.h"
#include "low_flash_mock.h"
#include "serial.h"

extern void low_flash_init_core1(void);
extern bool low_flash_storage_locked(void);

/* Production fs state (src/fs/flash.c). */
extern const uintptr_t end_rom_pool, start_rom_pool, end_data_pool, start_data_pool;
extern const uintptr_t last_base;

#define MARKER_OFFSET MOCK_MARKER_OFFSET
#define MARKER_SECTOR (mock_flash() + MARKER_OFFSET)

/* Same helpers as low_flash_scenarios.c (kept local: both are test files). */
static const uint8_t OTHER_UID[FLASH_MARKER_UID_SIZE] = { 9, 9, 9, 9, 9, 9, 9, 9 };

extern uint32_t crc32c(const_byte_array_t data);
static uint32_t harness_crc32(const uint8_t *data, size_t len) {
    return crc32c(CONST_BYTE_ARRAY(data, len));
}

static void program_valid_marker(void) {
    build_marker_page(pico_serial.id, mock_flash_mutable() + MARKER_OFFSET, harness_crc32);
}

static uint32_t fnv1a(const uint8_t *p, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t hash_chip(void) {
    return fnv1a(mock_flash(), MOCK_CHIP_BYTES);
}

static void assert_zero_writes(void) {
    assert(mock_erase_calls() == 0);
    assert(mock_program_calls() == 0);
}

static void assert_zero_bounds_published(void) {
    assert(start_data_pool == 0 && end_data_pool == 0);
    assert(start_rom_pool == 0 && end_rom_pool == 0);
    assert(last_base == 0);
}

/* The startup continuation main() runs after a locked low_flash_init():
 * file_scan_flash (scan + first-init + journal recovery), the later
 * storage-dependent file writes, and the main loop's flash task. Everything
 * must be a safe refusal: zero recorded erase/program calls, no read outside
 * the fake flash, no usable bounds, byte-identical flash content. */
static void assert_locked_continuation(void) {
    assert(low_flash_storage_locked());
    assert_zero_bounds_published();
    for (int i = 0; i < mock_ncalls; i++) {
        assert(mock_calls[i].op != 'B'); // no bounds record at all
    }
    const uint32_t before = hash_chip();
    // The boot itself may have recorded the marker program (the readback
    // cause): the continuation must not add any erase or program.
    const int erase_at_boot = mock_erase_calls();
    const int program_at_boot = mock_program_calls();

    file_scan_flash(); // production: scan, first-init seeding, journal recovery
    assert(low_flash_storage_locked());
    assert_zero_bounds_published();
    assert(mock_erase_calls() == erase_at_boot);
    assert(mock_program_calls() == program_at_boot);
    assert(hash_chip() == before);

    // Direct journal recovery (the 0xfffff000 underflow reproduction).
    assert(low_flash_recover_journal(false) == PICOKEYS_ERR_BLOCKED);
    assert_zero_bounds_published();
    assert(mock_erase_calls() == erase_at_boot);
    assert(mock_program_calls() == program_at_boot);
    assert(hash_chip() == before);

    // Storage-dependent FIDO file writes fail with an error, never crash and
    // never wipe: allocate_free_addr would dereference end_data_pool == 0.
    uint8_t buf[16];
    memset(buf, 0xA5, sizeof(buf));
    assert(flash_write_data_to_file(ef_phy, CONST_BYTE_ARRAY(buf, sizeof(buf))) == PICOKEYS_ERR_BLOCKED);
    assert(mock_erase_calls() == erase_at_boot);
    assert(mock_program_calls() == program_at_boot);
    assert(hash_chip() == before);

    // The device keeps booting: the main loop's flash task runs and refuses.
    const uintptr_t addr = MOCK_XIP_BASE + 0x101000u + 0x4000u; // inside the data region
    assert(flash_program_block(addr, CONST_BYTE_ARRAY(buf, sizeof(buf))) == PICOKEYS_ERR_BLOCKED);
    low_flash_init_core1();
    for (int i = 0; i < 8; i++) {
        flash_task(); // what core0_loop drains every iteration
    }
    assert(low_flash_storage_locked());
    assert(mock_erase_calls() == erase_at_boot);
    assert(mock_program_calls() == program_at_boot);
    assert(mock_lockout_calls() == 0);
    assert(hash_chip() == before);
}

/* ---- per-cause setups: each locks the storage through a different branch --- */

static void cause_jedec_invalid_00(void) {
    mock_reset(0x00); // capacity exponent below the valid range
    low_flash_init();
    assert_locked_continuation();
}

static void cause_jedec_invalid_ff(void) {
    mock_reset(0xFF); // erased JEDEC response
    low_flash_init();
    assert_locked_continuation();
}

static void cause_cap_gt_chip_1m(void) {
    mock_reset(0x14); // 1 MiB chip: the 2 MiB cap exceeds it
    low_flash_init();
    assert_locked_continuation();
}

static void cause_cap_gt_chip_512k(void) {
    mock_reset(0x13); // 512 KiB chip: ditto
    low_flash_init();
    assert_locked_continuation();
}

static void cause_foreign_random(void) {
    mock_reset(0x16);
    memset(mock_flash_mutable() + MARKER_OFFSET, 0x5A, FLASH_LAYOUT_SECTOR_SIZE);
    low_flash_init();
    assert_locked_continuation();
}

static void cause_foreign_bad_crc(void) {
    mock_reset(0x16);
    program_valid_marker();
    (mock_flash_mutable() + MARKER_OFFSET)[23] ^= 0x01; // one bit of the stored CRC
    low_flash_init();
    assert_locked_continuation();
}

static void cause_foreign_other_uid(void) {
    mock_reset(0x16);
    build_marker_page(OTHER_UID, mock_flash_mutable() + MARKER_OFFSET, harness_crc32);
    low_flash_init();
    assert_locked_continuation();
}

static void cause_foreign_dirty_page(void) {
    mock_reset(0x16);
    program_valid_marker();
    (mock_flash_mutable() + MARKER_OFFSET)[100] = 0x00;
    low_flash_init();
    assert_locked_continuation();
}

static void cause_readback_mismatch(void) {
    mock_reset(0x16);
    mock_set_program_drop(true); // the marker program does not stick
    low_flash_init();
    mock_set_program_drop(false);
    assert_locked_continuation();
}

/* Positive control: an unlocked boot runs the same continuation for real -
 * the scan seeds first-init records in the RAM cache, and the main-loop task
 * flushes them once commits are allowed. Unlocked behavior must be unchanged:
 * no hardware erase/program before the flush, and every flushed operation
 * aligned and inside the chip (the mock counts violations). */
static void control_unlocked_scan(void) {
    mock_reset(0x16);
    program_valid_marker();
    low_flash_init();
    assert(!low_flash_storage_locked());
    assert(start_data_pool == MOCK_XIP_BASE + 0x101000u); // marker gap + one sector
    assert(end_data_pool == MOCK_XIP_BASE + 0x1FBFECu); // production pool math
    mock_log_reset(); // drop the boot's own records; watch the continuation only

    file_scan_flash();
    assert(!low_flash_storage_locked());
    assert_zero_writes(); // first-init only fills the RAM cache

    low_flash_init_core1();
    flash_commit(); // what flash_commit() from an app would do
    for (int i = 0; i < 8; i++) {
        flash_task();
    }
    assert(mock_api_violations() == 0);
    assert(mock_erase_calls() + mock_program_calls() > 0); // the flush really ran
}

static const struct {
    const char *name;
    void (*fn)(void);
} causes[] = {
    { "jedec-invalid-00", cause_jedec_invalid_00 },
    { "jedec-invalid-ff", cause_jedec_invalid_ff },
    { "cap-gt-chip-1m", cause_cap_gt_chip_1m },
    { "cap-gt-chip-512k", cause_cap_gt_chip_512k },
    { "foreign-random", cause_foreign_random },
    { "foreign-bad-crc", cause_foreign_bad_crc },
    { "foreign-other-uid", cause_foreign_other_uid },
    { "foreign-dirty", cause_foreign_dirty_page },
    { "readback-mismatch", cause_readback_mismatch },
    { "unlocked-scan-control", control_unlocked_scan },
};

/* ---- fresh-process runner ---- */

static int run_named(const char *name) {
    setvbuf(stdout, NULL, _IONBF, 0); // keep the cause name visible on failure
    if (mock_map_flash() != 0) {
        return 2;
    }
    for (size_t i = 0; i < sizeof(causes) / sizeof(causes[0]); i++) {
        if (strcmp(causes[i].name, name) == 0) {
            printf("RUN %s\n", name);
            causes[i].fn();
            assert(mock_api_violations() == 0); // every op aligned + inside the chip
            printf("PASS %s\n", name);
            return 0;
        }
    }
    printf("FAIL %s: unknown cause\n", name);
    return 2;
}

static int run_parent(const char *self) {
    int failures = 0;
    for (size_t i = 0; i < sizeof(causes) / sizeof(causes[0]); i++) {
        fflush(NULL);
        pid_t pid = fork();
        if (pid < 0) {
            perror("fork");
            return 2;
        }
        if (pid == 0) {
            execl(self, self, causes[i].name, (char *)NULL);
            perror("execl");
            _exit(127);
        }
        int status = 0;
        if (waitpid(pid, &status, 0) < 0) {
            printf("FAIL %s: waitpid failed\n", causes[i].name);
            failures++;
        }
        else if (WIFSIGNALED(status)) {
            // A fault (out-of-bounds read through the zero bounds, or beyond
            // the PROT_NONE chip boundary) kills the child.
            printf("FAIL %s: killed by signal %d (out-of-bounds access?)\n",
                   causes[i].name, WTERMSIG(status));
            failures++;
        }
        else if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            printf("FAIL %s: exit status %d\n", causes[i].name,
                   WIFEXITED(status) ? WEXITSTATUS(status) : -1);
            failures++;
        }
    }
    printf("locked-boot startup continuation: %zu cause(s) in fresh processes, %d failure(s)\n",
           sizeof(causes) / sizeof(causes[0]), failures);
    return failures > 0 ? 1 : 0;
}

int main(int argc, char **argv) {
    if (argc == 2) {
        return run_named(argv[1]);
    }
    char self[4096];
    ssize_t n = readlink("/proc/self/exe", self, sizeof(self) - 1);
    if (n <= 0) {
        perror("readlink /proc/self/exe");
        return 2;
    }
    self[n] = '\0';
    return run_parent(self);
}
