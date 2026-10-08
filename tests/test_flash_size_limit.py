#!/usr/bin/env python3
"""Host-only regression for low_flash_init; no SDK build or hardware simulation.

Compile the actual RP2040 boot stage of low_flash.c (phymarker stage,
low_flash_init_rp2040() and low_flash_init()) together with the real pure
flash_layout module, with small ROM, JEDEC and NOR stubs. The flash
sizing/marker logic is not reimplemented.
Run: python3 tests/test_flash_size_limit.py (requires a GCC/Clang-compatible CC).

Adapted from the fix/flash-size-limit (28cd6a4) harness when that head was
merged into the M1 storage design; see tests/README.flash-size-limit.md for
what changed and why.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src/fs/low_flash.c").read_text()
# The RP2040 boot stage: the phymarker stage, low_flash_init_rp2040() and
# low_flash_init(), up to the core1 bootstrap that never runs in these tests.
START = SOURCE.index("#ifdef PICO_RP2040\nextern uintptr_t __phymarker_start;")
INIT = SOURCE[START:SOURCE.index("void low_flash_init_core1(void)", START)]
PREFIX = SOURCE[:SOURCE.index("#define TOTAL_FLASH_PAGES")]
GUARD_START = PREFIX.find("#ifdef PICO_FLASH_SIZE_LIMIT_BYTES")
GUARD = PREFIX[GUARD_START:] if GUARD_START >= 0 else ""
FLASH_LAYOUT_C = ROOT / "src/fs/flash_layout.c"

STUBS = r"""
#define _GNU_SOURCE 1
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <sys/mman.h>
#include "flash_layout.h"
#define FLASH_SECTOR_SIZE 0x1000u
#define FLASH_PAGE_SIZE 256u
#define TOTAL_FLASH_PAGES 6
#if defined(PICO_RP2040) || defined(PICO_RP2350)
#define PICO_PLATFORM 1
#define XIP_BASE 0x10000000u
#else
#define XIP_BASE 0u
#endif
uint32_t FLASH_SIZE_BYTES;
typedef struct { unsigned unused; } page_flash_t;
page_flash_t flash_pages[TOTAL_FLASH_PAGES];
int mtx_flash;
// The boot-stage lock state lives outside the extracted region in
// low_flash.c; the harness defines the same object so the stage can flip it.
bool storage_locked;
uint8_t data_page[FLASH_PAGE_SIZE];
struct { uint8_t id[8]; } pico_serial = {{1, 2, 3, 4, 5, 6, 7, 8}};
typedef struct { const uint8_t *data; size_t len; } const_byte_array_t;
#define CONST_BYTE_ARRAY(p, n) ((const_byte_array_t){(p), (n)})
// Constant stand-ins for the SDK crc32c() behind the marker format (standard
// reflected CRC-32): a fixed value keeps build_marker_page and
// classify_marker_sector self-consistent without reimplementing the CRC.
uint32_t crc32c(const_byte_array_t data) { (void)data; return 0x12345678u; }
uint32_t stub_crc32(const uint8_t *d, size_t n) { (void)d; (void)n; return 0x12345678u; }
static unsigned capacity, bounds_calls;
static uint32_t observed_start, observed_end;
static unsigned marker_erased, marker_programmed;
static jmp_buf panic_target;
#ifndef TEST_PHYMARKER_START
#define TEST_PHYMARKER_START 0x10100000u
#endif
uintptr_t __phymarker_start = TEST_PHYMARKER_START;
// The real boot stage reads the marker sector through __phymarker_start, so
// the harness maps scratch memory at the XIP marker address the same way the
// host flash harness maps its fake chip at 0x10000000.
static uint8_t *marker_map;
__attribute__((constructor)) static void map_marker_sector(void) {
    void *p = mmap((void *)(uintptr_t)TEST_PHYMARKER_START, FLASH_SECTOR_SIZE,
                   PROT_READ | PROT_WRITE,
                   MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) {
        abort();
    }
    memset(p, 0xFF, FLASH_SECTOR_SIZE);
    marker_map = p;
}
void mutex_init(int *m) { *m = 1; }
_Noreturn void panic(const char *msg) { (void)msg; longjmp(panic_target, 1); }
void flash_set_bounds(uint32_t start, uint32_t end) {
    observed_start = start; observed_end = end; bounds_calls++;
}
void flash_do_cmd(const uint8_t *tx, uint8_t *rx, size_t n) {
    (void)tx; (void)n; rx[3] = (uint8_t)capacity;
}
// NOR-accurate marker writers: a program may only clear bits, and both calls
// record what the boot stage did to the sector.
uint32_t save_and_disable_interrupts(void) { return 42; }
void restore_interrupts(uint32_t n) { assert(n == 42); }
void flash_range_erase(uint32_t offset, size_t size) {
    assert(size == FLASH_SECTOR_SIZE);
    assert((offset & (FLASH_SECTOR_SIZE - 1)) == 0);
    memset((uint8_t *)(uintptr_t)(XIP_BASE + offset), 0xFF, size);
    marker_erased++;
}
void flash_range_program(uint32_t offset, const uint8_t *data, size_t size) {
    assert(size == FLASH_PAGE_SIZE);
    uint8_t *dst = (uint8_t *)(uintptr_t)(XIP_BASE + offset);
    for (size_t i = 0; i < size; i++) {
        assert((dst[i] & data[i]) == data[i]); // NOR: programming only clears
        dst[i] &= data[i];
    }
    marker_programmed++;
}
#define PT_INFO_PARTITION_LOCATION_AND_FLAGS 0u
#define PT_INFO_SINGLE_PARTITION 0u
#define PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_BITS 0x0000ffffu
#define PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_LSB 0
#define PICOBIN_PARTITION_LOCATION_LAST_SECTOR_BITS 0xffff0000u
#define PICOBIN_PARTITION_LOCATION_LAST_SECTOR_LSB 16
int rom_load_partition_table(uint8_t *p, size_t n, bool b) {
    (void)p; (void)n; (void)b; return 0;
}
int rom_get_partition_table_info(uint32_t *p, unsigned n, unsigned flags) {
    (void)n; (void)flags;
    p[1] = 768u | (1023u << 16); /* partition [3 MiB, 4 MiB) */
#ifdef TEST_PARTITION_FALLBACK
    return 0;
#else
    return 3;
#endif
}
void reset_usb_boot(unsigned a, unsigned b) { (void)a; (void)b; abort(); }
typedef struct { uint32_t size; } esp_partition_t;
static const esp_partition_t test_partition = {0x300000u};
const esp_partition_t *part0;
uint8_t *map;
int fd_map;
typedef int esp_partition_mmap_handle_t;
#define ESP_PARTITION_MMAP_DATA 0
const esp_partition_t *esp_partition_find_first(unsigned a, unsigned b, const char *s) {
    (void)a; (void)b; (void)s; return &test_partition;
}
int esp_partition_mmap(const esp_partition_t *p, unsigned a, unsigned b,
                      unsigned c, const void **m, esp_partition_mmap_handle_t *h) {
    (void)p; (void)a; (void)b; (void)c; (void)m; (void)h; return 0;
}
"""
MAIN = r"""
int main(int argc, char **argv) {
    assert(argc == 6);
    bool expect_locked;
    if (setjmp(panic_target)) {
        // The M1 boot stage locks the storage instead of panicking; a panic
        // here is always a failure.
        assert(!"unexpected panic");
        return 1;
    }
    if (strcmp(argv[1], "marker") == 0) {
        // Marker-stage scenarios on an uncapped 4 MiB chip, where the marker
        // sector lies below the data region and never interferes.
        capacity = 22;
        if (strcmp(argv[2], "legacy") == 0) {
            marker_map[0] = 0x53; marker_map[1] = 0x59;
            marker_map[2] = 0x45; marker_map[3] = 0x4B; // "SYEK"
        } else if (strcmp(argv[2], "valid") == 0) {
            build_marker_page(pico_serial.id, marker_map, stub_crc32);
        } else if (strcmp(argv[2], "foreign") == 0) {
            marker_map[0] = 0x00;
        } else {
            assert(strcmp(argv[2], "blank") == 0); // sector already 0xFF
        }
        expect_locked = strtoul(argv[3], NULL, 0) != 0;
        low_flash_init();
        assert(storage_locked == expect_locked);
        assert(marker_erased == strtoul(argv[4], NULL, 0));
        assert(marker_programmed == strtoul(argv[5], NULL, 0));
        assert(bounds_calls == (expect_locked ? 0u : 1u));
        if (!expect_locked) {
            assert(observed_start == 0x10200000u && observed_end == 0x10400000u);
        }
#ifdef PICO_RP2040
        if (marker_programmed) {
            uint64_t magic;
            uint16_t version, flags;
            uint32_t crc;
            uint8_t uid[FLASH_MARKER_UID_SIZE];
            memcpy(&magic, marker_map, sizeof(magic));
            memcpy(&version, marker_map + 8, sizeof(version));
            memcpy(&flags, marker_map + 10, sizeof(flags));
            memcpy(uid, marker_map + 12, sizeof(uid));
            memcpy(&crc, marker_map + 20, sizeof(crc));
            assert(magic == FLASH_MARKER_MAGIC && version == 1 && flags == 0);
            assert(crc == 0x12345678u);
            assert(memcmp(uid, pico_serial.id, sizeof(uid)) == 0);
            for (size_t i = sizeof(flash_marker_t); i < FLASH_SECTOR_SIZE; i++) {
                assert(marker_map[i] == 0xFF); // page padding and sector tail
            }
        }
#endif
        return 0;
    }
    capacity = (unsigned)strtoul(argv[1], NULL, 0);
    expect_locked = strtoul(argv[2], NULL, 0) != 0;
    low_flash_init();
    assert(storage_locked == expect_locked);
    assert(bounds_calls == (expect_locked ? 0u : 1u));
    if (!expect_locked) {
        assert(observed_start == strtoul(argv[3], NULL, 0));
        assert(observed_end == strtoul(argv[4], NULL, 0));
        assert(observed_start < observed_end);
    }
    assert(marker_erased + marker_programmed == strtoul(argv[5], NULL, 0));
    return 0;
}
"""

class FlashSizeLimitTests(unittest.TestCase):
    def compile_case(self, defines, *, reject=False, guard_only=False):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        src, exe = Path(temp.name)/"test.c", Path(temp.name)/"test"
        src.write_text(STUBS + GUARD + ("int main(void) {return 0;}\n" if guard_only else INIT + MAIN))
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
            "-fno-sanitize-recover=all", *["-D" + d for d in defines],
            "-I" + str(ROOT / "src" / "fs"), str(src), str(FLASH_LAYOUT_C), "-o", str(exe)]
        result = subprocess.run(command, text=True, capture_output=True, timeout=30)
        if reject:
            self.assertNotEqual(result.returncode, 0, f"unsafe option accepted: {defines}")
            self.assertIn("PICO_FLASH_SIZE_LIMIT_BYTES", result.stderr)
        else:
            self.assertEqual(result.returncode, 0, result.stderr)
        return exe

    def run_case(self, exe, capacity, locked, start=0, end=0, ops=0):
        result = subprocess.run([str(exe), str(capacity), str(int(locked)), hex(start), hex(end), str(ops)],
                                text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def run_marker_case(self, exe, scenario, locked, erased, programmed):
        result = subprocess.run([str(exe), "marker", scenario, str(int(locked)), str(erased), str(programmed)],
                                text=True, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_rp2040_default_and_capped_layouts(self):
        for limit in [None, "0x200000", "2097152", "0x400000", "0x800000", "0x1000000"]:
            with self.subTest(limit=limit):
                defs = ["PICO_RP2040=1"] + ([] if limit is None else ["PICO_FLASH_SIZE_LIMIT_BYTES="+limit])
                exe = self.compile_case(defs)
                for capacity in [21, 22, 23, 24]:
                    with self.subTest(limit=limit, capacity=capacity):
                        detected = 1 << capacity
                        if limit is not None and int(limit, 0) > detected:
                            # A cap larger than the detected capacity is an
                            # explicit error, never a silent "no cap".
                            self.run_case(exe, capacity, True)
                            continue
                        size = min(detected, int(limit, 0)) if limit else detected
                        # Capped builds move the data start above the marker
                        # sector; uncapped builds keep the upstream layout.
                        start = 0x101000 if (limit is not None and size // 2 <= 0x100000) else size // 2
                        self.run_case(exe, capacity, False, 0x10000000 + start, 0x10000000 + size, 1)

    def test_invalid_capacity_fails_before_marker_or_bounds(self):
        for limit in [None, "0x200000"]:
            exe = self.compile_case(["PICO_RP2040=1"] + ([] if limit is None else ["PICO_FLASH_SIZE_LIMIT_BYTES="+limit]))
            # 0, 15 and 25..255 are outside the JEDEC window (16..24); 16 is a
            # 64 KiB chip that cannot hold the minimum pool+journal headroom.
            for capacity in [0, 15, 16, 25, 31, 32, 255]:
                with self.subTest(limit=limit, capacity=capacity):
                    self.run_case(exe, capacity, True)

    def test_small_chip_marker_beyond_chip_is_skipped(self):
        exe = self.compile_case(["PICO_RP2040=1"])
        # 1 MiB chip: the marker sector lies beyond the chip, the layout is
        # still the upstream upper-half range and no marker handling happens.
        self.run_case(exe, 20, False, 0x10080000, 0x10100000, 0)

    def test_marker_inside_capped_data_range_rejected_before_write(self):
        exe = self.compile_case(["PICO_RP2040=1", "PICO_FLASH_SIZE_LIMIT_BYTES=0x200000",
                                 "TEST_PHYMARKER_START=0x101ff000u"])
        # A cap that leaves no data region above the marker sector fails
        # closed before any write: storage locked, not a panic.
        self.run_case(exe, 21, True)

    def test_invalid_cap_values_fail_closed_at_boot(self):
        # 28cd6a4 rejected these values at compile time; the M1 design
        # validates cap values at boot in compute_layout() and locks the
        # storage instead (tests/flash_harness covers the same rule).
        for limit in ["0", "-1", "0x100000", "0x101000",
                      "0x200001", "0x201000", "0x200800", "0x2000000"]:
            with self.subTest(limit=limit):
                exe = self.compile_case(["PICO_RP2040=1", "PICO_FLASH_SIZE_LIMIT_BYTES="+limit])
                self.run_case(exe, 24, True)

    def test_sector_aligned_non_power_of_two_cap(self):
        # The M1 layout rule accepts any positive sector-aligned cap that
        # leaves a valid data region; powers of two are not required.
        exe = self.compile_case(["PICO_RP2040=1", "PICO_FLASH_SIZE_LIMIT_BYTES=0x300000"])
        self.run_case(exe, 24, False, 0x10000000 + 0x180000, 0x10000000 + 0x300000, 1)

    def test_unsupported_platforms_reject_cap(self):
        for platform in [["PICO_RP2350=1"], ["ESP_PLATFORM=1"], ["ENABLE_EMULATION=1"], [],
                         ["PICO_RP2040=0"], ["PICO_RP2040=1", "PICO_RP2350=1"],
                         ["PICO_RP2040=1", "ESP_PLATFORM=1"], ["PICO_RP2040=1", "ENABLE_EMULATION=1"]]:
            with self.subTest(platform=platform):
                self.compile_case(platform+["PICO_FLASH_SIZE_LIMIT_BYTES=0x200000"], reject=True, guard_only=True)

    def test_marker_stage_scenarios(self):
        # The M1 boot stage replaces phymarker_write(): classification drives
        # the writes (VALID: none, BLANK: one full-page program plus readback,
        # LEGACY_TRUNCATED: erase plus program, FOREIGN: storage locked) via
        # the real classify_marker_sector/build_marker_page.
        exe = self.compile_case(["PICO_RP2040=1"])
        for scenario, locked, erased, programmed in [
                ("blank", False, 0, 1), ("legacy", False, 1, 1),
                ("valid", False, 0, 0), ("foreign", True, 0, 0)]:
            with self.subTest(scenario=scenario):
                self.run_marker_case(exe, scenario, locked, erased, programmed)

    def test_marker_stage_skipped_beyond_chip(self):
        exe = self.compile_case(["PICO_RP2040=1", "TEST_PHYMARKER_START=0x10800000u"])
        # Marker sector beyond the 4 MiB chip: no erase and no program even on
        # a blank sector.
        self.run_marker_case(exe, "blank", False, 0, 0)

    def test_rp2350_uncapped_partition_and_fallback(self):
        for fallback, start in [(False, 0x10300000), (True, 0x10200000)]:
            exe = self.compile_case(["PICO_RP2350=1"] + (["TEST_PARTITION_FALLBACK=1"] if fallback else []))
            self.run_case(exe, 22, False, start, 0x103fe000)

    def test_esp32_uncapped_partition(self):
        exe = self.compile_case(["ESP_PLATFORM=1"])
        self.run_case(exe, 22, False, 0, 0x300000)

    def test_emulation_without_cap_has_no_option_guard_error(self):
        self.compile_case(["ENABLE_EMULATION=1"], guard_only=True)

if __name__ == "__main__":
    unittest.main(verbosity=2)
