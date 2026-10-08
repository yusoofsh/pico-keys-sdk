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

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "flash_layout.h"

// Byte-identical host copy of the SDK crc32c() algorithm (src/crypto_utils.c):
// standard reflected CRC-32, poly 0xEDB88320, equal to Python zlib.crc32.
// The pure flash_layout module takes the CRC as an injected callback, so the
// firmware passes a wrapper over the SDK crc32c() and the tests pass this copy.
#define POLY 0xedb88320

static uint32_t test_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (POLY & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

#define XIP_BASE_RP2040 0x10000000u

static const uint8_t UID[FLASH_MARKER_UID_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
static const uint8_t UID_OTHER[FLASH_MARKER_UID_SIZE] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};

// ---------------------------------------------------------------------------

static void test_jedec_validation(void) {
    uint32_t cap = 0xDEADBEEF;
    // Rejected before any shift, no capacity value produced.
    const uint8_t rejected[] = {0x00, 0x0F, 0x19, 0x1F, 0x20, 0x7F, 0xFF};
    for (size_t i = 0; i < sizeof(rejected); i++) {
        assert(validate_jedec_capacity(rejected[i], &cap) == FLASH_LAYOUT_ERR_JEDEC);
        assert(cap == 0xDEADBEEF);
    }
    // Accepted window is 2^16..2^24 (64 KiB..16 MiB, the RP2040 XIP window).
    const struct { uint8_t e; uint32_t bytes; } accepted[] = {
        {0x10, 0x00010000}, {0x13, 0x00080000}, {0x14, 0x00100000},
        {0x15, 0x00200000}, {0x16, 0x00400000}, {0x17, 0x00800000},
        {0x18, 0x01000000},
    };
    for (size_t i = 0; i < sizeof(accepted) / sizeof(accepted[0]); i++) {
        cap = 0;
        assert(validate_jedec_capacity(accepted[i].e, &cap) == FLASH_LAYOUT_OK);
        assert(cap == accepted[i].bytes);
    }
    assert(validate_jedec_capacity(0x16, NULL) == FLASH_LAYOUT_ERR_PARAM);
}

// 4 MiB chip with the 2 MiB cap: data region [0x10101000, 0x10200000) absolute,
// whatever the detected capacity reports (the clamp runs before the cap).
static void test_layout_4m_cap2m(void) {
    const uint32_t detected[] = {0x00400000, 0x00800000, 0x01000000}; // JEDEC 0x16, 0x17, 0x18
    for (size_t i = 0; i < sizeof(detected) / sizeof(detected[0]); i++) {
        flash_layout_t l = {0xFFFFFFFF, 0xFFFFFFFF, true};
        assert(compute_layout(detected[i], 0x00400000, true, 0x00200000,
                              0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                              FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
        assert(l.data_start == 0x101000);
        assert(l.data_end == 0x200000);
        assert(l.marker_skipped == false);
        // Absolute XIP addresses as flash_set_bounds sees them.
        assert(l.data_start + XIP_BASE_RP2040 == 0x10101000);
        assert(l.data_end + XIP_BASE_RP2040 == 0x10200000);
        // The marker sector sits below the data region.
        assert(l.data_start >= 0x100000 + FLASH_LAYOUT_SECTOR_SIZE);
        assert(l.data_end <= 0x200000);
    }
}

// Without a cap the upstream upper-half layout is preserved, marker excluded or not.
static void test_layout_uncapped_unchanged(void) {
    flash_layout_t l;
    const struct { uint32_t size; uint32_t start, end; } cases[] = {
        {0x00200000, 0x100000, 0x200000}, // upstream 2 MiB: marker sector is the lowest data sector
        {0x00400000, 0x200000, 0x400000}, // upstream 4 MiB
        {0x01000000, 0x800000, 0x1000000}, // upstream 16 MiB
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        assert(compute_layout(cases[i].size, cases[i].size, false, 0,
                              0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                              FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
        assert(l.data_start == cases[i].start);
        assert(l.data_end == cases[i].end);
        assert(l.marker_skipped == false);
        assert(l.data_start != 0x101000); // the a26c831 unconditional-guard regression
    }
}

// The detected capacity is clamped to the build-time PICO_FLASH_SIZE_BYTES
// before the cap and before the platform layout.
static void test_layout_clamps_to_build_time(void) {
    flash_layout_t l;
    // (a) uncapped: 4 MiB detected on a 2 MiB board -> 2 MiB layout, not 4 MiB.
    assert(compute_layout(0x00400000, 0x00200000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
    assert(l.data_start == 0x100000 && l.data_end == 0x200000);
    // (b) capped: 16 MiB detected on a 2 MiB board with cap 2 MiB.
    assert(compute_layout(0x01000000, 0x00200000, true, 0x00200000,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
    assert(l.data_start == 0x101000 && l.data_end == 0x200000);
}

// Invalid caps are rejected with explicit errors and no data range is produced.
static void test_layout_cap_invalid(void) {
    flash_layout_t l;
    // 0: a defined cap must be positive; it is never silently treated as "no cap".
    assert(compute_layout(0x00400000, 0x00400000, true, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_CAP);
    // 0x200800: not sector-aligned.
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00200800,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_CAP);
    // 0x800000: larger than the 4 MiB effective capacity, rejected explicitly.
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00800000,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_CAP);
    // 0x100000: too small, no valid data region above the marker sector.
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00100000,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_RANGE);
    // No invalid input may leave a consumable range behind.
    assert(l.data_start == 0 && l.data_end == 0 && l.marker_skipped == false);
}

// On a 1 MiB or 512 KiB chip the marker sector at 0x100000 lies beyond the chip:
// the layout stays valid and carries the "marker skipped" flag.
static void test_layout_marker_skipped(void) {
    flash_layout_t l;
    assert(compute_layout(0x00100000, 0x00100000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
    assert(l.data_start == 0x80000 && l.data_end == 0x100000);
    assert(l.marker_skipped == true);
    assert(compute_layout(0x00080000, 0x00080000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_OK);
    assert(l.data_start == 0x40000 && l.data_end == 0x80000);
    assert(l.marker_skipped == true);
    // A cap larger than the physical chip is rejected explicitly.
    assert(compute_layout(0x00100000, 0x00100000, true, 0x00200000,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_CAP);
    assert(l.data_start == 0 && l.data_end == 0);
}

// Explicit errors for inverted/empty ranges, misaligned boundaries and
// insufficient pool+journal headroom.
static void test_layout_range_errors(void) {
    flash_layout_t l;
    // start > end: cap so small that the data region cannot start above the marker.
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00100000,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_RANGE);
    // start == end: the marker bump consumes the whole data region.
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00102000,
                          0x101000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_RANGE);
    // Misaligned build-time size.
    assert(compute_layout(0x00400000, 0x00200001, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_ALIGNMENT);
    // Misaligned marker offset (marker inside the chip).
    assert(compute_layout(0x00400000, 0x00400000, true, 0x00200000,
                          0x100001, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_ALIGNMENT);
    // Misaligned data start (odd effective size).
    assert(compute_layout(0x00003000, 0x00003000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_ALIGNMENT);
    // Smaller than FLASH_LAYOUT_MIN_DATA_SECTORS (ROM pool + journal + 1 data sector).
    assert(compute_layout(0x00010000, 0x00010000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_HEADROOM);
    // This also rejects the 64 KiB capacity (JEDEC 0x10) by headroom.
    uint32_t cap = 0;
    assert(validate_jedec_capacity(0x10, &cap) == FLASH_LAYOUT_OK);
    assert(compute_layout(cap, cap, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_HEADROOM);
    // Bad platform, null out, zero sector.
    assert(compute_layout(0x00400000, 0x00400000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          (flash_layout_platform_t)99, &l) == FLASH_LAYOUT_ERR_PLATFORM);
    assert(compute_layout(0x00400000, 0x00400000, false, 0,
                          0x100000, FLASH_LAYOUT_SECTOR_SIZE,
                          FLASH_LAYOUT_RP2040, NULL) == FLASH_LAYOUT_ERR_PARAM);
    assert(compute_layout(0x00400000, 0x00400000, false, 0,
                          0x100000, 0,
                          FLASH_LAYOUT_RP2040, &l) == FLASH_LAYOUT_ERR_PARAM);
}

// ---------------------------------------------------------------------------

static void fill_ff(uint8_t *buf, size_t len) {
    memset(buf, 0xFF, len);
}

static void test_marker_classify(void) {
    uint8_t sector[FLASH_LAYOUT_SECTOR_SIZE];
    uint8_t page[FLASH_MARKER_PAGE_SIZE];

    // All 0xFF is BLANK.
    fill_ff(sector, sizeof(sector));
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_BLANK);

    // A built page with the matching UID is VALID.
    build_marker_page(UID, page, test_crc32);
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, FLASH_MARKER_PAGE_SIZE);
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_VALID);

    // The same page with a different UID is FOREIGN.
    assert(classify_marker_sector(sector, UID_OTHER, test_crc32) == FLASH_MARKER_FOREIGN);

    // One flipped CRC bit is FOREIGN.
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, FLASH_MARKER_PAGE_SIZE);
    sector[20] ^= 0x01;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // Version 2 is FOREIGN.
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, FLASH_MARKER_PAGE_SIZE);
    sector[8] = 0x02;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // A valid page with any non-0xFF byte in bytes 24..255 is FOREIGN.
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, FLASH_MARKER_PAGE_SIZE);
    sector[24] = 0x00;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, FLASH_MARKER_PAGE_SIZE);
    sector[255] = 0xAA;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // The exact legacy artifact 53 59 45 4B ("SYEK") followed by 0xFF is LEGACY_TRUNCATED.
    fill_ff(sector, sizeof(sector));
    sector[0] = 0x53; sector[1] = 0x59; sector[2] = 0x45; sector[3] = 0x4B;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_LEGACY_TRUNCATED);

    // One non-0xFF byte anywhere later in the sector makes it FOREIGN.
    fill_ff(sector, sizeof(sector));
    sector[0] = 0x53; sector[1] = 0x59; sector[2] = 0x45; sector[3] = 0x4B;
    sector[4] = 0x00;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);
    fill_ff(sector, sizeof(sector));
    sector[0] = 0x53; sector[1] = 0x59; sector[2] = 0x45; sector[3] = 0x4B;
    sector[4095] = 0x7F;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // "PICO" (50 49 43 4F) followed by 0xFF is FOREIGN.
    fill_ff(sector, sizeof(sector));
    sector[0] = 0x50; sector[1] = 0x49; sector[2] = 0x43; sector[3] = 0x4F;
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // All 0x00 is FOREIGN.
    memset(sector, 0x00, sizeof(sector));
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_FOREIGN);

    // Fail closed on unusable inputs.
    fill_ff(sector, sizeof(sector));
    assert(classify_marker_sector(NULL, UID, test_crc32) == FLASH_MARKER_FOREIGN);
    assert(classify_marker_sector(sector, NULL, test_crc32) == FLASH_MARKER_FOREIGN);
    assert(classify_marker_sector(sector, UID, NULL) == FLASH_MARKER_FOREIGN);
}

// The on-flash v1 format is unchanged: 24-byte packed struct, CRC-32 over the
// first 20 bytes, full 256-byte 0xFF-padded page.
static void test_marker_page_format(void) {
    uint8_t page[FLASH_MARKER_PAGE_SIZE];

    memset(page, 0, sizeof(page));
    build_marker_page(UID, page, test_crc32);

    // Little-endian magic 0x5049434F4B455953 ("PICOKEYS").
    assert(memcmp(page, "\x53\x59\x45\x4B\x4F\x43\x49\x50", 8) == 0);
    // version = 1, flags = 0, little-endian.
    assert(page[8] == 0x01 && page[9] == 0x00);
    assert(page[10] == 0x00 && page[11] == 0x00);
    // uid at offset 12.
    assert(memcmp(page + 12, UID, FLASH_MARKER_UID_SIZE) == 0);
    // crc32 at offset 20, over the first 20 bytes.
    uint32_t crc = test_crc32(page, 20);
    assert(memcmp(page + 20, &crc, sizeof(crc)) == 0);
    // The rest of the page is 0xFF.
    for (size_t i = 24; i < FLASH_MARKER_PAGE_SIZE; i++) {
        assert(page[i] == 0xFF);
    }
    // And it classifies as VALID for its own UID.
    uint8_t sector[FLASH_LAYOUT_SECTOR_SIZE];
    fill_ff(sector, sizeof(sector));
    memcpy(sector, page, sizeof(page));
    assert(classify_marker_sector(sector, UID, test_crc32) == FLASH_MARKER_VALID);

    // A second build is deterministic.
    uint8_t page2[FLASH_MARKER_PAGE_SIZE];
    build_marker_page(UID, page2, test_crc32);
    assert(memcmp(page, page2, FLASH_MARKER_PAGE_SIZE) == 0);
}

int main(void) {
    test_jedec_validation();
    test_layout_4m_cap2m();
    test_layout_uncapped_unchanged();
    test_layout_clamps_to_build_time();
    test_layout_cap_invalid();
    test_layout_marker_skipped();
    test_layout_range_errors();
    test_marker_classify();
    test_marker_page_format();
    puts("flash_layout_test: OK");
    return 0;
}
