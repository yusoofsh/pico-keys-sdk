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
 * Pure flash geometry and physical marker helpers.
 *
 * This module has no Pico SDK, ESP or hardware dependencies and compiles on
 * the host, so the boot-time layout decisions can be unit tested. All sizes
 * and offsets are bytes measured from the start of the flash chip; callers
 * add their own XIP base to obtain absolute addresses.
 */

#ifndef _FLASH_LAYOUT_H_
#define _FLASH_LAYOUT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* On-flash v1 physical marker format (must stay identical to the format
 * programmed by upstream low_flash.c: phymarker_t / PHYSICAL_MARKER_MAGIC). */
#define FLASH_MARKER_MAGIC         0x5049434F4B455953ULL // "PICOKEYS"
#define FLASH_MARKER_VERSION       1
#define FLASH_MARKER_UID_SIZE      8                     // PICO_UNIQUE_BOARD_ID_SIZE_BYTES
#define FLASH_MARKER_PAGE_SIZE     256                   // FLASH_PAGE_SIZE
#define FLASH_LAYOUT_SECTOR_SIZE   4096                  // FLASH_SECTOR_SIZE

/* JEDEC capacity exponent window accepted by validate_jedec_capacity():
 * 2^16 (64 KiB) .. 2^24 (16 MiB), the RP2040 XIP window. Any other byte is
 * rejected before any shift is performed. A capacity that passes the window
 * may still be rejected by compute_layout() when the data region cannot hold
 * the minimum headroom (this is what rejects the 64 KiB / exponent 16 chip). */
#define FLASH_LAYOUT_JEDEC_EXP_MIN 16
#define FLASH_LAYOUT_JEDEC_EXP_MAX 24

/* Minimum data region: 4 sectors ROM pool + up to 7 journal/redo sectors
 * + at least 1 record sector (see flash_set_bounds() and the journal layout
 * in low_flash.c). */
#define FLASH_LAYOUT_MIN_DATA_SECTORS 12

typedef struct {
    uint64_t magic;                          // FLASH_MARKER_MAGIC
    uint16_t version;                        // FLASH_MARKER_VERSION
    uint16_t flags;                          // 0x0000
    uint8_t uid[FLASH_MARKER_UID_SIZE];      // pico_serial.id
    uint32_t crc32;                          // CRC-32 over the first 20 bytes
} __attribute__ ((packed)) flash_marker_t;

_Static_assert(sizeof(flash_marker_t) == 24, "physical marker v1 format changed");

typedef enum {
    FLASH_LAYOUT_RP2040 = 0, // data region is the upper half of the effective size
} flash_layout_platform_t;

typedef enum {
    FLASH_LAYOUT_OK = 0,
    FLASH_LAYOUT_ERR_PARAM,      // null output, zero sector size or zero capacity
    FLASH_LAYOUT_ERR_PLATFORM,   // platform has no layout rule in this module
    FLASH_LAYOUT_ERR_JEDEC,      // JEDEC capacity byte outside the accepted window
    FLASH_LAYOUT_ERR_CAP,        // cap not positive, not sector-aligned or unsupported
    FLASH_LAYOUT_ERR_ALIGNMENT,  // a boundary would not be sector-aligned
    FLASH_LAYOUT_ERR_RANGE,      // data range would be empty or inverted
    FLASH_LAYOUT_ERR_HEADROOM,   // data region below the minimum pool+journal size
    FLASH_LAYOUT_ERR_MARKER      // marker sector would overlap the capped data range
} flash_layout_result_t;

typedef enum {
    FLASH_MARKER_BLANK = 0,      // whole sector is 0xFF
    FLASH_MARKER_VALID,          // v1 marker for this board, rest of page 0xFF
    FLASH_MARKER_LEGACY_TRUNCATED, // exact 4-byte legacy artifact, rest 0xFF
    FLASH_MARKER_FOREIGN         // anything else
} flash_marker_class_t;

typedef struct {
    uint32_t data_start;   // inclusive offset of the data region
    uint32_t data_end;     // exclusive offset of the data region
    bool marker_skipped;   // marker sector lies beyond the chip: skip all marker handling
} flash_layout_t;

/* CRC over len bytes: standard reflected CRC-32, poly 0xEDB88320, equal to
 * Python zlib.crc32. The firmware passes a wrapper over the existing SDK
 * crc32c() (src/crypto_utils.c); host tests pass a byte-identical copy. The
 * on-flash marker format depends on this algorithm, so it must never be
 * "fixed" to a different CRC. */
typedef uint32_t (*flash_layout_crc32_t)(const uint8_t *data, size_t len);

/* Validate a JEDEC capacity exponent byte before any shift is performed.
 * Accepts exponents FLASH_LAYOUT_JEDEC_EXP_MIN..FLASH_LAYOUT_JEDEC_EXP_MAX
 * (2^16..2^24). On success returns FLASH_LAYOUT_OK and stores the capacity in
 * bytes; otherwise returns an explicit error and leaves *capacity_bytes
 * untouched, so an abnormal ID is never read as a large capacity. */
int validate_jedec_capacity(uint8_t jedec_e, uint32_t *capacity_bytes);

/* Compute the data region for the physical marker layout.
 *
 *   detected_bytes    capacity reported by the chip (validated JEDEC exponent)
 *   build_time_bytes  PICO_FLASH_SIZE_BYTES of the build
 *   cap_defined       true when PICO_FLASH_SIZE_LIMIT_BYTES is defined
 *   cap_bytes         PICO_FLASH_SIZE_LIMIT_BYTES value (ignored if !cap_defined)
 *   marker_offset     offset of the physical marker sector start
 *   sector_size       flash sector size in bytes (4096)
 *   platform          FLASH_LAYOUT_RP2040; other platforms keep their native
 *                     layout code in low_flash.c and are rejected here
 *
 * Order: (1) clamp to build_time_bytes; (2) apply the cap only if defined,
 * positive, sector-aligned and not larger than the clamped capacity (a larger
 * cap is an explicit error, never a silent "no cap"); (3) RP2040 data region
 * is the upper half of the effective size; (4) in capped builds only, move the
 * data start above the marker sector when it would overlap; (5) enforce
 * start < end, sector alignment and the minimum pool+journal headroom; (6)
 * return an explicit error, never a silently inverted or empty range.
 *
 * If the marker sector lies beyond the physical chip (e.g. a 1 MiB chip), the
 * layout is still valid and out->marker_skipped is true: the caller must skip
 * all marker handling (no erase, no program). A cap that leaves no valid data
 * region above an in-chip marker sector is an explicit error. */
int compute_layout(uint32_t detected_bytes, uint32_t build_time_bytes,
                   bool cap_defined, uint32_t cap_bytes,
                   uint32_t marker_offset, uint32_t sector_size,
                   flash_layout_platform_t platform, flash_layout_t *out);

/* Classify a 4096-byte marker sector for the given board UID.
 * A NULL sector, uid or crc32 fails closed as FLASH_MARKER_FOREIGN. */
flash_marker_class_t classify_marker_sector(const uint8_t sector[FLASH_LAYOUT_SECTOR_SIZE],
                                            const uint8_t uid[FLASH_MARKER_UID_SIZE],
                                            flash_layout_crc32_t crc32);

/* Build the full 256-byte, 0xFF-padded v1 marker page for a board UID. The
 * CRC is taken from the injected callback so the format stays identical to
 * the one produced with the SDK crc32c(). */
void build_marker_page(const uint8_t uid[FLASH_MARKER_UID_SIZE],
                       uint8_t page[FLASH_MARKER_PAGE_SIZE],
                       flash_layout_crc32_t crc32);

#endif
