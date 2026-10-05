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

#include "flash_layout.h"

#include <string.h>

int validate_jedec_capacity(uint8_t jedec_e, uint32_t *capacity_bytes) {
    if (capacity_bytes == NULL) {
        return FLASH_LAYOUT_ERR_PARAM;
    }
    // Reject out-of-range exponents before any shift: a shift by 32 or more is
    // undefined behavior on the host, and an abnormal ID must never be read as
    // a large capacity.
    if (jedec_e < FLASH_LAYOUT_JEDEC_EXP_MIN || jedec_e > FLASH_LAYOUT_JEDEC_EXP_MAX) {
        return FLASH_LAYOUT_ERR_JEDEC;
    }
    *capacity_bytes = 1u << jedec_e;
    return FLASH_LAYOUT_OK;
}

int compute_layout(uint32_t detected_bytes, uint32_t build_time_bytes,
                   bool cap_defined, uint32_t cap_bytes,
                   uint32_t marker_offset, uint32_t sector_size,
                   flash_layout_platform_t platform, flash_layout_t *out) {
    if (out == NULL || sector_size == 0 || detected_bytes == 0 || build_time_bytes == 0) {
        return FLASH_LAYOUT_ERR_PARAM;
    }
    if (platform != FLASH_LAYOUT_RP2040) {
        // RP2350 and ESP32 keep their native layout code in low_flash.c.
        return FLASH_LAYOUT_ERR_PLATFORM;
    }
    out->data_start = 0;
    out->data_end = 0;
    out->marker_skipped = false;

    // Step 1: clamp the detected capacity to the build-time PICO_FLASH_SIZE_BYTES.
    uint64_t phys = (uint64_t)(detected_bytes < build_time_bytes ? detected_bytes : build_time_bytes);
    if ((phys % sector_size) != 0) {
        return FLASH_LAYOUT_ERR_ALIGNMENT;
    }

    // The marker sector is handled only while it lies inside the physical chip.
    bool marker_skipped = ((uint64_t)marker_offset + sector_size > phys);
    if (!marker_skipped && (marker_offset % sector_size) != 0) {
        return FLASH_LAYOUT_ERR_ALIGNMENT;
    }

    // Step 2: apply the cap only if it is defined, positive, sector-aligned and
    // not larger than the clamped capacity. Anything else is an explicit error,
    // never a silent "no cap".
    uint64_t effective = phys;
    if (cap_defined) {
        if (cap_bytes == 0 || (cap_bytes % sector_size) != 0 || (uint64_t)cap_bytes > phys) {
            return FLASH_LAYOUT_ERR_CAP;
        }
        effective = cap_bytes;
    }

    // Step 3: the RP2040 data region is the upper half of the effective size.
    uint64_t data_start = effective / 2;
    uint64_t data_end = effective;

    // Step 4: in capped builds only, keep the data pool off the marker sector.
    if (cap_defined && !marker_skipped && data_start <= (uint64_t)marker_offset) {
        data_start = (uint64_t)marker_offset + sector_size;
    }

    // Step 5: range, alignment and headroom guards. Never yield an inverted or
    // empty range.
    if ((data_start % sector_size) != 0 || (data_end % sector_size) != 0) {
        return FLASH_LAYOUT_ERR_ALIGNMENT;
    }
    if (data_start >= data_end) {
        return FLASH_LAYOUT_ERR_RANGE;
    }
    if (data_end - data_start < (uint64_t)FLASH_LAYOUT_MIN_DATA_SECTORS * sector_size) {
        return FLASH_LAYOUT_ERR_HEADROOM;
    }
    // Defense in depth: in capped builds the marker sector must not overlap the
    // data range. Uncapped builds keep the upstream layout, where the marker
    // may be the lowest data sector.
    if (cap_defined && !marker_skipped &&
        (uint64_t)marker_offset < data_end && (uint64_t)marker_offset + sector_size > data_start) {
        return FLASH_LAYOUT_ERR_MARKER;
    }

    // Step 6: explicit success.
    out->data_start = (uint32_t)data_start;
    out->data_end = (uint32_t)data_end;
    out->marker_skipped = marker_skipped;
    return FLASH_LAYOUT_OK;
}

flash_marker_class_t classify_marker_sector(const uint8_t sector[FLASH_LAYOUT_SECTOR_SIZE],
                                            const uint8_t uid[FLASH_MARKER_UID_SIZE],
                                            flash_layout_crc32_t crc32) {
    if (sector == NULL || uid == NULL || crc32 == NULL) {
        return FLASH_MARKER_FOREIGN; // fail closed
    }
    bool blank = true;
    for (size_t i = 0; i < FLASH_LAYOUT_SECTOR_SIZE; i++) {
        if (sector[i] != 0xFF) {
            blank = false;
            break;
        }
    }
    if (blank) {
        return FLASH_MARKER_BLANK;
    }
    // The exact artifact of the old sizeof(pointer) program on RP2040: the
    // little-endian low half of the magic ("SYEK"), rest of the sector 0xFF.
    static const uint8_t legacy_prefix[4] = {0x53, 0x59, 0x45, 0x4B};
    if (memcmp(sector, legacy_prefix, sizeof(legacy_prefix)) == 0) {
        bool rest_blank = true;
        for (size_t i = sizeof(legacy_prefix); i < FLASH_LAYOUT_SECTOR_SIZE; i++) {
            if (sector[i] != 0xFF) {
                rest_blank = false;
                break;
            }
        }
        if (rest_blank) {
            return FLASH_MARKER_LEGACY_TRUNCATED;
        }
    }
    flash_marker_t marker;
    memcpy(&marker, sector, sizeof(marker));
    if (marker.magic == FLASH_MARKER_MAGIC && marker.version == FLASH_MARKER_VERSION &&
        crc32(sector, sizeof(marker) - sizeof(uint32_t)) == marker.crc32 &&
        memcmp(marker.uid, uid, FLASH_MARKER_UID_SIZE) == 0) {
        bool page_rest_blank = true;
        for (size_t i = sizeof(marker); i < FLASH_MARKER_PAGE_SIZE; i++) {
            if (sector[i] != 0xFF) {
                page_rest_blank = false;
                break;
            }
        }
        if (page_rest_blank) {
            return FLASH_MARKER_VALID;
        }
    }
    return FLASH_MARKER_FOREIGN;
}

void build_marker_page(const uint8_t uid[FLASH_MARKER_UID_SIZE],
                       uint8_t page[FLASH_MARKER_PAGE_SIZE],
                       flash_layout_crc32_t crc32) {
    if (uid == NULL || page == NULL || crc32 == NULL) {
        return; // documented precondition; nothing safe to build
    }
    flash_marker_t marker;
    memset(&marker, 0, sizeof(marker));
    marker.magic = FLASH_MARKER_MAGIC;
    marker.version = FLASH_MARKER_VERSION;
    marker.flags = 0x0000;
    memcpy(marker.uid, uid, FLASH_MARKER_UID_SIZE);
    marker.crc32 = crc32((const uint8_t *)&marker, sizeof(marker) - sizeof(uint32_t));
    memset(page, 0xFF, FLASH_MARKER_PAGE_SIZE);
    memcpy(page, &marker, sizeof(marker));
}
