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
 * SRAM PUF fuzzy extractor and its boot-time enrollment state machine.
 *
 * Enrollment runs over several real power-ons while the device keeps working
 * normally: the first power-up pattern becomes the reference, and every bit
 * that differs in a later power-up is marked unstable. After
 * PUF_ENROLL_SAMPLES samples, the first PUF_STABLE_BITS stable bits are split
 * into PUF_CODEWORDS BCH(127,k,PUF_BCH_T) code-offset blocks. The helper
 * record (mask, code offsets, salt, check value) is public; the root key is
 * HKDF-SHA256 over the reconstructed responses.
 *
 * A fresh helper is provisional: it must reproduce the key on
 * PUF_VERIFY_BOOTS further power-ons before it is committed and the key is
 * handed out. A provisional helper that fails is discarded and enrollment
 * starts over, which is safe because nothing has been bound to it yet.
 *
 * Pure C over caller buffers; persistence is reported back as actions.
 */

#ifndef _PUF_FE_H_
#define _PUF_FE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bch.h"

#define PUF_SAMPLE_BYTES   1024
#define PUF_SAMPLE_BITS    (PUF_SAMPLE_BYTES * 8)
#define PUF_BCH_T          13
#define PUF_CODEWORDS      32
#define PUF_STABLE_BITS    (PUF_CODEWORDS * BCH_N)
#define PUF_ENROLL_SAMPLES 5
#define PUF_VERIFY_BOOTS   2
#define PUF_KEY_SIZE       32
#define PUF_SALT_SIZE      32
#define PUF_CHECK_SIZE     16

// A power-up that differs from the reference in fewer bits than this kept
// its previous contents (a short power gap) and carries no new information.
#define PUF_FRESH_MIN_BITS 16

#define PUF_SCRATCH_MAGIC  0x53465550u // "PUFS"
#define PUF_HELPER_MAGIC   0x48465550u // "PUFH"
#define PUF_HELPER_VERSION 1

enum {
    PUF_STATE_PROVISIONAL = 0,
    PUF_STATE_COMMITTED   = 1,
};

// Enrollment accumulator. Holds raw power-up data: it must never be kept
// longer than enrollment needs it.
typedef struct {
    uint32_t magic;
    uint32_t count; // samples merged, including the reference
    uint8_t ref[PUF_SAMPLE_BYTES];
    uint8_t unstable[PUF_SAMPLE_BYTES];
} puf_scratch_t;

typedef struct {
    uint32_t magic;
    uint8_t version;
    uint8_t state;
    uint8_t verified;
    uint8_t t;
    uint8_t salt[PUF_SALT_SIZE];
    uint8_t mask[PUF_SAMPLE_BYTES]; // exactly PUF_STABLE_BITS bits set
    uint8_t offset[PUF_CODEWORDS][BCH_WORD_BYTES];
    uint8_t check[PUF_CHECK_SIZE];
} puf_helper_t;

typedef void (*puf_rng_t)(uint8_t *buf, size_t len);

typedef enum {
    PUF_BOOT_NO_KEY = 0, // not enrolled or not yet committed: no key this boot
    PUF_BOOT_KEY,        // committed helper reproduced the key
    PUF_BOOT_FAILED,     // committed (or unreadable) helper did not reproduce the key
} puf_boot_result_t;

typedef struct {
    bool scratch_write;  // persist *scratch
    bool scratch_erase;  // destroy the persisted scratch
    bool helper_write;   // persist *helper
    bool helper_delete;  // remove the persisted helper
} puf_actions_t;

/* Build a provisional helper from an accumulated scratch. Returns 0, or -1
 * when there are fewer than PUF_STABLE_BITS stable bits. */
int puf_enroll(const puf_scratch_t *scratch, puf_rng_t rng, puf_helper_t *helper, uint8_t key[PUF_KEY_SIZE]);

/* Reproduce the key from a power-up sample. Returns 0, or -1 when the helper
 * is malformed, a block is not decodable or the check value does not match. */
int puf_reconstruct(const puf_helper_t *helper, const uint8_t sample[PUF_SAMPLE_BYTES], uint8_t key[PUF_KEY_SIZE]);

/* One boot. sample is this boot's power-up pattern; power_on is true only
 * for a real power-on reset. scratch and helper are RAM copies of the
 * persisted records (contents ignored unless their magic matches);
 * helper_present says whether a helper record exists at all. Updates the
 * copies in place and reports what must be persisted in *actions. */
puf_boot_result_t puf_boot_step(const uint8_t sample[PUF_SAMPLE_BYTES], bool power_on,
                                puf_scratch_t *scratch, puf_helper_t *helper, bool helper_present,
                                puf_rng_t rng, uint8_t key[PUF_KEY_SIZE], puf_actions_t *actions);

#endif // _PUF_FE_H_
