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

#include "puf_fe.h"

#include <string.h>

#include "mbedtls/constant_time.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/md.h"
#include "mbedtls/platform_util.h"

static const uint8_t root_info[] = "PIPICO/PUF/ROOT/v1";
static const uint8_t check_info[] = "PIPICO/PUF/CHECK/v1";

static int bit_count(const uint8_t *buf, size_t len) {
    int n = 0;
    for (size_t i = 0; i < len; i++) {
        for (uint8_t b = buf[i]; b; b &= (uint8_t)(b - 1)) {
            n++;
        }
    }
    return n;
}

// Collect the masked bits of sample, in position order, into PUF_CODEWORDS
// words of BCH_N bits.
static void gather(const uint8_t mask[PUF_SAMPLE_BYTES], const uint8_t sample[PUF_SAMPLE_BYTES],
                   uint8_t words[PUF_CODEWORDS][BCH_WORD_BYTES]) {
    memset(words, 0, PUF_CODEWORDS * BCH_WORD_BYTES);
    unsigned out = 0;
    for (unsigned p = 0; p < PUF_SAMPLE_BITS && out < PUF_STABLE_BITS; p++) {
        if (bch_get_bit(mask, p)) {
            bch_set_bit(words[out / BCH_N], out % BCH_N, bch_get_bit(sample, p));
            out++;
        }
    }
}

static int derive(const uint8_t salt[PUF_SALT_SIZE], const uint8_t words[PUF_CODEWORDS][BCH_WORD_BYTES],
                  uint8_t key[PUF_KEY_SIZE], uint8_t check[PUF_CHECK_SIZE]) {
    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (mbedtls_hkdf(md, salt, PUF_SALT_SIZE, (const uint8_t *)words, PUF_CODEWORDS * BCH_WORD_BYTES,
                     root_info, sizeof(root_info) - 1, key, PUF_KEY_SIZE) != 0) {
        return -1;
    }
    uint8_t mac[32];
    if (mbedtls_md_hmac(md, key, PUF_KEY_SIZE, check_info, sizeof(check_info) - 1, mac) != 0) {
        mbedtls_platform_zeroize(key, PUF_KEY_SIZE);
        return -1;
    }
    memcpy(check, mac, PUF_CHECK_SIZE);
    mbedtls_platform_zeroize(mac, sizeof(mac));
    return 0;
}

int puf_enroll(const puf_scratch_t *scratch, puf_rng_t rng, puf_helper_t *helper, uint8_t key[PUF_KEY_SIZE]) {
    bch_code_t code;
    if (bch_init(&code, PUF_BCH_T) != 0) {
        return -1;
    }
    memset(helper, 0, sizeof(*helper));
    unsigned taken = 0;
    for (unsigned p = 0; p < PUF_SAMPLE_BITS && taken < PUF_STABLE_BITS; p++) {
        if (!bch_get_bit(scratch->unstable, p)) {
            bch_set_bit(helper->mask, p, 1);
            taken++;
        }
    }
    if (taken < PUF_STABLE_BITS) {
        return -1;
    }
    uint8_t words[PUF_CODEWORDS][BCH_WORD_BYTES];
    gather(helper->mask, scratch->ref, words);
    for (unsigned i = 0; i < PUF_CODEWORDS; i++) {
        uint8_t msg[BCH_WORD_BYTES], cw[BCH_WORD_BYTES];
        rng(msg, sizeof(msg));
        bch_encode(&code, msg, cw);
        for (unsigned b = 0; b < BCH_WORD_BYTES; b++) {
            helper->offset[i][b] = (uint8_t)(cw[b] ^ words[i][b]);
        }
        mbedtls_platform_zeroize(msg, sizeof(msg));
        mbedtls_platform_zeroize(cw, sizeof(cw));
    }
    rng(helper->salt, sizeof(helper->salt));
    int ret = derive(helper->salt, (const uint8_t (*)[BCH_WORD_BYTES])words, key, helper->check);
    mbedtls_platform_zeroize(words, sizeof(words));
    if (ret != 0) {
        return -1;
    }
    helper->magic = PUF_HELPER_MAGIC;
    helper->version = PUF_HELPER_VERSION;
    helper->state = PUF_STATE_PROVISIONAL;
    helper->verified = 0;
    helper->t = PUF_BCH_T;
    return 0;
}

static bool helper_well_formed(const puf_helper_t *helper) {
    return helper->magic == PUF_HELPER_MAGIC && helper->version == PUF_HELPER_VERSION &&
           helper->t == PUF_BCH_T &&
           (helper->state == PUF_STATE_PROVISIONAL || helper->state == PUF_STATE_COMMITTED) &&
           bit_count(helper->mask, sizeof(helper->mask)) == PUF_STABLE_BITS;
}

int puf_reconstruct(const puf_helper_t *helper, const uint8_t sample[PUF_SAMPLE_BYTES], uint8_t key[PUF_KEY_SIZE]) {
    bch_code_t code;
    if (!helper_well_formed(helper) || bch_init(&code, PUF_BCH_T) != 0) {
        return -1;
    }
    uint8_t words[PUF_CODEWORDS][BCH_WORD_BYTES];
    gather(helper->mask, sample, words);
    int ret = 0;
    for (unsigned i = 0; i < PUF_CODEWORDS && ret == 0; i++) {
        uint8_t w[BCH_WORD_BYTES];
        for (unsigned b = 0; b < BCH_WORD_BYTES; b++) {
            w[b] = (uint8_t)(helper->offset[i][b] ^ words[i][b]);
        }
        if (bch_decode(&code, w) < 0) {
            ret = -1;
        }
        for (unsigned b = 0; b < BCH_WORD_BYTES; b++) {
            words[i][b] = (uint8_t)(helper->offset[i][b] ^ w[b]);
        }
        mbedtls_platform_zeroize(w, sizeof(w));
    }
    uint8_t check[PUF_CHECK_SIZE];
    if (ret == 0) {
        ret = derive(helper->salt, (const uint8_t (*)[BCH_WORD_BYTES])words, key, check);
    }
    mbedtls_platform_zeroize(words, sizeof(words));
    if (ret == 0 && mbedtls_ct_memcmp(check, helper->check, PUF_CHECK_SIZE) != 0) {
        mbedtls_platform_zeroize(key, PUF_KEY_SIZE);
        ret = -1;
    }
    return ret;
}

static int hamming(const uint8_t *a, const uint8_t *b, size_t len) {
    int n = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t x = (uint8_t)(a[i] ^ b[i]);
        n += bit_count(&x, 1);
    }
    return n;
}

static bool scratch_blank(const puf_scratch_t *scratch) {
    const uint8_t *p = (const uint8_t *)scratch;
    for (size_t i = 0; i < sizeof(*scratch); i++) {
        if (p[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

static void scratch_start(puf_scratch_t *scratch, const uint8_t sample[PUF_SAMPLE_BYTES], puf_actions_t *actions) {
    scratch->magic = PUF_SCRATCH_MAGIC;
    scratch->count = 1;
    memcpy(scratch->ref, sample, PUF_SAMPLE_BYTES);
    memset(scratch->unstable, 0, PUF_SAMPLE_BYTES);
    actions->scratch_write = true;
}

puf_boot_result_t puf_boot_step(const uint8_t sample[PUF_SAMPLE_BYTES], bool power_on,
                                puf_scratch_t *scratch, puf_helper_t *helper, bool helper_present,
                                puf_rng_t rng, uint8_t key[PUF_KEY_SIZE], puf_actions_t *actions) {
    memset(actions, 0, sizeof(*actions));
    const bool scratch_valid = scratch->magic == PUF_SCRATCH_MAGIC &&
                               scratch->count >= 1 && scratch->count < PUF_ENROLL_SAMPLES;

    if (helper_present) {
        // A finished enrollment leaves no raw samples behind.
        actions->scratch_erase = !scratch_blank(scratch);
        if (!helper_well_formed(helper)) {
            return PUF_BOOT_FAILED;
        }
        const bool ok = puf_reconstruct(helper, sample, key) == 0;
        if (helper->state == PUF_STATE_COMMITTED) {
            return ok ? PUF_BOOT_KEY : PUF_BOOT_FAILED;
        }
        if (power_on && ok) {
            if (++helper->verified >= PUF_VERIFY_BOOTS) {
                helper->state = PUF_STATE_COMMITTED;
            }
            actions->helper_write = true;
            if (helper->state == PUF_STATE_COMMITTED) {
                return PUF_BOOT_KEY;
            }
        }
        else if (power_on) {
            actions->helper_delete = true;
            actions->scratch_erase = false;
            scratch_start(scratch, sample, actions);
        }
        mbedtls_platform_zeroize(key, PUF_KEY_SIZE);
        return PUF_BOOT_NO_KEY;
    }

    if (!power_on) {
        return PUF_BOOT_NO_KEY;
    }
    if (!scratch_valid) {
        scratch_start(scratch, sample, actions);
        return PUF_BOOT_NO_KEY;
    }
    if (hamming(scratch->ref, sample, PUF_SAMPLE_BYTES) < PUF_FRESH_MIN_BITS) {
        return PUF_BOOT_NO_KEY;
    }
    for (unsigned i = 0; i < PUF_SAMPLE_BYTES; i++) {
        scratch->unstable[i] |= (uint8_t)(scratch->ref[i] ^ sample[i]);
    }
    scratch->count++;
    if (scratch->count < PUF_ENROLL_SAMPLES) {
        actions->scratch_write = true;
        return PUF_BOOT_NO_KEY;
    }
    uint8_t provisional[PUF_KEY_SIZE];
    if (puf_enroll(scratch, rng, helper, provisional) != 0) {
        // Too few stable bits: start over with this boot as the reference.
        scratch_start(scratch, sample, actions);
        return PUF_BOOT_NO_KEY;
    }
    mbedtls_platform_zeroize(provisional, sizeof(provisional));
    actions->helper_write = true;
    actions->scratch_erase = true;
    return PUF_BOOT_NO_KEY;
}
