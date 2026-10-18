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

// SRAM PUF fuzzy extractor and boot state machine against simulated chips.
// A chip is a per-cell probability of powering up as 1; most cells are
// strongly skewed and a minority are noisy, giving the ~5% bit error rate
// between two power-ups measured on RP2040 parts. One scenario per process.

#include "puf/puf_fe.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static uint64_t rng_state = 0x243F6A8885A308D3ULL;

static uint64_t next64(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}

static double uniform(void) {
    return (double)(next64() >> 11) / 9007199254740992.0;
}

static void test_rng(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(next64() >> 32);
    }
}

typedef struct {
    float p1[PUF_SAMPLE_BITS];
} chip_t;

static void chip_make(chip_t *chip) {
    for (unsigned i = 0; i < PUF_SAMPLE_BITS; i++) {
        double u = uniform(), p;
        if (u < 0.80) {
            p = 0.0005;
        }
        else if (u < 0.92) {
            p = 0.02 + 0.08 * uniform();
        }
        else {
            p = 0.2 + 0.3 * uniform();
        }
        chip->p1[i] = (float)((next64() & 1) ? p : 1.0 - p);
    }
}

static void power_up(const chip_t *chip, uint8_t sample[PUF_SAMPLE_BYTES]) {
    memset(sample, 0, PUF_SAMPLE_BYTES);
    for (unsigned i = 0; i < PUF_SAMPLE_BITS; i++) {
        bch_set_bit(sample, i, uniform() < chip->p1[i]);
    }
}

// Persisted records as the firmware would keep them.
typedef struct {
    puf_scratch_t scratch;
    puf_helper_t helper;
    bool helper_present;
} store_t;

static void store_init(store_t *s) {
    memset(&s->scratch, 0xFF, sizeof(s->scratch));
    memset(&s->helper, 0, sizeof(s->helper));
    s->helper_present = false;
}

static puf_boot_result_t boot(store_t *s, const uint8_t sample[PUF_SAMPLE_BYTES], bool power_on,
                              uint8_t key[PUF_KEY_SIZE], puf_actions_t *act) {
    puf_scratch_t scratch = s->scratch;
    puf_helper_t helper = s->helper;
    puf_boot_result_t r = puf_boot_step(sample, power_on, &scratch, &helper, s->helper_present, test_rng, key, act);
    CHECK(!(act->scratch_write && act->scratch_erase));
    CHECK(!(act->helper_write && act->helper_delete));
    if (act->scratch_erase) {
        memset(&s->scratch, 0xFF, sizeof(s->scratch));
    }
    if (act->scratch_write) {
        s->scratch = scratch;
    }
    if (act->helper_delete) {
        memset(&s->helper, 0, sizeof(s->helper));
        s->helper_present = false;
    }
    if (act->helper_write) {
        s->helper = helper;
        s->helper_present = true;
    }
    return r;
}

static puf_boot_result_t boot_chip(store_t *s, const chip_t *chip, bool power_on, uint8_t key[PUF_KEY_SIZE], puf_actions_t *act) {
    uint8_t sample[PUF_SAMPLE_BYTES];
    power_up(chip, sample);
    return boot(s, sample, power_on, key, act);
}

static bool all_ff(const void *p, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (((const uint8_t *)p)[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

// Run a fresh chip through enrollment and verification; return the key.
static void enroll_and_commit(store_t *s, const chip_t *chip, uint8_t key[PUF_KEY_SIZE]) {
    puf_actions_t act;
    for (unsigned i = 0; i < PUF_ENROLL_SAMPLES; i++) {
        CHECK(boot_chip(s, chip, true, key, &act) == PUF_BOOT_NO_KEY);
    }
    CHECK(s->helper_present && s->helper.state == PUF_STATE_PROVISIONAL);
    CHECK(all_ff(&s->scratch, sizeof(s->scratch)));
    for (unsigned i = 1; i < PUF_VERIFY_BOOTS; i++) {
        CHECK(boot_chip(s, chip, true, key, &act) == PUF_BOOT_NO_KEY);
        CHECK(s->helper.state == PUF_STATE_PROVISIONAL);
    }
    CHECK(boot_chip(s, chip, true, key, &act) == PUF_BOOT_KEY);
    CHECK(s->helper.state == PUF_STATE_COMMITTED);
}

static void scene_enroll_commit(void) {
    chip_t chip;
    chip_make(&chip);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE], k2[PUF_KEY_SIZE];
    enroll_and_commit(&s, &chip, key);
    // The committed key is stable over many later power-ons and warm boots.
    unsigned failures = 0;
    puf_actions_t act;
    for (unsigned i = 0; i < 3000; i++) {
        puf_boot_result_t r = boot_chip(&s, &chip, (i % 3) != 0, k2, &act);
        CHECK(!act.helper_write && !act.helper_delete && !act.scratch_write && !act.scratch_erase);
        if (r == PUF_BOOT_KEY) {
            CHECK(memcmp(key, k2, sizeof(key)) == 0);
        }
        else {
            CHECK(r == PUF_BOOT_FAILED);
            failures++;
        }
    }
    printf("committed reconstruction failures: %u / 3000\n", failures);
    CHECK(failures <= 3);
}

static void scene_warm_boots_ignored(void) {
    chip_t chip;
    chip_make(&chip);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE];
    puf_actions_t act;
    for (unsigned i = 0; i < 20; i++) {
        CHECK(boot_chip(&s, &chip, false, key, &act) == PUF_BOOT_NO_KEY);
        CHECK(!act.scratch_write && !act.scratch_erase && !act.helper_write && !act.helper_delete);
    }
    CHECK(boot_chip(&s, &chip, true, key, &act) == PUF_BOOT_NO_KEY && act.scratch_write);
    CHECK(s.scratch.count == 1);
    for (unsigned i = 0; i < 20; i++) {
        CHECK(boot_chip(&s, &chip, false, key, &act) == PUF_BOOT_NO_KEY);
        CHECK(!act.scratch_write);
    }
    CHECK(s.scratch.count == 1);
}

static void scene_retained_sample_ignored(void) {
    chip_t chip;
    chip_make(&chip);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE], sample[PUF_SAMPLE_BYTES];
    puf_actions_t act;
    power_up(&chip, sample);
    CHECK(boot(&s, sample, true, key, &act) == PUF_BOOT_NO_KEY);
    // The same contents again (SRAM kept its data over a short power gap).
    for (unsigned i = 0; i < 10; i++) {
        CHECK(boot(&s, sample, true, key, &act) == PUF_BOOT_NO_KEY);
        CHECK(!act.scratch_write);
    }
    CHECK(s.scratch.count == 1);
}

static void scene_verify_failure_restarts(void) {
    chip_t chip, other;
    chip_make(&chip);
    chip_make(&other);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE];
    puf_actions_t act;
    for (unsigned i = 0; i < PUF_ENROLL_SAMPLES; i++) {
        boot_chip(&s, &chip, true, key, &act);
    }
    CHECK(s.helper_present && s.helper.state == PUF_STATE_PROVISIONAL);
    // A warm boot with garbage never counts or discards.
    CHECK(boot_chip(&s, &other, false, key, &act) == PUF_BOOT_NO_KEY);
    CHECK(s.helper_present && s.helper.verified == 0);
    // A power-on that does not reproduce the key discards the helper and
    // restarts enrollment from this boot.
    CHECK(boot_chip(&s, &other, true, key, &act) == PUF_BOOT_NO_KEY);
    CHECK(act.helper_delete && act.scratch_write);
    CHECK(!s.helper_present && s.scratch.magic == PUF_SCRATCH_MAGIC && s.scratch.count == 1);
}

static void scene_committed_failure(void) {
    chip_t chip, other;
    chip_make(&chip);
    chip_make(&other);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE], k2[PUF_KEY_SIZE];
    enroll_and_commit(&s, &chip, key);
    puf_actions_t act;
    CHECK(boot_chip(&s, &other, true, k2, &act) == PUF_BOOT_FAILED);
    CHECK(!act.helper_delete && !act.helper_write && !act.scratch_write);
    CHECK(s.helper_present && s.helper.state == PUF_STATE_COMMITTED);
    // A malformed helper fails closed and is never replaced.
    store_t bad = s;
    bad.helper.magic ^= 1;
    CHECK(boot_chip(&bad, &chip, true, k2, &act) == PUF_BOOT_FAILED);
    CHECK(!act.helper_delete && !act.helper_write && !act.scratch_write);
    bad = s;
    bch_set_bit(bad.helper.mask, 0, !bch_get_bit(bad.helper.mask, 0));
    CHECK(boot_chip(&bad, &chip, true, k2, &act) == PUF_BOOT_FAILED);
    // A wrong check value is caught even when every block decodes.
    bad = s;
    bad.helper.check[0] ^= 0x80;
    CHECK(boot_chip(&bad, &chip, true, k2, &act) == PUF_BOOT_FAILED);
}

static void scene_distinct_chips(void) {
    chip_t a, b;
    chip_make(&a);
    chip_make(&b);
    store_t sa, sb;
    store_init(&sa);
    store_init(&sb);
    uint8_t ka[PUF_KEY_SIZE], kb[PUF_KEY_SIZE];
    enroll_and_commit(&sa, &a, ka);
    enroll_and_commit(&sb, &b, kb);
    CHECK(memcmp(ka, kb, sizeof(ka)) != 0);
    // Re-enrolling the same chip gives a different key (new salt and offsets).
    store_t s2;
    store_init(&s2);
    uint8_t k2[PUF_KEY_SIZE];
    enroll_and_commit(&s2, &a, k2);
    CHECK(memcmp(ka, k2, sizeof(ka)) != 0);
}

static void scene_leftover_scratch_erased(void) {
    chip_t chip;
    chip_make(&chip);
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE];
    enroll_and_commit(&s, &chip, key);
    // Power lost between writing the helper and erasing the scratch.
    s.scratch.magic = PUF_SCRATCH_MAGIC;
    s.scratch.count = 3;
    puf_actions_t act;
    CHECK(boot_chip(&s, &chip, false, key, &act) == PUF_BOOT_KEY);
    CHECK(act.scratch_erase);
    CHECK(all_ff(&s.scratch, sizeof(s.scratch)));
}

static void scene_too_few_stable_bits(void) {
    // Every cell noisy: enrollment cannot find enough stable bits and starts over.
    chip_t chip;
    for (unsigned i = 0; i < PUF_SAMPLE_BITS; i++) {
        chip.p1[i] = 0.5f;
    }
    store_t s;
    store_init(&s);
    uint8_t key[PUF_KEY_SIZE];
    puf_actions_t act;
    for (unsigned i = 0; i < PUF_ENROLL_SAMPLES; i++) {
        CHECK(boot_chip(&s, &chip, true, key, &act) == PUF_BOOT_NO_KEY);
    }
    CHECK(!s.helper_present);
    CHECK(s.scratch.magic == PUF_SCRATCH_MAGIC && s.scratch.count == 1);
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s <scene>\n", argv[0]);
        return 2;
    }
    static const struct { const char *name; void (*fn)(void); } scenes[] = {
        { "enroll_commit", scene_enroll_commit },
        { "warm_boots_ignored", scene_warm_boots_ignored },
        { "retained_sample_ignored", scene_retained_sample_ignored },
        { "verify_failure_restarts", scene_verify_failure_restarts },
        { "committed_failure", scene_committed_failure },
        { "distinct_chips", scene_distinct_chips },
        { "leftover_scratch_erased", scene_leftover_scratch_erased },
        { "too_few_stable_bits", scene_too_few_stable_bits },
    };
    for (size_t i = 0; i < sizeof(scenes) / sizeof(scenes[0]); i++) {
        if (strcmp(argv[1], scenes[i].name) == 0) {
            scenes[i].fn();
            printf("puf_fe_test %s: OK\n", argv[1]);
            return 0;
        }
    }
    fprintf(stderr, "unknown scene %s\n", argv[1]);
    return 2;
}
