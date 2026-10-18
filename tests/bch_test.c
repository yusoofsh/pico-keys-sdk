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

// BCH(127,k,t) codec: code dimensions against the standard table, clean
// round trips, exact correction of up to t random errors, and safe failure
// (no crash, no silent "fix" back to the original) beyond t.

#include "puf/bch.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static uint64_t rng_state = 0x9E3779B97F4A7C15ULL;

static uint32_t rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (uint32_t)(rng_state >> 16);
}

static void random_msg(const bch_code_t *c, uint8_t msg[BCH_WORD_BYTES]) {
    memset(msg, 0, BCH_WORD_BYTES);
    for (unsigned i = 0; i < c->k; i++) {
        bch_set_bit(msg, i, (int)(rnd() & 1));
    }
}

static void flip_distinct(uint8_t w[BCH_WORD_BYTES], unsigned count) {
    uint8_t used[BCH_N] = { 0 };
    for (unsigned e = 0; e < count;) {
        unsigned p = rnd() % BCH_N;
        if (used[p]) {
            continue;
        }
        used[p] = 1;
        bch_set_bit(w, p, !bch_get_bit(w, p));
        e++;
    }
}

static void test_dimensions(void) {
    static const struct { unsigned t, k; } table[] = {
        { 1, 120 }, { 2, 113 }, { 5, 92 }, { 10, 64 }, { 11, 57 }, { 13, 50 }, { 14, 43 }, { 15, 36 }, { 21, 29 }, { 31, 8 },
    };
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        bch_code_t c;
        CHECK(bch_init(&c, table[i].t) == 0);
        CHECK(c.k == table[i].k);
        CHECK(c.gen[0] == 1 && c.gen[BCH_N - c.k] == 1);
    }
    bch_code_t c;
    CHECK(bch_init(&c, 0) == -1);
    CHECK(bch_init(&c, 32) == -1);
}

static void test_correction(unsigned t) {
    bch_code_t c;
    CHECK(bch_init(&c, t) == 0);
    uint8_t msg[BCH_WORD_BYTES], cw[BCH_WORD_BYTES], w[BCH_WORD_BYTES];
    for (unsigned trial = 0; trial < 300; trial++) {
        random_msg(&c, msg);
        bch_encode(&c, msg, cw);
        CHECK(bch_get_bit(cw, 127) == 0);
        for (unsigned i = 0; i < c.k; i++) {
            CHECK(bch_get_bit(cw, BCH_N - c.k + i) == bch_get_bit(msg, i));
        }
        memcpy(w, cw, sizeof(w));
        CHECK(bch_decode(&c, w) == 0);
        CHECK(memcmp(w, cw, sizeof(w)) == 0);
        for (unsigned e = 1; e <= t; e++) {
            memcpy(w, cw, sizeof(w));
            flip_distinct(w, e);
            CHECK(bch_decode(&c, w) == (int)e);
            CHECK(memcmp(w, cw, sizeof(w)) == 0);
        }
        for (unsigned e = t + 1; e <= t + 6; e++) {
            memcpy(w, cw, sizeof(w));
            flip_distinct(w, e);
            uint8_t before[BCH_WORD_BYTES];
            memcpy(before, w, sizeof(w));
            int r = bch_decode(&c, w);
            if (r < 0) {
                CHECK(memcmp(w, before, sizeof(w)) == 0);
            }
            else {
                // A miscorrection lands on another codeword, never the original.
                CHECK(r <= (int)t);
                CHECK(memcmp(w, cw, sizeof(w)) != 0);
                uint8_t again[BCH_WORD_BYTES];
                memcpy(again, w, sizeof(w));
                CHECK(bch_decode(&c, again) == 0);
            }
        }
    }
}

int main(void) {
    test_dimensions();
    test_correction(13);
    test_correction(10);
    test_correction(1);
    test_correction(31);
    printf("bch_test: OK\n");
    return 0;
}
