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

#include "bch.h"

#include <string.h>

#define GF_M    7
#define GF_POLY 0x89 // x^7 + x^3 + 1, primitive

static uint8_t gf_exp[2 * BCH_N];
static uint8_t gf_log[BCH_N + 1];
static int gf_ready = 0;

static void gf_init(void) {
    if (gf_ready) {
        return;
    }
    unsigned x = 1;
    for (unsigned i = 0; i < BCH_N; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_exp[i + BCH_N] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & (1u << GF_M)) {
            x ^= GF_POLY;
        }
    }
    gf_log[0] = 0; // never used: callers test for zero first
    gf_ready = 1;
}

static uint8_t gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) {
        return 0;
    }
    return gf_exp[gf_log[a] + gf_log[b]];
}

static uint8_t gf_div(uint8_t a, uint8_t b) {
    if (a == 0) {
        return 0;
    }
    return gf_exp[(gf_log[a] + BCH_N - gf_log[b]) % BCH_N];
}

int bch_init(bch_code_t *code, unsigned t) {
    if (code == NULL || t < 1 || t > BCH_MAX_T) {
        return -1;
    }
    gf_init();
    uint8_t done[BCH_N] = { 0 };
    uint8_t g[BCH_N + 1] = { 1 };
    unsigned deg = 0;
    for (unsigned i = 1; i <= 2 * t; i++) {
        if (done[i % BCH_N]) {
            continue;
        }
        // Minimal polynomial of alpha^i: product of (x + alpha^e) over the
        // cyclotomic coset of i. Its coefficients land in GF(2).
        uint8_t mp[GF_M + 1] = { 1 };
        unsigned mdeg = 0;
        unsigned e = i % BCH_N;
        do {
            done[e] = 1;
            uint8_t root = gf_exp[e];
            for (unsigned j = mdeg + 1; j > 0; j--) {
                mp[j] = (uint8_t)(mp[j - 1] ^ gf_mul(mp[j], root));
            }
            mp[0] = gf_mul(mp[0], root);
            mdeg++;
            e = (e * 2) % BCH_N;
        } while (e != i % BCH_N);
        uint8_t prod[BCH_N + 1] = { 0 };
        for (unsigned a = 0; a <= deg; a++) {
            if (!g[a]) {
                continue;
            }
            for (unsigned b = 0; b <= mdeg; b++) {
                prod[a + b] ^= mp[b];
            }
        }
        deg += mdeg;
        memcpy(g, prod, sizeof(g));
    }
    if (deg >= BCH_N) {
        return -1;
    }
    memset(code, 0, sizeof(*code));
    code->t = (uint8_t)t;
    code->k = (uint8_t)(BCH_N - deg);
    memcpy(code->gen, g, sizeof(g));
    return 0;
}

void bch_encode(const bch_code_t *code, const uint8_t msg[BCH_WORD_BYTES], uint8_t cw[BCH_WORD_BYTES]) {
    const unsigned r = BCH_N - code->k;
    uint8_t rem[BCH_N] = { 0 };
    // LFSR division of m(x) * x^r by g(x), highest message bit first.
    for (unsigned i = code->k; i-- > 0;) {
        uint8_t fb = (uint8_t)(bch_get_bit(msg, i) ^ rem[r - 1]);
        for (unsigned j = r - 1; j > 0; j--) {
            rem[j] = (uint8_t)(rem[j - 1] ^ (fb & code->gen[j]));
        }
        rem[0] = (uint8_t)(fb & code->gen[0]);
    }
    memset(cw, 0, BCH_WORD_BYTES);
    for (unsigned j = 0; j < r; j++) {
        bch_set_bit(cw, j, rem[j]);
    }
    for (unsigned i = 0; i < code->k; i++) {
        bch_set_bit(cw, r + i, bch_get_bit(msg, i));
    }
}

int bch_decode(const bch_code_t *code, uint8_t word[BCH_WORD_BYTES]) {
    const unsigned t = code->t;
    uint8_t s[2 * BCH_MAX_T + 1] = { 0 };
    int any = 0;
    for (unsigned j = 1; j <= 2 * t; j++) {
        uint8_t acc = 0;
        for (unsigned i = 0; i < BCH_N; i++) {
            if (bch_get_bit(word, i)) {
                acc ^= gf_exp[(i * j) % BCH_N];
            }
        }
        s[j] = acc;
        any |= acc;
    }
    if (!any) {
        return 0;
    }

    // Berlekamp-Massey: error locator C(x) of degree L.
    uint8_t C[2 * BCH_MAX_T + 2] = { 1 }, B[2 * BCH_MAX_T + 2] = { 1 }, T[2 * BCH_MAX_T + 2];
    unsigned L = 0, m = 1;
    uint8_t b = 1;
    for (unsigned n = 0; n < 2 * t; n++) {
        uint8_t d = s[n + 1];
        for (unsigned i = 1; i <= L; i++) {
            d ^= gf_mul(C[i], s[n + 1 - i]);
        }
        if (d == 0) {
            m++;
            continue;
        }
        uint8_t coef = gf_div(d, b);
        memcpy(T, C, sizeof(T));
        for (unsigned i = 0; i + m < sizeof(C); i++) {
            C[i + m] ^= gf_mul(coef, B[i]);
        }
        if (2 * L <= n) {
            L = n + 1 - L;
            memcpy(B, T, sizeof(B));
            b = d;
            m = 1;
        }
        else {
            m++;
        }
    }
    if (L > t) {
        return -1;
    }

    // Chien search: an error at position i makes C(alpha^-i) zero.
    unsigned pos[BCH_MAX_T];
    unsigned found = 0;
    for (unsigned i = 0; i < BCH_N; i++) {
        uint8_t v = 0;
        unsigned inv = (BCH_N - i) % BCH_N;
        for (unsigned j = 0; j <= L; j++) {
            if (C[j]) {
                v ^= gf_exp[(gf_log[C[j]] + inv * j) % BCH_N];
            }
        }
        if (v == 0) {
            if (found == L) {
                return -1;
            }
            pos[found++] = i;
        }
    }
    if (found != L) {
        return -1;
    }
    for (unsigned e = 0; e < found; e++) {
        bch_set_bit(word, pos[e], !bch_get_bit(word, pos[e]));
    }
    return (int)found;
}
