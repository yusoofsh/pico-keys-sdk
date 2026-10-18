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
 * Binary BCH codes of length 127 over GF(2^7), t = 1..31.
 *
 * A word is 16 bytes: bit i (the coefficient of x^i) lives in byte i / 8 at
 * mask 0x80 >> (i % 8). Bit 127 is unused and always 0. Encoding is
 * systematic: the k message bits occupy positions n-k..n-1 and the parity
 * bits positions 0..n-k-1. Pure C, no platform dependencies.
 */

#ifndef _BCH_H_
#define _BCH_H_

#include <stdint.h>

#define BCH_N          127
#define BCH_WORD_BYTES 16
#define BCH_MAX_T      31

typedef struct {
    uint8_t t;
    uint8_t k;
    uint8_t gen[BCH_N + 1]; // generator polynomial over GF(2), degree n-k
} bch_code_t;

/* Build the code for t errors. Returns 0, or -1 when t is out of range. */
int bch_init(bch_code_t *code, unsigned t);

/* Encode the message bits 0..k-1 of msg into a codeword. */
void bch_encode(const bch_code_t *code, const uint8_t msg[BCH_WORD_BYTES], uint8_t cw[BCH_WORD_BYTES]);

/* Correct word in place. Returns the number of corrected bits (0..t), or -1
 * when the error pattern is not decodable (word is then left untouched). A
 * pattern of more than t errors may also decode to a different codeword. */
int bch_decode(const bch_code_t *code, uint8_t word[BCH_WORD_BYTES]);

static inline int bch_get_bit(const uint8_t *w, unsigned i) {
    return (w[i >> 3] >> (7 - (i & 7))) & 1;
}

static inline void bch_set_bit(uint8_t *w, unsigned i, int v) {
    uint8_t m = (uint8_t)(0x80 >> (i & 7));
    w[i >> 3] = v ? (uint8_t)(w[i >> 3] | m) : (uint8_t)(w[i >> 3] & ~m);
}

#endif // _BCH_H_
