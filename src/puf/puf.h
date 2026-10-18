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
 * SRAM PUF root secret (RP2040 only; every other platform keeps puf_key NULL).
 *
 * puf_init() runs once at boot, after the flash scan and before USB and
 * core1 start. It samples the power-up contents of the lower half of the
 * SCRATCH_X bank, advances the enrollment state machine (src/puf/puf_fe.h)
 * and, once a committed helper reproduces the key, publishes it in puf_key.
 * When a committed helper exists but the key cannot be reproduced,
 * puf_status is PUF_STATUS_FAILED and the device must not run: data bound to
 * the key would be unreadable or, worse, silently replaced.
 */

#ifndef _PUF_H_
#define _PUF_H_

#include <stdint.h>

typedef enum {
    PUF_STATUS_OFF = 0,   // platform without a PUF, or not sampled
    PUF_STATUS_ENROLLING, // enrolling or verifying: no key yet
    PUF_STATUS_READY,     // puf_key is valid
    PUF_STATUS_FAILED,    // committed key not reproduced this boot
} puf_status_t;

#define PUF_ROOT_KEY_SIZE 32

extern const uint8_t *puf_key;
extern puf_status_t puf_status;

void puf_init(void);

/* Never returns when puf_status is PUF_STATUS_FAILED: blinks the LED red
 * until the next power cycle. Call after led_init(). */
void puf_halt_if_failed(void);

#endif // _PUF_H_
