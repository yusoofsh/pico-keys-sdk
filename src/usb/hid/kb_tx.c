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

/* Single-owner keyboard transmitter: pure state machine, see kb_tx.h. */

#include <string.h>

#include "kb_tx.h"

static kb_tx_ops_t ops;
static uint8_t owner = KB_TX_OWNER_NONE;

static uint8_t buffer[KB_TX_BUFFER_MAX];
static uint16_t buf_len = 0;
static uint16_t buf_pos = 0;
static bool buf_encode = false;
/* A key-down report was accepted by the transport and its all-released
 * report has not been sent yet. */
static bool key_down = false;

static bool typing(void) {
    return key_down || buf_len > 0;
}

void kb_tx_init(const kb_tx_ops_t *init_ops) {
    if (init_ops != NULL) {
        ops = *init_ops;
    }
    else {
        memset(&ops, 0, sizeof(ops));
    }
    owner = KB_TX_OWNER_NONE;
    buf_len = 0;
    buf_pos = 0;
    buf_encode = false;
    key_down = false;
}

bool kb_tx_busy(void) {
    return owner != KB_TX_OWNER_NONE || typing();
}

bool kb_tx_typing(void) {
    return typing();
}

uint8_t kb_tx_owner(void) {
    return owner;
}

bool kb_tx_claim(uint8_t new_owner) {
    if (new_owner == KB_TX_OWNER_NONE || kb_tx_busy()) {
        return false;
    }
    owner = new_owner;
    return true;
}

bool kb_tx_release(uint8_t rel_owner) {
    if (rel_owner == KB_TX_OWNER_NONE || owner != rel_owner) {
        return false;
    }
    owner = KB_TX_OWNER_NONE;
    return true;
}

bool kb_tx_add_buffer(uint8_t add_owner, const uint8_t *data, size_t len, bool encode) {
    if (add_owner == KB_TX_OWNER_NONE || owner != add_owner) {
        return false;
    }
    if (typing()) {
        /* Never overwrite text that is still being sent. */
        return false;
    }
    if (len > KB_TX_BUFFER_MAX) {
        len = KB_TX_BUFFER_MAX;
    }
    if (len > 0) {
        memcpy(buffer, data, len);
    }
    buf_len = (uint16_t)len;
    buf_pos = 0;
    buf_encode = encode;
    key_down = false;
    return true;
}

bool kb_tx_append_buffer(uint8_t app_owner, const uint8_t *data, size_t len) {
    if (app_owner == KB_TX_OWNER_NONE || owner != app_owner) {
        return false;
    }
    if (len > KB_TX_BUFFER_MAX - buf_len) {
        return false;
    }
    if (len > 0) {
        memcpy(buffer + buf_len, data, len);
    }
    buf_len = (uint16_t)(buf_len + len);
    return true;
}

void kb_tx_task(void) {
    if (ops.ready == NULL || ops.send == NULL) {
        return;
    }
    for (;;) {
        if (key_down) {
            static const uint8_t released[6] = { 0 };
            if (!ops.ready()) {
                return;
            }
            if (!ops.send(0, released)) {
                return;
            }
            key_down = false;
        }
        else if (buf_pos < buf_len) {
            if (!ops.ready()) {
                return;
            }
            uint8_t chr = buffer[buf_pos];
            uint8_t modifier = 0;
            uint8_t keycode = 0;
            if (buf_encode && ops.lookup != NULL) {
                ops.lookup(chr, &modifier, &keycode);
            }
            else {
                /* Raw keycodes: bit 7 selects left shift (legacy convention). */
                if (chr & 0x80) {
                    modifier = KB_TX_MOD_LEFTSHIFT;
                }
                keycode = chr & 0x7f;
            }
            uint8_t keycodes[6] = { 0 };
            keycodes[0] = keycode;
            if (!ops.send(modifier, keycodes)) {
                return;
            }
            buf_pos++;
            key_down = true;
        }
        else {
            /* Fully drained: recycle the storage. */
            if (buf_len > 0) {
                buf_len = 0;
                buf_pos = 0;
                buf_encode = false;
            }
            return;
        }
    }
}
