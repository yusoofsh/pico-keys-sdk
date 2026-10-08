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

#ifndef KB_TX_H
#define KB_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Single-owner keyboard transmitter.
 *
 * Pure state machine (no Pico SDK or TinyUSB headers): all producers (OTP
 * typing, the companion) claim the transmitter, queue key reports through
 * kb_tx_add_buffer()/kb_tx_append_buffer() and release it. The thin adapter
 * (src/usb/hid/hid.c) injects the transport ops and pumps kb_tx_task() from
 * the HID task.
 *
 * Guarantees:
 *   - one owner at a time; a claim while busy (claimed or reports in
 *     flight) fails; a release by a non-owner fails;
 *   - kb_tx_busy() is true from the claim until the final release report
 *     has left the transmitter;
 *   - kb_tx_add_buffer() never overwrites text still being sent: it is
 *     refused while reports are in flight, so queued text is typed in full
 *     and in order;
 *   - every key-down report that was sent is followed by an all-keys-
 *     released report, including when the busy/owner state changes
 *     mid-press or the endpoint reports not-ready for a number of polls.
 */

/* Owner ids. 0 is reserved as "none". */
#define KB_TX_OWNER_NONE       0u
#define KB_TX_OWNER_OTP        1u
#define KB_TX_OWNER_COMPANION  2u

/* HID keyboard modifier byte, bit 1: left shift (KEYBOARD_MODIFIER_LEFTSHIFT
 * in TinyUSB; redeclared locally to keep this module dependency-free). */
#define KB_TX_MOD_LEFTSHIFT 0x02u

/* Queued keystroke storage, in bytes (one keystroke per byte, like the
 * legacy keyboard_buffer). */
#define KB_TX_BUFFER_MAX 256u

/* Transport ops, injected so the state machine stays host-testable without
 * TinyUSB. In firmware the adapter wires them to the keyboard HID instance:
 * ready -> tud_hid_n_ready(ITF_HID_KB) (never the generic tud_hid_ready(),
 * which checks HID instance 0), send -> tud_hid_n_keyboard_report(ITF_HID_KB,
 * REPORT_ID_KEYBOARD, modifier, keycodes), lookup -> the ASCII conversion
 * table used for encode=true buffers. */
typedef struct {
    bool (*ready)(void);
    bool (*send)(uint8_t modifier, const uint8_t *keycodes);
    /* May be NULL: encode=true buffers then fall back to the raw rule. */
    void (*lookup)(uint8_t ascii, uint8_t *modifier, uint8_t *keycode);
} kb_tx_ops_t;

/* Install the transport ops and reset the state. Passing NULL disables the
 * transmitter (kb_tx_task() becomes a no-op). */
void kb_tx_init(const kb_tx_ops_t *ops);

/* True while the transmitter is claimed or reports are still going out. */
bool kb_tx_busy(void);
/* True while reports are still going out (queued text or an unpaired
 * key-down), whether or not an owner holds the claim. */
bool kb_tx_typing(void);
/* The current claimant (KB_TX_OWNER_NONE if free). */
uint8_t kb_tx_owner(void);

/* Claim the transmitter for `owner`. Fails when anyone holds the claim or
 * reports are in flight. */
bool kb_tx_claim(uint8_t owner);
/* Release the claim. Queued text keeps draining (fire-and-forget typing);
 * the release guarantee still applies. Fails for a non-owner. */
bool kb_tx_release(uint8_t owner);

/* Start a fresh typing transaction for `owner`: refused when the owner does
 * not hold the claim or when reports are still going out (never overwrite
 * in-flight text); otherwise the queue is replaced. Buffers longer than
 * KB_TX_BUFFER_MAX are truncated (legacy add_keyboard_buffer behavior). */
bool kb_tx_add_buffer(uint8_t owner, const uint8_t *data, size_t len, bool encode);

/* Extend the current transaction of `owner` (add then append, like the OTP
 * producer). Never reorders or replaces queued text; refused on overflow. */
bool kb_tx_append_buffer(uint8_t owner, const uint8_t *data, size_t len);

/* Pump the transmitter: send the next key-down or the pending all-released
 * report whenever the transport is ready. Call once per main-loop
 * iteration; never blocks. */
void kb_tx_task(void);

#endif // KB_TX_H
