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

/* Host tests for the single-owner keyboard transmitter (src/usb/hid/kb_tx.c).
 *
 * The transport is recorded: the fake keyboard endpoint accepts one report
 * and stays busy (not ready) until the test acks it, mirroring TinyUSB HID
 * semantics (the keyboard instance is not ready while a report is in
 * flight). One process per scenario, selected by argv[1], so ctest names are
 * readable (ctest -R kb_tx).
 *
 * Covered contract:
 *   - single-owner claim: a second owner's claim fails while the first
 *     holds the transmitter; a non-owner's release is rejected; kb_tx_busy()
 *     is true from the claim until the final release report.
 *   - add never overwrites in-flight text: kb_tx_add_buffer() is refused
 *     while reports are going out, so the queued text is typed in full and
 *     in order.
 *   - every key-down that was sent is followed by an all-zero (all keys
 *     released) report: on normal completion, when the owner releases or
 *     another owner claims mid-press, and when the endpoint reports
 *     not-ready for N polls in between.
 *   - readiness: no report is attempted at all while ops.ready() is false.
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kb_tx.h"

#define MAX_SENT 1200

typedef struct {
    bool ready;      /* keyboard endpoint usable (mounted + configured) */
    bool ep_busy;    /* a report is in flight until the test acks it */
    struct {
        uint8_t modifier;
        uint8_t keycodes[6];
    } sent[MAX_SENT];
    int n;
    int send_calls;  /* send() invocations, including refused ones */
} recorder_t;

static recorder_t rec;

static void rec_reset(void) {
    memset(&rec, 0, sizeof(rec));
    rec.ready = true;
}

static bool rec_ready(void) {
    return rec.ready && !rec.ep_busy;
}

static bool rec_send(uint8_t modifier, const uint8_t *keycodes) {
    rec.send_calls++;
    if (!rec_ready()) {
        return false;
    }
    assert(rec.n < MAX_SENT);
    rec.sent[rec.n].modifier = modifier;
    memcpy(rec.sent[rec.n].keycodes, keycodes, 6);
    rec.n++;
    rec.ep_busy = true;
    return true;
}

/* Fake ASCII lookup (stands in for the adapter's conv_table): uppercase
 * letters type with LEFTSHIFT, everything else types as its own code. */
static void rec_lookup(uint8_t ascii, uint8_t *modifier, uint8_t *keycode) {
    *modifier = (ascii >= 'A' && ascii <= 'Z') ? KB_TX_MOD_LEFTSHIFT : 0;
    *keycode = ascii;
}

static const kb_tx_ops_t ops = { .ready = rec_ready, .send = rec_send, .lookup = rec_lookup };

/* One pump tick plus a host ack: at most one report can go out per tick. */
static void tick(void) {
    kb_tx_task();
    rec.ep_busy = false;
}

/* Pump until the transmitter stops sending (idle or endpoint stuck busy). */
static void drain(int max_ticks) {
    int last_n = -1;
    for (int i = 0; i < max_ticks; i++) {
        tick();
        if (rec.n == last_n) {
            return;
        }
        last_n = rec.n;
    }
}

static bool is_release(int i) {
    if (rec.sent[i].modifier != 0) {
        return false;
    }
    for (int k = 0; k < 6; k++) {
        if (rec.sent[i].keycodes[k] != 0) {
            return false;
        }
    }
    return true;
}

static bool is_press(int i, uint8_t modifier, uint8_t keycode) {
    return rec.sent[i].modifier == modifier && rec.sent[i].keycodes[0] == keycode;
}

static void test_claim_owner(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_busy() == false);
    assert(kb_tx_owner() == KB_TX_OWNER_NONE);

    /* Owner A claims. */
    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_owner() == KB_TX_OWNER_OTP);
    assert(kb_tx_busy() == true);

    /* Owner B's claim fails while A holds the transmitter. */
    assert(kb_tx_claim(KB_TX_OWNER_COMPANION) == false);
    assert(kb_tx_owner() == KB_TX_OWNER_OTP);

    /* Release by a non-owner is rejected. */
    assert(kb_tx_release(KB_TX_OWNER_COMPANION) == false);
    assert(kb_tx_owner() == KB_TX_OWNER_OTP);
    assert(kb_tx_busy() == true);

    /* Release by the owner clears the claim. */
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_owner() == KB_TX_OWNER_NONE);
    assert(kb_tx_busy() == false);

    /* The freed transmitter accepts a new owner. */
    assert(kb_tx_claim(KB_TX_OWNER_COMPANION) == true);
    assert(kb_tx_release(KB_TX_OWNER_COMPANION) == true);

    printf("claim_owner: PASS\n");
}

static void test_busy_until_final_release(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t text[] = { 0x61, 0x62 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, text, sizeof(text), false) == true);

    /* Fire-and-forget: the owner releases while text is still queued. */
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_busy() == true); /* text still in flight */

    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));
    assert(kb_tx_busy() == true);

    drain(64);
    assert(rec.n == 4); /* press+release for both keys */
    assert(is_release(1) && is_release(3));
    assert(kb_tx_busy() == false); /* final release report sent */

    /* A new owner can claim only after the last report. */
    assert(kb_tx_claim(KB_TX_OWNER_COMPANION) == true);
    assert(kb_tx_release(KB_TX_OWNER_COMPANION) == true);

    printf("busy_until_final_release: PASS\n");
}

static void test_no_overwrite(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t abc[] = { 0x61, 0x62, 0x63 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, abc, sizeof(abc), false) == true);

    /* Advance one report: the 'a' key-down is sent. */
    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));

    /* A second buffer (OTP retry or the companion) is refused: nothing is
     * overwritten or interleaved, "abc" comes out in full and in order. */
    const uint8_t xy[] = { 'X', 'Y' };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, xy, sizeof(xy), true) == false);
    assert(kb_tx_add_buffer(KB_TX_OWNER_COMPANION, xy, sizeof(xy), true) == false);
    assert(kb_tx_owner() == KB_TX_OWNER_OTP);

    drain(64);
    assert(rec.n == 6);
    assert(is_press(0, 0, 0x61) && is_release(1));
    assert(is_press(2, 0, 0x62) && is_release(3));
    assert(is_press(4, 0, 0x63) && is_release(5));
    for (int i = 0; i < rec.n; i++) {
        assert(!is_press(i, KB_TX_MOD_LEFTSHIFT, 'X'));
        assert(!is_press(i, KB_TX_MOD_LEFTSHIFT, 'Y'));
    }
    assert(kb_tx_busy() == true); /* the OTP owner still holds the claim */
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_busy() == false);

    printf("no_overwrite: PASS\n");
}

static void test_add_requires_idle_and_owner(void) {
    rec_reset();
    kb_tx_init(&ops);

    /* No claim held: add is refused. */
    const uint8_t a[] = { 0x61 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == false);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == true);

    /* While the text is still going out, a new add is refused. */
    tick();
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == false);

    drain(64);
    /* Idle again: add starts a fresh transaction (replaces, does not append). */
    const uint8_t b[] = { 0x62 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, b, sizeof(b), false) == true);
    drain(64);
    assert(rec.n == 4); /* a-press, a-release, b-press, b-release */
    assert(is_press(2, 0, 0x62));

    /* Not the owner: refused. */
    const uint8_t c[] = { 0x63 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_COMPANION, c, sizeof(c), false) == false);

    printf("add_requires_idle_and_owner: PASS\n");
}

static void test_append(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t ab[] = { 0x61, 0x62 };
    const uint8_t c[] = { 0x63 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, ab, sizeof(ab), false) == true);
    /* Extending the queued text of the owning transaction is allowed. */
    assert(kb_tx_append_buffer(KB_TX_OWNER_OTP, c, sizeof(c)) == true);
    assert(kb_tx_append_buffer(KB_TX_OWNER_COMPANION, c, sizeof(c)) == false);

    drain(64);
    assert(rec.n == 6);
    assert(is_press(4, 0, 0x63) && is_release(5));
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_busy() == false);

    /* Overflow is refused, never silently wrapped. */
    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    uint8_t big[KB_TX_BUFFER_MAX];
    memset(big, 0x61, sizeof(big));
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, big, sizeof(big), false) == true);
    const uint8_t one[] = { 0x62 };
    assert(kb_tx_append_buffer(KB_TX_OWNER_OTP, one, sizeof(one)) == false);

    printf("append: PASS\n");
}

static void test_capacity(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    /* Like the legacy add_keyboard_buffer, a buffer longer than the storage
     * is truncated to the storage size (and never wraps to zero). */
    uint8_t big[300];
    memset(big, 0x64, sizeof(big));
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, big, sizeof(big), false) == true);
    drain(2048);
    assert(rec.n == 2 * KB_TX_BUFFER_MAX);
    for (int i = 0; i < rec.n; i += 2) {
        assert(is_press(i, 0, 0x64) && is_release(i + 1));
    }
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_busy() == false);

    printf("capacity: PASS\n");
}

static void test_release_normal(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t text[] = { 0x61, 0x62, 0x63 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, text, sizeof(text), false) == true);

    drain(64);
    assert(rec.n == 6);
    for (int i = 0; i < rec.n; i += 2) {
        assert(is_release(i + 1)); /* every key-down followed by all-released */
    }
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);
    assert(kb_tx_busy() == false);

    printf("release_normal: PASS\n");
}

static void test_release_owner_change(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t a[] = { 0x61 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == true);

    /* Key-down is out, then the owner changes state mid-press. */
    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);

    /* The next successful report is still the all-released report. */
    tick();
    assert(rec.n == 2 && is_release(1));
    tick();
    assert(rec.n == 2); /* nothing else was sent */
    assert(kb_tx_busy() == false);

    /* The freed transmitter types the next owner cleanly. */
    assert(kb_tx_claim(KB_TX_OWNER_COMPANION) == true);
    const uint8_t f13[] = { 0x68 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_COMPANION, f13, sizeof(f13), false) == true);
    drain(64);
    assert(rec.n == 4);
    assert(is_press(2, 0, 0x68) && is_release(3));

    printf("release_owner_change: PASS\n");
}

static void test_release_claim_race(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t a[] = { 0x61 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == true);

    /* Key-down is out; another owner's claim mid-press fails... */
    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));
    assert(kb_tx_claim(KB_TX_OWNER_COMPANION) == false);

    /* ...and the next successful report is the all-released report. */
    tick();
    assert(rec.n == 2 && is_release(1));
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);

    printf("release_claim_race: PASS\n");
}

static void test_release_not_ready(void) {
    rec_reset();
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t a[] = { 0x61 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == true);

    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));

    /* The keyboard endpoint reports not-ready for N polls: the press report
     * is in flight and the host never acks it. */
    rec.ep_busy = true;
    for (int i = 0; i < 5; i++) {
        kb_tx_task();
        assert(rec.n == 1); /* nothing sent while not ready */
    }

    /* Ready again: the next successful report is the all-released report. */
    rec.ep_busy = false;
    tick();
    assert(rec.n == 2 && is_release(1));
    assert(kb_tx_release(KB_TX_OWNER_OTP) == true);

    printf("release_not_ready: PASS\n");
}

static void test_readiness_gate(void) {
    rec_reset();
    rec.ready = false; /* keyboard endpoint unusable */
    kb_tx_init(&ops);

    assert(kb_tx_claim(KB_TX_OWNER_OTP) == true);
    const uint8_t a[] = { 0x61 };
    assert(kb_tx_add_buffer(KB_TX_OWNER_OTP, a, sizeof(a), false) == true);

    for (int i = 0; i < 5; i++) {
        kb_tx_task();
    }
    assert(rec.send_calls == 0); /* no report attempted at all */
    assert(rec.n == 0);
    assert(kb_tx_busy() == true); /* text still pending */

    /* Once ready, the queued text goes out, press then release. */
    rec.ready = true;
    tick();
    assert(rec.n == 1 && is_press(0, 0, 0x61));
    tick();
    assert(rec.n == 2 && is_release(1));

    printf("readiness_gate: PASS\n");
}

int main(int argc, char **argv) {
    const char *scene = argc > 1 ? argv[1] : "";
    if (strcmp(scene, "claim_owner") == 0) {
        test_claim_owner();
    }
    else if (strcmp(scene, "busy_until_final_release") == 0) {
        test_busy_until_final_release();
    }
    else if (strcmp(scene, "no_overwrite") == 0) {
        test_no_overwrite();
    }
    else if (strcmp(scene, "add_requires_idle_and_owner") == 0) {
        test_add_requires_idle_and_owner();
    }
    else if (strcmp(scene, "append") == 0) {
        test_append();
    }
    else if (strcmp(scene, "capacity") == 0) {
        test_capacity();
    }
    else if (strcmp(scene, "release_normal") == 0) {
        test_release_normal();
    }
    else if (strcmp(scene, "release_owner_change") == 0) {
        test_release_owner_change();
    }
    else if (strcmp(scene, "release_claim_race") == 0) {
        test_release_claim_race();
    }
    else if (strcmp(scene, "release_not_ready") == 0) {
        test_release_not_ready();
    }
    else if (strcmp(scene, "readiness_gate") == 0) {
        test_readiness_gate();
    }
    else {
        fprintf(stderr, "unknown scenario: %s\n", scene);
        return 2;
    }
    return 0;
}
