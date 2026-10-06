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

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "apdu.h"
#include "button.h"
#include "ctap_hid.h"
#include "emulation.h"
#include "led/led.h"
#include "serial.h"
#include "signal.h"
#include "usb.h"
#include "usb/emulation/button_emul.h"

/*
 * Host regression for the CTAPHID_CANCEL fast same-channel retry race.
 *
 * Compiles the UNMODIFIED production src/usb/hid/hid.c, src/usb/usb.c,
 * src/button.c (emulated-button path) and src/usb/emulation/button_emul.c
 * with ENABLE_EMULATION + FORCE_BUTTON_WAIT, drives the real core0-side
 * packet processing (tud_hid_set_report_cb -> driver_process_usb_packet_hid,
 * hid_task/card_status, button_task) and models the CBOR worker (core1)
 * with the exact queue protocol of the production code:
 * - the command loop of cbor_thread (src/cbor.c): remove an event,
 *   acknowledge it, run the command, queue EV_EXEC_FINISHED;
 * - the user-presence wait of wait_button_pressed_timeout
 *   (pico-fido src/fido/fido.c): request EV_PRESS_BUTTON, then remove
 *   events from usb_to_card_q until a button event arrives, DISCARDING
 *   anything else.
 *
 * The race: after CTAPHID_CANCEL fabricates the 0x2D keepalive-cancel
 * response, a same-channel CBOR retry arriving before the next 10 ms
 * button poll used to be admitted immediately; its EV_CMD_AVAILABLE was
 * removed and discarded by the still-pending UP wait, so the retry was
 * never processed. The fix defers admission (buffering the packet on the
 * HID side) until the cancelled transaction's marked completion has been
 * consumed, and delivers a pending cancellation at wait start instead of
 * discarding it. One scenario per process (argv[1]).
 */

/* Private to hid.c: the status byte of the fabricated keepalive-cancel
 * CTAPHID_CBOR response. */
#ifndef CTAPHID_KEEPALIVE_CANCEL_STATUS
#define CTAPHID_KEEPALIVE_CANCEL_STATUS 0x2D
#endif

/* Production entry points defined in src/usb/hid/hid.c. */
extern void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize);
extern void hid_task(void);
extern void button_task(void);
/* Failure-dump accessor (hid.c compiles it with HID_CANCEL_TEST_HOOKS). */
extern void hid_cancel_test_state(uint8_t *txn_state, uint32_t *txn_cid, unsigned *def_count, bool *cancel_marker);

/* ------------------------------------------------------------------ *
 * Mocks for the symbols the production sources reference but that live
 * in files not compiled here (cbor.c, apdu.c, app.c, led, signal, ...).
 * ------------------------------------------------------------------ */
struct apdu apdu;
picokey_serial_t pico_serial;
const uint8_t fido_aid[] = { 0x00 };
const uint8_t u2f_aid[] = { 0x00 };
const uint8_t oath_aid[] = { 0x00 };

static uint32_t led_mode = MODE_MOUNTED;
uint32_t led_get_mode(void) { return led_mode; }
void led_set_mode(uint32_t mode) { led_mode = mode; }

int select_app(const_byte_array_t aid) { (void) aid; return 0; }
void init_fido(void) {}
void low_flash_init_core1(void) {}
int set_atr(void) { return 0; }
void ccid_init(void) {}
uint16_t apdu_process(uint8_t itf, const_byte_array_t buffer) { (void) itf; (void) buffer; return 0; }
void *apdu_thread(void *arg) { (void) arg; return NULL; }
int signal_emit_param(signal_code_t code, void *data) { (void) code; (void) data; return 0; }
int signal_emit(signal_code_t code) { (void) code; return 0; }

uint8_t emul_rx[USB_BUFFER_SIZE];
uint16_t emul_rx_size = 0, emul_tx_size = 0;
/* Declared by compat/queue.h; defined here (the emulator build defines them
 * in its own main/emulation unit). */
pthread_t hcore0, hcore1;
int emul_init(const char *host, uint16_t port) { (void) host; (void) port; return 0; }
void emul_task(void) {}
uint16_t emul_read(uint8_t itf) { (void) itf; return 0; }

/* Captured TX: every 64-byte report the production code submits. Room for
 * the pre-fix behaviour of a 129-report message, where every report past the
 * old 3-slot ring is answered with a CHANNEL_BUSY error frame. */
#define MAX_TX_FRAMES 300
static uint8_t tx_frames[MAX_TX_FRAMES][HID_RPT_SIZE];
static int tx_count = 0;
bool tud_hid_n_report(uint8_t itf, uint8_t report_id, const uint8_t *buffer, uint32_t n) {
    (void) itf;
    (void) report_id;
    assert(n == HID_RPT_SIZE);
    assert(tx_count < MAX_TX_FRAMES);
    memcpy(tx_frames[tx_count++], buffer, HID_RPT_SIZE);
    return true;
}

/* Worker model state. mock_parse_calls counts admissions (the mocked
 * cbor_process); the response marker equals the parse index (1-based), so a
 * response frame's payload identifies which command produced it. */
static int mock_parse_calls = 0;
static uint8_t mock_parse_cmds[8];
static uint8_t mock_parse_payload[8][8];
static size_t mock_parse_lens[8];
/* The test pushes one flag per admitted command (in order); the worker pops
 * one per parse, mirroring the FIFO the real queues provide. */
static volatile bool mock_up_flags[8];
static int mock_up_flag_tail = 0;          /* test side (per admission) */
static volatile int mock_up_flag_head = 0; /* worker side (per parse) */
static int worker_up_waits = 0, worker_pressed = 0, worker_cancelled = 0, worker_timed_out = 0;
/* Snapshot of mock_parse_calls taken by the worker thread at the moment it
 * observes the cancellation: the retry must not have been parsed by then. */
static volatile int worker_cancel_parse_count = -1;
static volatile int worker_parse_index = 0;

static void mock_up_push(bool required) {
    assert(mock_up_flag_tail < 8);
    mock_up_flags[mock_up_flag_tail++] = required;
}

static bool mock_up_pop(void) {
    if (mock_up_flag_head >= mock_up_flag_tail) {
        return false;
    }
    return mock_up_flags[mock_up_flag_head++];
}

/* Faithful cbor_process (src/cbor.c): install the payload pointers and
 * return 2 (card_start ITF_HID + cbor_thread + EV_CMD_AVAILABLE follow in
 * hid.c). */
int cbor_process(uint8_t cmd, const uint8_t *data, size_t len) {
    assert(mock_parse_calls < 8);
    mock_parse_cmds[mock_parse_calls] = cmd;
    memset(mock_parse_payload[mock_parse_calls], 0, 8);
    memcpy(mock_parse_payload[mock_parse_calls], data, len < 8 ? len : 8);
    mock_parse_lens[mock_parse_calls] = len;
    mock_parse_calls++;
    ctap_resp->init.data[0] = 0;
    apdu.rdata = ctap_resp->init.data + 1;
    apdu.rlen = 0;
    return 2;
}

/* Faithful CBOR worker (cbor_thread + fido.c UP wait queue protocol). */
void *cbor_thread(void *arg) {
    (void) arg;
    card_init_core1();
    while (1) {
        uint32_t m;
        queue_remove_blocking(&usb_to_card_q, &m);
        uint32_t flag = m + 1;
        queue_add_blocking(&card_to_usb_q, &flag);
        if (m == EV_EXIT) {
            break;
        }
        if (mock_up_pop()) {
            worker_up_waits++;
            uint32_t val = EV_PRESS_BUTTON_WITH_TIMEOUT(2);
            queue_try_add(&card_to_usb_q, &val);
            do {
                queue_remove_blocking(&usb_to_card_q, &val);
            } while (val != EV_BUTTON_PRESSED && val != EV_BUTTON_TIMEOUT && val != EV_BUTTON_CANCELLED);
            if (val == EV_BUTTON_PRESSED) {
                worker_pressed++;
            }
            else if (val == EV_BUTTON_CANCELLED) {
                worker_cancelled++;
                worker_cancel_parse_count = mock_parse_calls;
            }
            else {
                worker_timed_out++;
            }
        }
        apdu.rdata[0] = (uint8_t) (worker_parse_index + 1);
        apdu.rlen = 1;
        apdu.sw = 0;
        finished_data_size = apdu.rlen + 1;
        worker_parse_index++;
        flag = EV_EXEC_FINISHED;
        queue_add_blocking(&card_to_usb_q, &flag);
    }
    return NULL;
}

/* ------------------------------------------------------------------ *
 * Emulated button control file (same pattern as button_emul_test.c).
 * ------------------------------------------------------------------ */
static char btn_file[256], btn_tmp[260];

static void write_btn(const char *text) {
    FILE *f = fopen(btn_tmp, "wb");
    assert(f != NULL);
    fputs(text, f);
    fclose(f);
    assert(rename(btn_tmp, btn_file) == 0);
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 2000000 };
    nanosleep(&ts, NULL);
}

static void env_setup(void) {
    /* Unique per process: ctest -j runs the long randomized scenes
     * concurrently, and a shared control file would leak presses across
     * processes. */
    snprintf(btn_file, sizeof(btn_file), "/tmp/picokeys_hid_cancel_retry_test_%d.cmd", (int) getpid());
    snprintf(btn_tmp, sizeof(btn_tmp), "%s.tmp", btn_file);
    setenv("PICOKEYS_EMULATION_BUTTON_FILE", btn_file, 1);
    unsetenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT");
    remove(btn_file);
    remove(btn_tmp);
}

/* ------------------------------------------------------------------ *
 * CTAPHID packet helpers.
 * ------------------------------------------------------------------ */
static const uint32_t CID = 0x0a0b0c0d;
static const uint32_t CID_B = 0x11223344;
static uint8_t report[HID_RPT_SIZE];

static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t) (v >> 24);
    p[1] = (uint8_t) (v >> 16);
    p[2] = (uint8_t) (v >> 8);
    p[3] = (uint8_t) v;
}

static void build_init(void) {
    memset(report, 0, sizeof(report));
    put32(report, CID);
    report[4] = CTAPHID_INIT;
    report[6] = 8;
    for (uint8_t i = 0; i < 8; i++) {
        report[7 + i] = (uint8_t) (0x40 + i);
    }
}

static void build_cbor(const uint8_t *payload, uint8_t len) {
    memset(report, 0, sizeof(report));
    put32(report, CID);
    report[4] = CTAPHID_CBOR;
    report[6] = len;
    memcpy(report + 7, payload, len);
}

static void build_cancel(void) {
    memset(report, 0, sizeof(report));
    put32(report, CID);
    report[4] = CTAPHID_CANCEL;
}

static void send_report(void) {
    tud_hid_set_report_cb(ITF_HID_CTAP, 0, 0, report, HID_RPT_SIZE);
}

/* CTAPHID init packet of a fragmented message: [cid][cmd][len16][first
 * payload bytes]. */
static void send_cbor_init_packet(uint32_t cid, const uint8_t *payload, uint16_t len) {
    memset(report, 0, sizeof(report));
    put32(report, cid);
    report[4] = CTAPHID_CBOR;
    report[5] = (uint8_t) (len >> 8);
    report[6] = (uint8_t) len;
    size_t n = len < (HID_RPT_SIZE - 7) ? len : (HID_RPT_SIZE - 7);
    memcpy(report + 7, payload, n);
    send_report();
}

/* Every continuation report of a fragmented message, in order. */
static void send_cbor_continuations(uint32_t cid, const uint8_t *payload, uint16_t len) {
    uint16_t sent = len < (HID_RPT_SIZE - 7) ? len : (HID_RPT_SIZE - 7);
    uint8_t seq = 0;
    while (sent < len) {
        uint16_t remain = (uint16_t) (len - sent);
        memset(report, 0, sizeof(report));
        put32(report, cid);
        /* Continuation frame: [cid][seq (type bit b7 cleared)][59 payload] */
        report[4] = seq++;
        size_t cn = remain < (HID_RPT_SIZE - 5) ? remain : (HID_RPT_SIZE - 5);
        memcpy(report + 5, payload + sent, cn);
        sent = (uint16_t) (sent + cn);
        send_report();
    }
}

/* Fragmented CTAPHID_CBOR message: an init packet carrying the 16-bit total
 * length plus 59-byte continuations, exactly as a real client fragments one
 * request. send_cbor_msg_full() delivers every report back-to-back with no
 * hid_task/button_task pump in between - the pre-poll delivery ordering of
 * the supported SET_REPORT control-transfer path. */
static void send_cbor_msg_full(uint32_t cid, const uint8_t *payload, uint16_t len) {
    send_cbor_init_packet(cid, payload, len);
    send_cbor_continuations(cid, payload, len);
}

/* One-report CTAPHID_PING: the transport answers it inline (no worker). */
static void send_ping_packet(uint32_t cid, const uint8_t *payload, uint8_t len) {
    memset(report, 0, sizeof(report));
    put32(report, cid);
    report[4] = CTAPHID_PING;
    report[6] = len;
    memcpy(report + 7, payload, len);
    send_report();
}

/* CTAPHID_INIT resync with an explicit nonce, so the response can be
 * matched to this exact request. */
static void send_init_packet(uint32_t cid, const uint8_t nonce[8]) {
    memset(report, 0, sizeof(report));
    put32(report, cid);
    report[4] = CTAPHID_INIT;
    report[6] = 8;
    memcpy(report + 7, nonce, 8);
    send_report();
}

static bool frame_has_cid(const uint8_t *f, uint32_t cid) {
    return f[0] == ((cid >> 24) & 0xff) && f[1] == ((cid >> 16) & 0xff) &&
           f[2] == ((cid >> 8) & 0xff) && f[3] == (cid & 0xff);
}

/* The index of the first CTAPHID_ERROR frame addressed to `cid` at or after
 * `from`, or -1. */
static int find_error_frame_for(uint32_t cid, int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_ERROR && frame_has_cid(f, cid)) {
            return i;
        }
    }
    return -1;
}

/* The index of the first INIT response echoing `nonce` on `cid`, or -1. */
static int find_init_response(uint32_t cid, const uint8_t nonce[8], int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_INIT && f[6] == 17 && memcmp(f + 7, nonce, 8) == 0 &&
            frame_has_cid(f, cid)) {
            return i;
        }
    }
    return -1;
}

/* The index of the first one-report PING echo of `payload` on `cid`, or -1. */
static int find_ping_echo(uint32_t cid, const uint8_t *payload, uint8_t len, int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_PING && f[5] == 0 && f[6] == len &&
            memcmp(f + 7, payload, len) == 0 && frame_has_cid(f, cid)) {
            return i;
        }
    }
    return -1;
}

static int find_error_frame(int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_ERROR) {
            return i;
        }
    }
    return -1;
}

/* The index of the first TX frame carrying `cmd` for our channel at or
 * after `from`, or -1. */
static int find_frame(uint8_t cmd, int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == cmd && f[0] == ((CID >> 24) & 0xff) && f[1] == ((CID >> 16) & 0xff)) {
            return i;
        }
    }
    return -1;
}

static int find_cbor_status_frame(uint8_t status, int from) {
    for (int i = from; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 1 && f[7] == status) {
            return i;
        }
    }
    return -1;
}

/* Pump the core0 loop until cond() holds or timeout_ms elapse. Each round
 * sleeps ~2 ms, so the 1 ms card_status poll and the 10 ms button poll gate
 * both run exactly as production paces them. */
/* Pump ONLY the HID task for a while: the production core0 loop always runs
 * hid_task (which flushes the fabricated 0x2D response) between the cancel
 * packet and any later packet, but must not run button_task here - that is
 * exactly the pre-poll window under test. */
static void pump_hid(unsigned ms) {
    for (unsigned waited = 0; waited < ms; waited += 2) {
        usleep(2000);
        hid_task();
    }
}

static bool spin_until(bool (*cond)(void), unsigned timeout_ms) {
    for (unsigned waited = 0; waited < timeout_ms; waited += 2) {
        if (cond()) {
            return true;
        }
        usleep(2000);
        hid_task();
        button_task();
    }
    return cond();
}

static bool old_completion_dropped(void) {
    return !exec_finished_cancelled;
}

static bool worker_saw_cancel(void) {
    return worker_cancelled == 1;
}

static void channel_setup(void) {
    tx_count = 0;
    build_init();
    send_report();
    assert(find_frame(CTAPHID_INIT, 0) == 0);
}

/* ------------------------------------------------------------------ *
 * Scenes.
 * ------------------------------------------------------------------ */

/* THE regression: the retry is sent immediately after the fabricated 0x2D,
 * with NO button poll in between. The fix must defer its admission until
 * the cancelled transaction has unwound, then process it exactly once with
 * a correctly framed response. */
static void scene_fast_retry(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t retry_payload[] = { 0x04, 0xbb };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    /* Pending UP wait. */
    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(mock_parse_calls == 1);
    assert(spin_until(is_req_button_pending, 2000));

    /* Host cancels; exactly one fabricated 0x2D goes out. */
    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, cancel_resp + 1) < 0);
    assert(exec_finished_cancelled);

    /* The retry arrives BEFORE any button poll ran: it must be buffered,
     * not admitted. */
    mock_up_push(false);
    build_cbor(retry_payload, sizeof(retry_payload));
    send_report();
    usleep(30000);
    hid_task();
    assert(mock_parse_calls == 1);
    assert(is_req_button_pending());

    /* Delivering the cancellation unwinds the old command; its late
     * completion is dropped and the buffered retry is replayed and
     * processed exactly once. */
    assert(spin_until(old_completion_dropped, 2000));
    assert(worker_cancelled == 1);
    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(mock_parse_calls == 2);
    assert(mock_parse_cmds[1] == CTAPHID_CBOR);
    assert(mock_parse_payload[1][1] == 0xbb);
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    /* Exactly one response frame after the 0x2D: no stray frames. */
    int first_resp = -1;
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) { /* a CBOR response, not keepalive */
            assert(first_resp < 0);
            first_resp = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker);
        }
    }
    assert(first_resp > 0);
    printf("fast_retry: retry admitted only after the cancelled transaction unwound, "
           "answered exactly once with a correctly framed response\n");
}

/* Scrutiny round 4: a legal fragmented same-channel retry (makeCredential
 * payloads are commonly larger than one report) delivered entirely before
 * the next cancellation poll. The deferral ring must retain EVERY report of
 * ONE CTAPHID message; losing any continuation to CHANNEL_BUSY leaves the
 * reassembled message incomplete, its handler never runs and the partial
 * message times out. */
static void scene_fast_retry_fragmented(void) {
    static uint8_t retry_payload[200]; /* init + 3 continuations = 4 reports */
    for (unsigned i = 0; i < sizeof(retry_payload); i++) {
        retry_payload[i] = (uint8_t) (i * 13 + 5);
    }
    static const uint8_t old_payload[] = { 0x04, 0xaa };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    /* Pending UP wait. */
    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    /* Host cancels; exactly one fabricated 0x2D goes out. */
    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, cancel_resp + 1) < 0);
    assert(exec_finished_cancelled);

    /* The 4-report retry arrives BEFORE any button poll ran: every report
     * must be buffered, none answered with an error. pump_hid flushes the
     * TX ring only - no button poll, the unwind window is still open. */
    mock_up_push(false);
    send_cbor_msg_full(CID, retry_payload, sizeof(retry_payload));
    pump_hid(2);
    assert(mock_parse_calls == 1);
    assert(is_req_button_pending());
    assert(find_error_frame(cancel_resp + 1) < 0);

    /* Delivering the cancellation unwinds the old command; its late
     * completion is dropped and the buffered message is replayed and
     * processed exactly once, complete, with a correctly framed response. */
    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancelled == 1);
    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(mock_parse_calls == 2);
    assert(mock_parse_cmds[1] == CTAPHID_CBOR);
    assert(mock_parse_lens[1] == sizeof(retry_payload));
    assert(memcmp(mock_parse_payload[1], retry_payload, 8) == 0);
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    /* Exactly one response frame after the 0x2D and no error frame: nothing
     * was dropped to CHANNEL_BUSY and nothing is stray. */
    int first_resp = -1;
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) { /* a CBOR response, not keepalive */
            assert(first_resp < 0);
            first_resp = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker);
        }
    }
    assert(first_resp > 0);
    assert(find_error_frame(cancel_resp + 1) < 0);
    printf("fast_retry_fragmented: a 4-report fragmented retry buffered whole during the unwind "
           "is processed exactly once with a correctly framed response\n");
}

/* Same delivery ordering at the CTAPHID maximum message size the firmware
 * supports: CTAP_MAX_PACKET_SIZE (7609) payload bytes = init packet plus 128
 * continuations = 129 reports, all arriving before any cancellation poll.
 * The deferral ring must hold exactly one such message. */
static void scene_fast_retry_max_size(void) {
    static uint8_t max_payload[CTAP_MAX_PACKET_SIZE];
    for (unsigned i = 0; i < sizeof(max_payload); i++) {
        max_payload[i] = (uint8_t) (i * 13 + 5);
    }
    static const uint8_t old_payload[] = { 0x04, 0xaa };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(exec_finished_cancelled);

    /* All 129 reports of the maximum-size retry arrive before any poll.
     * pump_hid flushes the TX ring only - no button poll, the unwind
     * window is still open. */
    mock_up_push(false);
    send_cbor_msg_full(CID, max_payload, sizeof(max_payload));
    pump_hid(2);
    assert(mock_parse_calls == 1);
    assert(is_req_button_pending());
    assert(find_error_frame(cancel_resp + 1) < 0);

    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancelled == 1);
    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(mock_parse_calls == 2);
    assert(mock_parse_cmds[1] == CTAPHID_CBOR);
    assert(mock_parse_lens[1] == CTAP_MAX_PACKET_SIZE);
    assert(memcmp(mock_parse_payload[1], max_payload, 8) == 0);
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    int first_resp = -1;
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) {
            assert(first_resp < 0);
            first_resp = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker);
        }
    }
    assert(first_resp > 0);
    assert(find_error_frame(cancel_resp + 1) < 0);
    printf("fast_retry_max_size: a 129-report maximum-size retry buffered whole during the unwind "
           "is processed exactly once with a correctly framed response\n");
}

/* The cancel arrives right after the admission keepalive, BEFORE the UP
 * wait has started (its EV_PRESS_BUTTON is still queued): the wait start
 * must deliver the pending cancellation, and the retry must still be
 * answered exactly once. */
static bool retry_parsed(void) {
    return mock_parse_calls == 2;
}

static void scene_cancel_before_wait_start(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t retry_payload[] = { 0x04, 0xbb };

    write_btn("none");
    write_btn("timeout:0");
    /* Process the control file now: without this the emulated button is
     * still in auto mode and the wait would complete without ever running. */
    button_task();
    channel_setup();

    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    /* No pump and no settle before the cancel: whatever the worker has
     * done by now, its EV_PRESS_BUTTON can only be consumed after the
     * cancel is marked, i.e. inside the pre-start window. */
    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);

    mock_up_push(false);
    build_cbor(retry_payload, sizeof(retry_payload));
    send_report();
    usleep(30000);
    hid_task();
    button_task();

    /* The retry may only be admitted once the cancelled wait has observed
     * its cancellation: the worker's snapshot of the parse count taken when
     * its wait loop returned must still show only the cancelled command. */
    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancel_parse_count == 1);
    assert(spin_until(retry_parsed, 2000));

    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(worker_cancelled == 1);
    assert(old_completion_dropped);
    int stray = -1;
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) {
            assert(stray < 0);
            stray = i;
        }
    }
    assert(stray > 0);
    printf("cancel_before_wait_start: a cancel during the pre-start window cancels the wait; the retry is admitted only after the unwind and answered exactly once\n");
}

/* Settled control: pumps run between the cancel and the retry (the ordering
 * test_cancel_same_channel.py covers end to end). Must keep working. */
static void scene_settled_retry(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t retry_payload[] = { 0x04, 0xbb };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(exec_finished_cancelled);

    /* Settle: the cancellation is delivered to the old wait (pre-fix the
     * marked late completion is consumed lazily at the next admission;
     * post-fix card_status() drops it promptly). */
    assert(spin_until(worker_saw_cancel, 2000));

    mock_up_push(false);
    build_cbor(retry_payload, sizeof(retry_payload));
    send_report();
    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(mock_parse_calls == 2);
    assert(!exec_finished_cancelled && !is_req_button_pending() && !is_busy());
    printf("settled_retry: the settled cancel->retry sequence still answers exactly once\n");
}

/* No-retry control (VAL-UP-027 shape): after the cancel, no stale frame may
 * circulate, and a fresh command works normally. */
static void scene_no_retry_after_cancel(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t next_payload[] = { 0x04, 0xcc };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    /* The cancellation reaches the pending wait. */
    assert(spin_until(worker_saw_cancel, 2000));
    /* No response frame beyond the 0x2D: the late completion was dropped
     * exactly once and nothing else was delivered. */
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        assert(!(f[4] == CTAPHID_CBOR && f[5] == 0));
    }
    assert(mock_parse_calls == 1);

    /* A fresh command on the same channel is admitted and answered. */
    mock_up_push(false);
    build_cbor(next_payload, sizeof(next_payload));
    send_report();
    assert(mock_parse_calls == 2);
    bool next_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !next_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == 2) {
                next_response_seen = true;
            }
        }
        if (!next_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(next_response_seen);
    assert(!is_busy());
    printf("no_retry_after_cancel: no stale frame after the cancel; a fresh command is answered\n");
}

/* Press control: a fresh press still completes a pending wait. */
static void scene_press_control(void) {
    static const uint8_t payload[] = { 0x04, 0xdd };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    mock_up_push(true);
    build_cbor(payload, sizeof(payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    write_btn("press");
    bool response_seen = false;
    for (unsigned waited = 0; waited < 4000 && !response_seen; waited += 2) {
        for (int i = 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == 1) {
                response_seen = true;
            }
        }
        if (!response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(response_seen);
    assert(worker_pressed == 1 && worker_cancelled == 0 && worker_timed_out == 0);
    assert(mock_parse_calls == 1);
    assert(!is_req_button_pending() && !is_busy());
    printf("press_control: a fresh press completes the wait with one framed response\n");
}

/* Scrutiny round 5, blocker 1: during the unwind the deferral ring has no
 * CID ownership, so a competing channel's report consumes one of the 129
 * slots reserved for the protected retry and the retry's final continuation
 * is answered CHANNEL_BUSY and lost. The adversarial order interleaves the
 * competing report between the retry's init packet and its continuations,
 * all before any cancellation poll. The fix must answer the competing
 * channel without consuming a slot or disturbing the buffered message. */
static void scene_competing_cid_during_unwind(void) {
    static uint8_t retry_payload[CTAP_MAX_PACKET_SIZE];
    for (unsigned i = 0; i < sizeof(retry_payload); i++) {
        retry_payload[i] = (uint8_t) (i * 13 + 5);
    }
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint32_t CID_B = 0x11223344;
    static const uint8_t ping_payload[] = { 0x50, 0x51, 0x52 };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    /* Pending UP wait on channel A. */
    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    /* Host cancels A; exactly one fabricated 0x2D goes out. */
    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(exec_finished_cancelled);

    /* Adversarial interleaving BEFORE any cancellation poll: A's max-size
     * retry init packet, B's one-report PING, then A's 128 continuations.
     * pump_hid flushes the TX ring only - no button poll, the unwind
     * window is still open. */
    mock_up_push(false);
    send_cbor_init_packet(CID, retry_payload, sizeof(retry_payload));
    send_ping_packet(CID_B, ping_payload, sizeof(ping_payload));
    send_cbor_continuations(CID, retry_payload, sizeof(retry_payload));
    pump_hid(2);
    assert(mock_parse_calls == 1);
    assert(is_req_button_pending());

    /* B was answered with an explicit CHANNEL_BUSY without consuming a
     * ring slot, and no error frame went to A's session. */
    int b_busy = find_error_frame_for(CID_B, cancel_resp + 1);
    assert(b_busy > 0);
    assert(tx_frames[b_busy][7] == CTAP1_ERR_CHANNEL_BUSY);
    assert(find_error_frame_for(CID, cancel_resp + 1) < 0);

    /* The unwind completes; A's protected retry is processed exactly once,
     * complete, with a correctly framed response. */
    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancelled == 1);
    static const uint8_t retry_marker = 2;
    bool retry_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !retry_response_seen; waited += 2) {
        for (int i = cancel_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker) {
                retry_response_seen = true;
            }
        }
        if (!retry_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(retry_response_seen);
    assert(mock_parse_calls == 2);
    assert(mock_parse_cmds[1] == CTAPHID_CBOR);
    assert(mock_parse_lens[1] == CTAP_MAX_PACKET_SIZE);
    assert(memcmp(mock_parse_payload[1], retry_payload, 8) == 0);
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    /* Exactly one response frame after the 0x2D (A's retry response) and
     * exactly one error frame (B's CHANNEL_BUSY): nothing else is stray. */
    int first_resp = -1;
    for (int i = cancel_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) { /* a CBOR response, not keepalive */
            assert(first_resp < 0);
            first_resp = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == retry_marker);
        }
    }
    assert(first_resp > 0);
    assert(find_error_frame(cancel_resp + 1) == b_busy);

    /* B is not locked out: a fresh PING on B is answered with its echo. */
    static const uint8_t ping2[] = { 0x60, 0x61 };
    send_ping_packet(CID_B, ping2, sizeof(ping2));
    for (unsigned waited = 0; waited < 2000; waited += 2) {
        if (find_ping_echo(CID_B, ping2, sizeof(ping2), first_resp) >= 0) {
            break;
        }
        usleep(2000);
        hid_task();
    }
    assert(find_ping_echo(CID_B, ping2, sizeof(ping2), first_resp) >= 0);
    assert(mock_parse_calls == 2);
    printf("competing_cid_during_unwind: a competing channel is answered CHANNEL_BUSY without "
           "consuming the protected retry's ring capacity; the max-size retry completes exactly once\n");
}

/* Scrutiny round 5, blocker 2: a same-channel CTAPHID_INIT sent after the
 * 0x2D but before the next cancellation poll must not enter card_exit()'s
 * blocking EV_EXIT handshake - the still-active UP wait removes and
 * discards EV_EXIT, so core0 would block forever. The resync aborts
 * non-blockingly: the INIT response goes out immediately (this assert runs
 * pump_hid only, no button poll), the cancellation is still delivered to
 * the old wait, and a fresh command is answered exactly afterwards. */
static void scene_init_after_cancel_active_wait(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t fresh_payload[] = { 0x04, 0xee };
    static const uint8_t nonce[8] = { 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77 };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    /* Pending UP wait. */
    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    build_cancel();
    send_report();
    pump_hid(6);
    int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1);
    assert(cancel_resp > 0);
    assert(exec_finished_cancelled);

    /* Same-channel INIT before any button poll: must be answered without
     * blocking on the worker. */
    send_init_packet(CID, nonce);
    pump_hid(4);
    int init_resp = find_init_response(CID, nonce, cancel_resp + 1);
    assert(init_resp > 0);

    /* The unwind still completes: the wait observes the cancellation, the
     * late completion is dropped. */
    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);

    /* Clean resync: a fresh command on the same channel is admitted and
     * answered exactly once, with no stale frame in between. */
    mock_up_push(false);
    build_cbor(fresh_payload, sizeof(fresh_payload));
    send_report();
    assert(spin_until(retry_parsed, 2000));
    static const uint8_t fresh_marker = 2;
    bool fresh_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !fresh_response_seen; waited += 2) {
        for (int i = init_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == fresh_marker) {
                fresh_response_seen = true;
            }
        }
        if (!fresh_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(fresh_response_seen);
    assert(mock_parse_calls == 2);
    assert(mock_parse_cmds[1] == CTAPHID_CBOR);
    assert(mock_parse_payload[1][1] == 0xee);
    /* Exactly one CBOR response after the INIT response: the aborted
     * transaction's late completion never produced a frame. */
    int stray = -1;
    for (int i = init_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) {
            assert(stray < 0);
            stray = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == fresh_marker);
        }
    }
    assert(stray > 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    printf("init_after_cancel_active_wait: same-channel INIT during the unwind is answered "
           "without blocking, the unwind completes, and the resynced channel works\n");
}

/* INIT during an active UP wait with NO cancellation: the same blocking
 * EV_EXIT hazard exists without any cancel marker, so the resync must
 * abort the wait non-blockingly here too - no CTAPHID_CANCEL was seen, no
 * keepalive-cancel frame is fabricated, and the resynced channel keeps
 * working. */
static void scene_init_during_active_wait(void) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static const uint8_t fresh_payload[] = { 0x04, 0xef };
    static const uint8_t nonce[8] = { 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87 };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    /* Pending UP wait, no cancel sent. */
    mock_up_push(true);
    build_cbor(old_payload, sizeof(old_payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    send_init_packet(CID, nonce);
    pump_hid(4);
    int init_resp = find_init_response(CID, nonce, 1);
    assert(init_resp > 0);
    /* No keepalive-cancel is fabricated: the abort is not a host CANCEL. */
    assert(find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, 1) < 0);

    /* The wait is aborted: the worker observes the cancellation and its
     * late completion is dropped. */
    assert(spin_until(worker_saw_cancel, 2000));
    assert(worker_cancelled == 1 && worker_pressed == 0 && worker_timed_out == 0);

    mock_up_push(false);
    build_cbor(fresh_payload, sizeof(fresh_payload));
    send_report();
    assert(spin_until(retry_parsed, 2000));
    static const uint8_t fresh_marker = 2;
    bool fresh_response_seen = false;
    for (unsigned waited = 0; waited < 2000 && !fresh_response_seen; waited += 2) {
        for (int i = init_resp + 1; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == fresh_marker) {
                fresh_response_seen = true;
            }
        }
        if (!fresh_response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(fresh_response_seen);
    assert(mock_parse_calls == 2);
    int stray = -1;
    for (int i = init_resp + 1; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) {
            assert(stray < 0);
            stray = i;
            assert(f[6] == 2 && f[7] == 0x00 && f[8] == fresh_marker);
        }
    }
    assert(stray > 0);
    assert(find_error_frame(1) < 0);
    assert(!exec_finished_cancelled);
    assert(!is_req_button_pending());
    assert(!is_busy());
    printf("init_during_active_wait: INIT during a plain UP wait aborts it non-blockingly "
           "and the resynced channel works\n");
}

/* ------------------------------------------------------------------ *
 * Seeded randomized-ordering scene.
 *
 * Drives the same production state machine through a few thousand
 * pseudo-randomly ordered legal host sequences (three fixed seeds, one
 * ctest entry each) and checks the consolidated cancel-path invariants
 * after every iteration:
 * - no hang: a per-iteration pump-step budget fails the scene instead of
 *   tripping the 30 s alarm;
 * - every request is processed exactly once with a correctly framed
 *   response, or answered with an explicit CTAPHID error (CHANNEL_BUSY
 *   only for a channel that does not own the in-flight transaction);
 * - a cancelled request's late completion is dropped exactly once (no
 *   response frame for it, the marker cleared by the end);
 * - exactly one fabricated 0x2D keepalive-cancel per cancelled request;
 * - no stale or unaccounted frame in the captured TX window.
 *
 * Iteration shapes (chosen by the PRNG):
 * - CANCEL during an active UP wait or in the pre-start window, then a
 *   same-channel retry of 1, 4 or 129 reports interleaved with
 *   other-channel PING/INIT/CBOR traffic, optionally a press racing the
 *   cancellation delivery, then the unwind;
 * - a press completing the wait (no cancel);
 * - a same-channel INIT resync aborting an active wait, then a fresh
 *   command on the resynced channel.
 *
 * Model exclusions (documented production behavior outside the cancel
 * path's scope, not exercised as defects here):
 * - a CBOR command admitted while a DIFFERENT channel's UP wait is
 *   active without a cancellation is swallowed by the wait's event loop
 *   upstream; the scene only sends other-channel CBOR inside the unwind
 *   window (answered CHANNEL_BUSY by the admission gate) or when no wait
 *   is active (processed exactly once);
 * - a same-channel INIT resync discards the reports buffered for the
 *   dead session by design, so the scene never orphans buffered retry
 *   reports behind a resync: the resync mode sends no retry traffic.
 * The op engine only pumps hid_task() while the unwind window is open
 * (the cancellation is delivered exclusively by button_task()), so every
 * buffered/answered outcome stays deterministic under the fixed seed.
 * On any failure the seed, iteration, phase and line are printed.
 * ------------------------------------------------------------------ */
#include <stdint.h>

#define RND_MAX_B_OPS 8
static uint64_t rnd_seed = 0;
static int rnd_iter = -1;
static const char *rnd_phase = "start";
static int rnd_steps = 0;

/* Filled per iteration by scene_randomized for the failure dump. */
static int rnd_dbg_mode = -1, rnd_dbg_kind = -1, rnd_dbg_nb = -1, rnd_dbg_press = -1;
static int rnd_count_responses(int tx_start);

static void rnd_dbg_dump(void) {
    int resp = -1;
    if (tx_count > 0) {
        resp = rnd_count_responses(0);
    }
    uint8_t txn_state = 0;
    uint32_t txn_cid = 0;
    unsigned def_count = 0;
    bool cancel_marker = false;
    hid_cancel_test_state(&txn_state, &txn_cid, &def_count, &cancel_marker);
    fprintf(stderr,
            "  dbg: mode=%d kind=%d n_b=%d press=%d parse=%d tx=%d resp=%d busy=%d pending=%d marker=%d "
            "wk(c=%d p=%d t=%d) cp=%d flags=%d/%d txn=%u cid=%08x def=%u cbtn=%d\n",
            rnd_dbg_mode, rnd_dbg_kind, rnd_dbg_nb, rnd_dbg_press,
            mock_parse_calls, tx_count, resp, is_busy(), is_req_button_pending(), exec_finished_cancelled,
            worker_cancelled, worker_pressed, worker_timed_out, worker_cancel_parse_count,
            mock_up_flag_head, mock_up_flag_tail, txn_state, txn_cid, def_count, cancel_marker);
    for (int i = 0; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        fprintf(stderr, "  dbg: frame[%d] cid=%02x%02x%02x%02x cmd=%02x bcnth=%02x bcntl=%02d d0=%02x d1=%02x\n",
                i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]);
    }
}

#define RASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "randomized: FAIL seed=0x%08llx iter=%d phase=%s line=%d\n", \
                (unsigned long long) rnd_seed, rnd_iter, rnd_phase, __LINE__); \
        rnd_dbg_dump(); \
        fflush(stderr); \
    } \
    assert(cond); \
} while (0)

static uint64_t rng_state = 0;
static uint32_t rnd_below(uint32_t n) {
    rng_state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = rng_state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return (uint32_t) ((z >> 32) % n);
}

/* One 1 ms core0 round. The unwind window is driven with hid_task() only
 * (rnd_pump_hid): button_task() is what delivers the cancellation, so the
 * window stays closed until the scene runs a full round (rnd_pump_full). */
static void rnd_step(void) {
    rnd_steps++;
    RASSERT(rnd_steps < 600); /* per-iteration step budget: fail, not hang */
}
static void rnd_pump_hid(unsigned rounds) {
    for (unsigned i = 0; i < rounds; i++) {
        usleep(1000);
        hid_task();
        rnd_step();
    }
}
static void rnd_pump_full(unsigned rounds) {
    for (unsigned i = 0; i < rounds; i++) {
        usleep(1000);
        hid_task();
        button_task();
        rnd_step();
    }
}

static const uint32_t RND_CID_B = CID_B;
static uint8_t rnd_b_ping[RND_MAX_B_OPS][4];
static bool rnd_b_ping_done[RND_MAX_B_OPS];
static int rnd_b_pings = 0;
static uint8_t rnd_b_nonce[RND_MAX_B_OPS][8];
static bool rnd_b_nonce_done[RND_MAX_B_OPS];
static int rnd_b_inits = 0;
static int rnd_b_cbor_sent = 0, rnd_b_cbor_admitted = 0;
static bool rnd_have_resync_init = false;
static uint8_t rnd_resync_nonce[8];
static bool rnd_resync_init_done = false;

static void rnd_reset_test_state(void) {
    tx_count = 0;
    mock_parse_calls = 0;
    mock_up_flag_head = 0;
    mock_up_flag_tail = 0;
    worker_up_waits = worker_pressed = worker_cancelled = worker_timed_out = 0;
    worker_cancel_parse_count = -1;
    worker_parse_index = 0;
    rnd_b_pings = rnd_b_inits = rnd_b_cbor_sent = rnd_b_cbor_admitted = 0;
    memset(rnd_b_ping_done, 0, sizeof(rnd_b_ping_done));
    memset(rnd_b_nonce_done, 0, sizeof(rnd_b_nonce_done));
    rnd_have_resync_init = false;
    rnd_resync_init_done = false;
}

/* A CBOR admission while another command is in flight with NO unwind
 * marker is either swallowed by an active UP wait or would rebind the
 * pending response staging (see hid.c resp_data) - both outside the
 * modeled envelope. Such an op sends a PING instead. */
enum { RND_OP_PING, RND_OP_INIT, RND_OP_CBOR };
static int rnd_pick_b_op(void) {
    int op = (int) rnd_below(3);
    if (op == RND_OP_CBOR && !exec_finished_cancelled && is_busy()) {
        op = RND_OP_PING;
    }
    return op;
}

/* Other-channel traffic ops. Outcomes while the unwind window is open
 * (exec_finished_cancelled set) are deterministic: PING and CBOR are
 * answered CHANNEL_BUSY by the admission gate, INIT is echoed by the
 * different-channel allocation branch. After the window closes, a PING is
 * echoed inline, an INIT echoes, and a CBOR is admitted (the worker is
 * free or running a wait-free command, so its event is consumed). */
static void rnd_b_op(int op, uint32_t salt) {
    if (op == RND_OP_PING) {
        RASSERT(rnd_b_pings < RND_MAX_B_OPS);
        uint8_t *p = rnd_b_ping[rnd_b_pings];
        p[0] = 0x50; p[1] = (uint8_t) rnd_iter; p[2] = (uint8_t) salt; p[3] = (uint8_t) rnd_b_pings;
        rnd_b_pings++;
        send_ping_packet(RND_CID_B, p, 4);
    }
    else if (op == RND_OP_INIT) {
        RASSERT(rnd_b_inits < RND_MAX_B_OPS);
        uint8_t *n = rnd_b_nonce[rnd_b_inits];
        n[0] = 0xB5; n[1] = (uint8_t) rnd_iter; n[2] = (uint8_t) salt;
        n[3] = (uint8_t) rnd_b_inits; n[4] = 0x11; n[5] = 0x22; n[6] = 0x33; n[7] = 0x44;
        rnd_b_inits++;
        send_init_packet(RND_CID_B, n);
    }
    else { /* CBOR */
        RASSERT(rnd_b_cbor_sent < RND_MAX_B_OPS);
        rnd_b_cbor_sent++;
        mock_up_push(false); /* consumed only if actually admitted */
        memset(report, 0, sizeof(report));
        put32(report, RND_CID_B);
        report[4] = CTAPHID_CBOR;
        report[6] = 5;
        report[7] = 0x06; report[8] = (uint8_t) rnd_iter; report[9] = (uint8_t) rnd_b_cbor_sent;
        report[10] = 0x5A; report[11] = 0x5B;
        int before = mock_parse_calls;
        send_report();
        if (mock_parse_calls > before) {
            rnd_b_cbor_admitted++; /* admission is synchronous */
        }
    }
}

/* One continuation report of a fragmented message (single-step variant of
 * send_cbor_continuations). */
static void rnd_send_cont(uint32_t cid, const uint8_t *payload, uint16_t len, uint16_t sent, uint8_t seq) {
    uint16_t remain = (uint16_t) (len - sent);
    memset(report, 0, sizeof(report));
    put32(report, cid);
    report[4] = seq;
    size_t cn = remain < (HID_RPT_SIZE - 5) ? remain : (HID_RPT_SIZE - 5);
    memcpy(report + 5, payload + sent, cn);
    send_report();
}

/* The retry is delivered ATOMICALLY (all reports back-to-back, no pump in
 * between - the pre-poll delivery ordering of the supported SET_REPORT
 * control-transfer path, exactly like the deterministic scenes). Splitting
 * the report stream with hid_task pumps in between would let the unwind's
 * drop+replay admit a PARTIAL message mid-assembly, whose interleaving
 * with the live continuation stream the oracle does not model; the
 * report-level interleaving of a competing channel is covered
 * deterministically by the competing_cid_during_unwind scene. */
static void rnd_send_retry(const uint8_t *payload, uint16_t len) {
    send_cbor_init_packet(CID, payload, len);
    uint16_t sent = len < (HID_RPT_SIZE - 7) ? len : (HID_RPT_SIZE - 7);
    uint8_t seq = 0;
    while (sent < len) {
        rnd_send_cont(CID, payload, len, sent, seq++);
        sent = (uint16_t) (sent + (HID_RPT_SIZE - 5));
    }
}

static bool rnd_quiescent(void) {
    return !is_busy() && !is_req_button_pending() && !exec_finished_cancelled;
}

/* CBOR response frames present in the TX window (keepalive-cancel frames
 * excluded). */
static int rnd_count_responses(int tx_start) {
    int n = 0;
    for (int i = tx_start; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0 && !(f[6] == 1 && f[7] == CTAPHID_KEEPALIVE_CANCEL_STATUS)) {
            n++;
        }
    }
    return n;
}

/* Full TX-window accounting for one iteration (see the block comment). */
static void rnd_scan_window(int tx_start, int first_marker, int p_final, bool expect_02d) {
    rnd_phase = "scan";
    int marker = first_marker, k02d = 0, errors = 0;
    for (int i = tx_start; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (f[4] == CTAPHID_CBOR && f[5] == 0) {
            if (f[6] == 1 && f[7] == CTAPHID_KEEPALIVE_CANCEL_STATUS) {
                k02d++;
                RASSERT(frame_has_cid(f, CID));
            }
            else {
                RASSERT(f[6] == 2 && f[7] == 0x00);
                RASSERT(f[8] == marker); /* responses in parse order, no dup, no gap */
                marker++;
            }
        }
        else if (f[4] == CTAPHID_ERROR) {
            errors++;
            /* CHANNEL_BUSY only for the non-owning channel; the protected
             * channel is never answered with an error. */
            RASSERT(frame_has_cid(f, RND_CID_B));
            RASSERT(f[6] == 1 && f[7] == CTAP1_ERR_CHANNEL_BUSY);
        }
        else if (f[4] == CTAPHID_INIT) {
            int hit = -1;
            if (rnd_have_resync_init && !rnd_resync_init_done && f[6] == 17 &&
                memcmp(f + 7, rnd_resync_nonce, 8) == 0) {
                hit = -2; /* the resync INIT's own response */
            }
            for (int k = 0; hit == -1 && k < rnd_b_inits; k++) {
                if (!rnd_b_nonce_done[k] && f[6] == 17 && memcmp(f + 7, rnd_b_nonce[k], 8) == 0) {
                    hit = k;
                }
            }
            RASSERT(hit != -1); /* every INIT answered exactly once, nonce echoed */
            if (hit == -2) {
                rnd_resync_init_done = true;
            }
            else if (hit >= 0) {
                rnd_b_nonce_done[hit] = true;
            }
        }
        else if (f[4] == CTAPHID_PING) {
            int hit = -1;
            for (int k = 0; k < rnd_b_pings; k++) {
                if (!rnd_b_ping_done[k] && f[6] == 4 && memcmp(f + 7, rnd_b_ping[k], 4) == 0) {
                    hit = k;
                }
            }
            RASSERT(hit >= 0); /* every PING echoed exactly once */
            if (hit >= 0) {
                rnd_b_ping_done[hit] = true;
            }
        }
        else if (f[4] == CTAPHID_KEEPALIVE) {
            /* legal: throttled processing keepalive of an admitted command */
        }
        else {
            RASSERT(0); /* stale or unaccounted frame */
        }
    }
    RASSERT(marker == p_final + 1);
    RASSERT(k02d == (expect_02d ? 1 : 0));
    /* Every request not seen above must have exactly one BUSY error. */
    int echoed = 0;
    for (int k = 0; k < rnd_b_pings; k++) {
        echoed += rnd_b_ping_done[k] ? 1 : 0;
    }
    int inited = 0;
    for (int k = 0; k < rnd_b_inits; k++) {
        inited += rnd_b_nonce_done[k] ? 1 : 0;
    }
    RASSERT(inited == rnd_b_inits);
    RASSERT(rnd_have_resync_init == rnd_resync_init_done);
    RASSERT(errors == (rnd_b_pings - echoed) + (rnd_b_cbor_sent - rnd_b_cbor_admitted));
}

static void scene_randomized(uint64_t seed, int iterations) {
    static const uint8_t old_payload[] = { 0x04, 0xaa };
    static uint8_t retry_payload[CTAP_MAX_PACKET_SIZE];
    static const uint8_t fresh_payload[] = { 0x04, 0x60 };
    enum { MODE_CANCEL, MODE_PRESYNC, MODE_PRESS, MODE_RESYNC };

    rnd_seed = seed;
    rng_state = seed * 0x9E3779B97F4A7C15ULL + 0xD1B54A32D192ED03ULL;
    printf("randomized: seed=0x%08llx iterations=%d\n", (unsigned long long) seed, iterations);
    fflush(stdout);

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    for (rnd_iter = 0; rnd_iter < iterations; rnd_iter++) {
        uint32_t roll = rnd_below(100);
        int mode = roll < 50 ? MODE_CANCEL : (roll < 65 ? MODE_PRESYNC : (roll < 85 ? MODE_PRESS : MODE_RESYNC));
        bool press_wins = mode == MODE_CANCEL && rnd_below(100) < 30;

        /* Retry of 1, 4 or 129 reports. Distinct payload per iteration. */
        uint16_t retry_len;
        uint32_t kind = rnd_below(3);
        if (kind == 0) {
            retry_len = 40; /* 1 report */
        }
        else if (kind == 1) {
            retry_len = 200; /* init packet + 3 continuations */
        }
        else {
            retry_len = CTAP_MAX_PACKET_SIZE; /* 129 reports */
        }
        for (uint16_t i = 0; i < retry_len; i++) {
            retry_payload[i] = (uint8_t) (i * 13 + 5) ^ (uint8_t) (rnd_iter * 7 + 1);
        }
        int n_b = (int) rnd_below(4); /* 0..3 other-channel ops */

        rnd_phase = "start";
        rnd_steps = 0;
        rnd_dbg_mode = mode;
        rnd_dbg_kind = (int) kind;
        rnd_dbg_nb = n_b;
        rnd_dbg_press = press_wins ? 1 : 0;
        /* Known button state for every iteration: a press left in the
         * control file would complete the next wait before the cancel. */
        write_btn("none");
        write_btn("timeout:0");
        rnd_reset_test_state();
        RASSERT(rnd_quiescent());

        int tx_start = tx_count;
        bool expect_02d = mode != MODE_PRESS && mode != MODE_RESYNC;
        int first_marker = (mode == MODE_PRESS) ? 1 : 2;

        /* Old command with a pending UP wait. */
        rnd_phase = "setup";
        mock_up_push(true);
        build_cbor(old_payload, sizeof(old_payload));
        send_report();

        if (mode != MODE_PRESYNC) {
            rnd_phase = "wait";
            bool pending = false;
            for (int r = 0; r < 100 && !pending; r++) {
                pending = is_req_button_pending();
                if (!pending) {
                    usleep(1000);
                    hid_task();
                    button_task();
                    rnd_step();
                }
            }
            RASSERT(pending);
        }

        if (mode == MODE_CANCEL || mode == MODE_PRESYNC) {
            rnd_phase = "cancel";
            build_cancel();
            send_report();
            rnd_pump_hid(2);
            int cancel_resp = find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, tx_start);
            RASSERT(cancel_resp >= 0);
            RASSERT(exec_finished_cancelled);

            if (press_wins) {
                /* The press races the cancellation delivery: the wait ends
                 * PRESSED (emul answers the press before the armed cancel),
                 * the cancelled transaction's completion is still dropped. */
                write_btn("press");
            }

            /* Retry + other-channel ops inside the unwind window (hid_task
             * only: the window stays open). No same-channel INIT here - a
             * resync discards buffered reports by design (see block
             * comment), so this scene never orphans them. */
            rnd_phase = "retry";
            mock_up_push(false);
            rnd_send_retry(retry_payload, retry_len);
            while (n_b > 0) {
                rnd_b_op(rnd_pick_b_op(), (uint32_t) n_b);
                n_b--;
                if (rnd_below(2) == 0) {
                    rnd_pump_hid(1);
                }
            }

            /* Button polls: the unwind completes (press or cancellation). */
            rnd_phase = "drain";
            int p_final = 1 + 1 + rnd_b_cbor_admitted;
            int want = p_final - first_marker + 1;
            int r = 0;
            for (; r < 200; r++) {
                if (rnd_quiescent() && rnd_count_responses(tx_start) == want) {
                    break;
                }
                usleep(1000);
                hid_task();
                button_task();
                rnd_step();
            }
            RASSERT(rnd_quiescent() && rnd_count_responses(tx_start) == want);

            rnd_pump_full(2);
            RASSERT(rnd_quiescent());
            RASSERT(worker_timed_out == 0);
            RASSERT(worker_cancelled == (press_wins ? 0 : 1));
            RASSERT(worker_pressed == (press_wins ? 1 : 0));
            RASSERT(worker_cancel_parse_count == (press_wins ? -1 : 1)); /* only the old cmd was parsed at cancellation */
            RASSERT(mock_parse_calls == p_final);
            if (press_wins) {
                write_btn("timeout:0"); /* clear the press for the next iteration */
            }
            /* The buffered retry was replayed whole and parsed exactly once. */
            RASSERT(mock_parse_cmds[1] == CTAPHID_CBOR);
            RASSERT(mock_parse_lens[1] == retry_len);
            RASSERT(memcmp(mock_parse_payload[1], retry_payload, 8) == 0);
            rnd_scan_window(tx_start, first_marker, p_final, expect_02d);
        }
        else if (mode == MODE_PRESS) {
            /* Press completes the wait normally: one framed response, no
             * 0x2D, no errors. Other-channel CBOR is excluded here: while
             * the wait is active without a cancellation, an admitted
             * second command's event is swallowed upstream (see block
             * comment) - outside the cancel path's scope. */
            rnd_phase = "ops";
            while (n_b > 0) {
                /* PING/INIT are always answered inline while the wait is
                 * active; other-channel CBOR is excluded (block comment). */
                rnd_b_op(rnd_below(100) < 50 ? RND_OP_PING : RND_OP_INIT, (uint32_t) n_b);
                n_b--;
                rnd_pump_hid(1);
            }

            rnd_phase = "press";
            write_btn("press");
            int p_final = 1;
            int want = p_final - first_marker + 1;
            int r = 0;
            for (; r < 200; r++) {
                if (rnd_quiescent() && rnd_count_responses(tx_start) == want) {
                    break;
                }
                usleep(1000);
                hid_task();
                button_task();
                rnd_step();
            }
            RASSERT(rnd_quiescent() && rnd_count_responses(tx_start) == want);

            rnd_pump_full(2);
            RASSERT(rnd_quiescent());
            RASSERT(worker_pressed == 1 && worker_cancelled == 0 && worker_timed_out == 0);
            RASSERT(worker_cancel_parse_count == -1);
            RASSERT(mock_parse_calls == 1);
            write_btn("timeout:0");
            rnd_scan_window(tx_start, first_marker, p_final, false);
        }
        else { /* MODE_RESYNC */
            /* INIT on the busy channel aborts the wait non-blockingly: the
             * INIT response goes out, the cancellation is delivered on the
             * normal path, no 0x2D is fabricated, and a fresh command on
             * the resynced channel is answered exactly once. B ops are
             * safe in both windows (PING/INIT answered inline; a CBOR
             * inside the unwind window answered BUSY by the gate; a CBOR
             * on an idle channel admitted and processed). */
            rnd_phase = "ops";
            while (n_b > 0) {
                rnd_b_op(rnd_pick_b_op(), (uint32_t) n_b);
                n_b--;
                rnd_pump_hid(1);
            }

            rnd_phase = "resync";
            static const uint8_t nonce[8] = { 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77 };
            memcpy(rnd_resync_nonce, nonce, 8);
            rnd_have_resync_init = true;
            send_init_packet(CID, nonce);
            rnd_pump_hid(2);
            int init_resp = find_init_response(CID, nonce, tx_start);
            RASSERT(init_resp >= 0);
            RASSERT(find_cbor_status_frame(CTAPHID_KEEPALIVE_CANCEL_STATUS, tx_start) < 0);

            rnd_phase = "unwind";
            int r = 0;
            for (; r < 200; r++) {
                if (worker_cancelled == 1 && !exec_finished_cancelled) {
                    break;
                }
                usleep(1000);
                hid_task();
                button_task();
                rnd_step();
            }
            RASSERT(worker_cancelled == 1 && !exec_finished_cancelled);
            RASSERT(worker_cancel_parse_count == 1);
            RASSERT(worker_pressed == 0 && worker_timed_out == 0);

            /* Fresh command on the resynced channel: admitted immediately
             * now that the unwind completed (marker clear, worker free). */
            mock_up_push(false);
            build_cbor(fresh_payload, sizeof(fresh_payload));
            send_report();
            RASSERT(mock_parse_calls == 2);

            rnd_phase = "drain";
            int p_final = 2 + rnd_b_cbor_admitted;
            int want = p_final - first_marker + 1;
            for (r = 0; r < 200; r++) {
                if (rnd_quiescent() && rnd_count_responses(tx_start) == want) {
                    break;
                }
                usleep(1000);
                hid_task();
                button_task();
                rnd_step();
            }
            RASSERT(rnd_quiescent() && rnd_count_responses(tx_start) == want);

            rnd_pump_full(2);
            RASSERT(rnd_quiescent());
            RASSERT(mock_parse_calls == p_final);
            RASSERT(mock_parse_cmds[1] == CTAPHID_CBOR);
            RASSERT(mock_parse_payload[1][1] == 0x60);
            rnd_scan_window(tx_start, first_marker, p_final, false);
        }

        /* No stale work anywhere: queues drained, state machine idle. */
        RASSERT(usb_to_card_q.num_elem == 0 && card_to_usb_q.num_elem == 0);
    }
    printf("randomized: seed=0x%08llx all %d iterations passed\n", (unsigned long long) seed, iterations);
    fflush(stdout);
}

/* Randomized-scene finding (seed 0x5EED0001, iteration 2): the CBOR
 * response is built at completion time from ctap_req->cid and last_cmd
 * (driver_exec_finished_cont_hid), but any inline traffic from another
 * channel between admission and completion - a PING echo is always
 * answerable - advances those globals. The completion is then delivered
 * as a bogus frame addressed to the WRONG channel with the WRONG command
 * byte, and the owning channel never receives its response. Deterministic
 * repro: channel A's CBOR is admitted with a pending UP wait, channel B's
 * one-report PING is echoed in between, then A's wait is pressed: the
 * completion must be answered as A's CBOR response, and B must receive
 * nothing but its PING echo. */
static void scene_ping_echo_during_completion(void) {
    static const uint8_t payload[] = { 0x04, 0xaa };
    static const uint8_t ping_payload[] = { 0x50, 0x51, 0x52, 0x53 };

    write_btn("none");
    write_btn("timeout:0");
    channel_setup();

    mock_up_push(true);
    build_cbor(payload, sizeof(payload));
    send_report();
    assert(spin_until(is_req_button_pending, 2000));

    /* Legal competing-channel traffic while A's command is in flight. */
    send_ping_packet(CID_B, ping_payload, sizeof(ping_payload));
    for (unsigned waited = 0; waited < 2000 && find_ping_echo(CID_B, ping_payload, sizeof(ping_payload), 0) < 0; waited += 2) {
        usleep(2000);
        hid_task();
        button_task();
    }
    assert(find_ping_echo(CID_B, ping_payload, sizeof(ping_payload), 0) >= 0);

    /* A's wait completes normally: the response must be addressed to A. */
    write_btn("press");
    bool response_seen = false;
    for (unsigned waited = 0; waited < 4000 && !response_seen; waited += 2) {
        for (int i = 0; i < tx_count; i++) {
            const uint8_t *f = tx_frames[i];
            if (f[4] == CTAPHID_CBOR && f[5] == 0 && f[6] == 2 && f[7] == 0x00 && f[8] == 1 &&
                frame_has_cid(f, CID)) {
                response_seen = true;
            }
        }
        if (!response_seen) {
            usleep(2000);
            hid_task();
            button_task();
        }
    }
    assert(response_seen);
    assert(mock_parse_calls == 1);
    assert(worker_pressed == 1 && worker_cancelled == 0 && worker_timed_out == 0);
    /* B received nothing but its PING echo: no stale/corrupted frame. */
    for (int i = 0; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        if (frame_has_cid(f, CID_B)) {
            assert(f[4] == CTAPHID_PING && f[6] == (uint8_t) sizeof(ping_payload) &&
                   memcmp(f + 7, ping_payload, sizeof(ping_payload)) == 0);
        }
    }
    /* No PING-cmd frame carrying a 2-byte CBOR-shaped body (the corruption
     * signature of a completion delivered with stale identity). */
    for (int i = 0; i < tx_count; i++) {
        const uint8_t *f = tx_frames[i];
        assert(!(f[4] == CTAPHID_PING && f[6] == 2));
    }
    assert(!is_req_button_pending() && !is_busy());
    printf("ping_echo_during_completion: a competing-channel PING echo between admission and "
           "completion does not corrupt the response identity\n");
}

int main(int argc, char **argv) {
    assert(argc == 2);
    /* Hard step budget: a deadlock (e.g. a blocking handshake while a UP
     * wait discards events) must fail the scene, not hang ctest. */
    alarm(30);
    env_setup();
    usb_init();
    if (strcmp(argv[1], "fast_retry") == 0) {
        scene_fast_retry();
    }
    else if (strcmp(argv[1], "fast_retry_fragmented") == 0) {
        scene_fast_retry_fragmented();
    }
    else if (strcmp(argv[1], "fast_retry_max_size") == 0) {
        scene_fast_retry_max_size();
    }
    else if (strcmp(argv[1], "cancel_before_wait_start") == 0) {
        scene_cancel_before_wait_start();
    }
    else if (strcmp(argv[1], "settled_retry") == 0) {
        scene_settled_retry();
    }
    else if (strcmp(argv[1], "no_retry_after_cancel") == 0) {
        scene_no_retry_after_cancel();
    }
    else if (strcmp(argv[1], "press_control") == 0) {
        scene_press_control();
    }
    else if (strcmp(argv[1], "competing_cid_during_unwind") == 0) {
        scene_competing_cid_during_unwind();
    }
    else if (strcmp(argv[1], "init_after_cancel_active_wait") == 0) {
        scene_init_after_cancel_active_wait();
    }
    else if (strcmp(argv[1], "init_during_active_wait") == 0) {
        scene_init_during_active_wait();
    }
    else if (strcmp(argv[1], "ping_echo_during_completion") == 0) {
        scene_ping_echo_during_completion();
    }
    else if (strcmp(argv[1], "randomized_a") == 0) {
        scene_randomized(0x5EED0001ull, 800);
    }
    else if (strcmp(argv[1], "randomized_b") == 0) {
        scene_randomized(0x5EED0002ull, 800);
    }
    else if (strcmp(argv[1], "randomized_c") == 0) {
        scene_randomized(0x5EED0003ull, 800);
    }
    else {
        return 2;
    }
    return 0;
}
