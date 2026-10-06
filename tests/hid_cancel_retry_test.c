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
    snprintf(btn_file, sizeof(btn_file), "/tmp/picokeys_hid_cancel_retry_test.cmd");
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
    else {
        return 2;
    }
    return 0;
}
