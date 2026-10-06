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

/* Captured TX: every 64-byte report the production code submits. */
#define MAX_TX_FRAMES 32
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

int main(int argc, char **argv) {
    assert(argc == 2);
    env_setup();
    usb_init();
    if (strcmp(argv[1], "fast_retry") == 0) {
        scene_fast_retry();
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
    else {
        return 2;
    }
    return 0;
}
