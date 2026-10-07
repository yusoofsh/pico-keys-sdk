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

#include <stdio.h>
#include "picokeys.h"
#include "serial.h"
#include "pico_time.h"
#ifndef ENABLE_EMULATION
#include "tusb.h"
#if defined(ESP_PLATFORM)
static portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
#endif
#else
#include "emulation.h"
#endif
#include "ctap_hid.h"
#include "kb_tx.h"
#include "picokeys_version.h"
#include "apdu.h"
#include "usb.h"
#include "button.h"

extern void init_fido(void);
bool is_nk = false;
uint8_t (*get_version_major)(void) = NULL;
uint8_t (*get_version_minor)(void) = NULL;

#define CTAPHID_KEEPALIVE_CANCEL_STATUS 0x2D

static usb_buffer_t *hid_rx = NULL, *hid_tx = NULL;

PACK(
typedef struct msg_packet {
    uint16_t len;
    uint16_t current_len;
    uint8_t data[CTAP_MAX_PACKET_SIZE];
}) msg_packet_t;

msg_packet_t msg_packet = { 0 };

static uint16_t *send_buffer_size = NULL;
static write_status_t *last_write_result = NULL;

CTAPHID_FRAME *ctap_req = NULL, *ctap_resp = NULL;
static void send_keepalive(void);
static int ctap_error_cid(uint32_t cid, uint8_t error);
int driver_process_usb_packet_hid(uint16_t read);
int driver_write_hid(uint8_t itf, const_byte_array_t buffer);
static int driver_process_usb_nopacket_hid(void);
void hid_init(void);
void hid_task(void);
#ifdef ENABLE_EMULATION
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen);
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize);
#endif

/* Transaction state of the HID transport: IDLE (no command in flight),
 * BUSY (a command is admitted to the worker; the TX ring holds its pending
 * response data from the worker's completion write until the delivery) and
 * UNWINDING (a CANCEL/INIT resync is aborting it). Response identity of the
 * in-flight command, snapshotted when the command is admitted (handed to
 * the worker). At completion time, legal inline traffic from other channels
 * - a PING echo, an INIT allocation - has advanced ctap_req/last_cmd, so
 * the completion must be addressed from the admission-time identity:
 * building it from the current globals would deliver the response to the
 * wrong channel with the wrong command byte, and the owning channel would
 * never see its answer. */
#define HID_TXN_IDLE      0u
#define HID_TXN_BUSY      1u
#define HID_TXN_UNWINDING 2u
static uint8_t hid_txn_state = HID_TXN_IDLE;
/* The channel of the BUSY/UNWINDING transaction; while unwinding it is the
 * only channel whose reports the ring buffers. */
static uint32_t hid_txn_cid = 0;
static uint32_t resp_cid = 0;
static uint8_t resp_cmd = 0;

void hid_init(void) {
    if (ITF_HID_TOTAL == 0) {
        return;
    }
    if (send_buffer_size == NULL) {
        send_buffer_size = (uint16_t *)calloc(ITF_HID_TOTAL, sizeof(uint16_t));
    }
    if (last_write_result == NULL) {
        last_write_result = (write_status_t *)calloc(ITF_HID_TOTAL, sizeof(write_status_t));
    }
    if (hid_rx == NULL) {
        hid_rx = (usb_buffer_t *)calloc(ITF_HID_TOTAL, sizeof(usb_buffer_t));
    }
    if (hid_tx == NULL) {
        hid_tx = (usb_buffer_t *)calloc(ITF_HID_TOTAL, sizeof(usb_buffer_t));
    }
#ifndef ENABLE_EMULATION
    kb_tx_init(&kb_tx_ops);
#endif
}

int driver_init_hid(void) {
#ifndef ENABLE_EMULATION
    static bool _init = false;
    if (_init == false) {
#if defined(ESP_PLATFORM)
        const tusb_rhport_init_t rh_init = {
            .role = TUSB_ROLE_DEVICE,
            .speed = TUSB_SPEED_AUTO,
        };

        tusb_init(BOARD_TUD_RHPORT, &rh_init);
#else
        tud_init(BOARD_TUD_RHPORT);
#endif
        _init = true;
    }
#endif
    ctap_req = (CTAPHID_FRAME *) (hid_rx[ITF_HID_CTAP].buffer + hid_rx[ITF_HID_CTAP].r_ptr);
    apdu.header = ctap_req->init.data;

    ctap_resp = (CTAPHID_FRAME *) (hid_tx[ITF_HID_CTAP].buffer);
    apdu.rdata = ctap_resp->init.data;
    usb_set_timeout_counter(ITF_HID, 200);

    is_nk = false;

    /* A received packet must not clobber a response that is in flight:
     * inline traffic (another channel's PING echo or INIT allocation, and
     * pre-admission error answers) is dispatched from this path while the
     * TX ring holds the in-flight transaction's pending response data,
     * which the worker writes at completion time. The CANCEL and
     * INIT-resync handlers reset the ring themselves when they abort a
     * transaction. hid_txn_sync() has already run for this packet, so the
     * state is current. */
    if (hid_txn_state != HID_TXN_BUSY) {
        memset(ctap_resp, 0, sizeof(CTAPHID_FRAME));
        hid_tx[ITF_HID_CTAP].w_ptr = hid_tx[ITF_HID_CTAP].r_ptr = 0;
        send_buffer_size[ITF_HID_CTAP] = 0;
    }
    return 0;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

uint16_t (*hid_get_report_cb)(uint8_t, uint8_t, hid_report_type_t, uint8_t *, uint16_t) = NULL;
// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    // TODO not Implemented
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    printf("get_report %d %d %d\n", itf, report_id, report_type);
    memset(buffer, 0, reqlen);
    DEBUG_PAYLOAD(buffer, reqlen);
    if (hid_get_report_cb) {
        hid_get_report_cb(itf, report_id, report_type, buffer, reqlen);
    }
    return reqlen;
}

static uint32_t hid_write_offset(uint16_t size, uint16_t offset) {
    if (hid_tx[ITF_HID_CTAP].buffer[offset] != 0x81) {
        DEBUG_PAYLOAD(&hid_tx[ITF_HID_CTAP].buffer[offset], size);
    }
    hid_tx[ITF_HID_CTAP].w_ptr += size + offset;
    hid_tx[ITF_HID_CTAP].r_ptr += offset;
    return size;
}

static uint32_t hid_write(uint16_t size) {
    return hid_write_offset(size, 0);
}

#ifndef ENABLE_EMULATION
static const uint8_t conv_table[128][2] =  { HID_ASCII_TO_KEYCODE };

/* Transport ops of the single-owner keyboard transmitter (kb_tx.h). The
 * keyboard send path gates on the keyboard HID instance,
 * tud_hid_n_ready(ITF_HID_KB) — never on the generic tud_hid_ready(),
 * which checks instance 0 (the CTAP HID interface). */
static bool kb_tx_ready(void) {
    return usb_kb_itf_enabled() && tud_hid_n_ready(ITF_HID_KB);
}
static bool kb_tx_send(uint8_t modifier, const uint8_t *keycodes) {
    return tud_hid_n_keyboard_report(ITF_HID_KB, REPORT_ID_KEYBOARD, modifier, keycodes);
}
static void kb_tx_lookup(uint8_t ascii, uint8_t *modifier, uint8_t *keycode) {
    /* conv_table covers ASCII only; a byte >= 128 in an encoded buffer
     * types nothing rather than reading past the table. */
    if (ascii < 128) {
        if (conv_table[ascii][0]) {
            *modifier = KEYBOARD_MODIFIER_LEFTSHIFT;
        }
        *keycode = conv_table[ascii][1];
    }
}
static const kb_tx_ops_t kb_tx_ops = {
    .ready = kb_tx_ready,
    .send = kb_tx_send,
    .lookup = kb_tx_lookup,
};

bool add_keyboard_buffer(const_byte_array_t data, bool encode) {
    /* Legacy OTP entry point: typed as the OTP owner of the transmitter. */
    return kb_tx_add_buffer(KB_TX_OWNER_OTP, data.data, data.len, encode);
}

bool append_keyboard_buffer(const_byte_array_t data) {
    return kb_tx_append_buffer(KB_TX_OWNER_OTP, data.data, data.len);
}

static void send_hid_report(uint8_t report_id) {
    (void)report_id;
    /* All keyboard output flows through the single-owner transmitter: its
     * injected ops gate on tud_hid_n_ready(ITF_HID_KB), and every key-down
     * it sent is followed by an all-released report. */
    kb_tx_task();
}
#endif

bool usb_kb_itf_enabled(void) {
    return ITF_HID_KB != ITF_INVALID;
}

#ifndef ENABLE_EMULATION
bool usb_kb_mounted(void) {
    return tud_mounted();
}

bool usb_kb_suspended(void) {
    return tud_suspended();
}
#endif

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {
    //printf("report_complete %d %d %d\n", instance, len, send_buffer_size[instance]);
    if (instance == ITF_HID_CTAP && len == 64) {
#ifdef ESP_PLATFORM
        taskENTER_CRITICAL(&mutex);
#endif
        CTAPHID_FRAME *req = (CTAPHID_FRAME *) report;
        if (last_write_result[instance] == WRITE_PENDING) {
            last_write_result[instance] = WRITE_SUCCESS;
            if (FRAME_TYPE(req) == TYPE_INIT) {
                if (req->init.cmd != CTAPHID_KEEPALIVE) {
                    send_buffer_size[instance] -= MIN(64 - 7, send_buffer_size[instance]);
                }
            }
            else {
                send_buffer_size[instance] -= MIN(64 - 5, send_buffer_size[instance]);
            }
        }
        if (last_write_result[instance] == WRITE_SUCCESS) {
            if (FRAME_TYPE(req) != TYPE_INIT || req->init.cmd != CTAPHID_KEEPALIVE) {
                if (send_buffer_size[instance] > 0) {
                    ctap_resp = (CTAPHID_FRAME *) ((uint8_t *) ctap_resp + 64 - 5);
                    uint8_t seq = FRAME_TYPE(req) == TYPE_INIT ? 0 : FRAME_SEQ(req) + 1;
                    ctap_resp->cid = req->cid;
                    ctap_resp->cont.seq = seq;

                    hid_tx[ITF_HID_CTAP].r_ptr += 64 - 5;
                }
                else {
                    hid_tx[ITF_HID_CTAP].r_ptr += 64;
                }
            }
        }
        if (hid_tx[ITF_HID_CTAP].r_ptr >= hid_tx[ITF_HID_CTAP].w_ptr) {
            hid_tx[ITF_HID_CTAP].r_ptr = hid_tx[ITF_HID_CTAP].w_ptr = 0;
        }
#ifdef ESP_PLATFORM
        taskEXIT_CRITICAL(&mutex);
#endif
    }
}

int driver_write_hid(uint8_t itf, const_byte_array_t buffer) {
    if ((!buffer.data && buffer.len > 0) || buffer.len > UINT16_MAX) {
        return 0;
    }
    uint16_t buffer_len = (uint16_t)buffer.len;
    if (last_write_result[itf] == WRITE_PENDING) {
        return 0;
    }
    bool r = tud_hid_n_report(itf, 0, buffer.data, buffer_len);
    last_write_result[itf] = r ? WRITE_PENDING : WRITE_FAILED;
    if (last_write_result[itf] == WRITE_FAILED) {
        return 0;
    }
#ifdef ENABLE_EMULATION
    tud_hid_report_complete_cb(ITF_HID_CTAP, buffer.data, buffer_len);
#endif
    return buffer_len > 64 ? 64 : buffer_len;
}

int (*hid_set_report_cb)(uint8_t, uint8_t, hid_report_type_t, uint8_t const *, uint16_t) = NULL;
// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )

/* HID-side CTAPHID transaction state machine and the cancellation unwind.
 *
 * States: IDLE (no transaction), BUSY(cid) (a worker command of channel
 * `cid` was admitted, its response timeout armed), UNWINDING(cid) (the
 * transaction was aborted - by CTAPHID_CANCEL or by a same-channel
 * CTAPHID_INIT resync - and is being torn down). hid_txn_sync() keeps the
 * state in step with the transport signals: the response timeout
 * disarming ends BUSY, the exec_finished_cancelled marker being consumed
 * ends UNWINDING.
 *
 * CTAPHID_CANCEL stops the response timeout and marks the aborted
 * transaction (exec_finished_cancelled), but the old UP wait is only
 * cancelled when button_task() next polls it (10 ms gate). Admitting a new
 * packet in that window would race the old wait: usb_send_event()
 * enqueues EV_CMD_AVAILABLE without awaiting acknowledgement, and the
 * still-pending wait (pico-fido fido.c wait_button_pressed loop) removes
 * and discards every non-button event - the new request would never be
 * processed. While the marker is set, arriving packets are therefore
 * buffered in the ring below and replayed from hid_task() once the
 * cancellation has been delivered and the aborted transaction's late
 * completion has been consumed and dropped by card_status().
 *
 * The ring belongs to the UNWINDING channel only. It is sized for ONE
 * complete maximum-size CTAPHID message of that channel (one init packet
 * plus 128 continuations, ctap_hid.h; makeCredential retries are commonly
 * several reports), and a report of any other channel is answered with
 * CTAP1_ERR_CHANNEL_BUSY immediately, without consuming a ring slot or
 * disturbing the buffered message. A report that does not fit - a second
 * message on the unwinding channel - is answered the same way instead of
 * being dropped silently.
 *
 * CTAPHID_INIT resynchronization must never block on the worker: the UP
 * wait removes and discards every non-button event, so card_exit()'s
 * blocking EV_EXIT handshake would never be acknowledged while a wait is
 * active (or committed to start) and core0 would deadlock. On the BUSY or
 * UNWINDING channel, INIT therefore aborts non-blockingly: the buffered
 * reports of the dead session are discarded, the cancellation is armed for
 * the active wait (cancel_button; the marker is also set when no
 * CTAPHID_CANCEL was seen, so the abort's late completion is dropped and
 * the admission gate stays closed until the unwind completes), and the
 * INIT response is written immediately. The unwind then completes on the
 * normal paths - button_task delivers the cancellation, card_status()
 * drops the late completion. INIT of a different channel only allocates
 * that channel and touches neither the in-flight transaction, the unwind
 * nor the ring. Only in IDLE does INIT run the classic card_exit()
 * teardown.
 *
 * Sizing the ring per protected message was chosen over the two
 * alternatives:
 * - Non-dropping backpressure (leaving the OUT endpoint un-armed so the
 *   host NAKs, or not reading the emulation socket) cannot be applied
 *   here without modifying third-party code: the control SET_REPORT data
 *   phase is consumed inside TinyUSB before this callback runs, and the
 *   OUT endpoint is re-armed inside TinyUSB's device task.
 * - Delivering the cancellation synchronously from the CANCEL branch
 *   would have to push a button event while the old UP wait may not even
 *   have started yet (its EV_PRESS_BUTTON may still be queued), and the
 *   worker's command loop would then read that stray event as a command.
 *   Deferral leaves the queue protocol untouched and keeps the
 *   cancellation delivery on the path that already owns it
 *   (button_task / wait start). */
#define HID_DEFERRED_MAX (1 + (CTAP_MAX_PACKET_SIZE - (HID_RPT_SIZE - 7) + (HID_RPT_SIZE - 5) - 1) / (HID_RPT_SIZE - 5))
static uint8_t deferred_reports[HID_DEFERRED_MAX][HID_RPT_SIZE];
static unsigned deferred_head = 0, deferred_tail = 0, deferred_count = 0;

/* Transaction states, see the comment above the ring. */
#define HID_TXN_IDLE       0u
#define HID_TXN_BUSY       1u

static void hid_deferred_flush(void) {
    deferred_head = deferred_tail = deferred_count = 0;
}

#ifdef HID_CANCEL_TEST_HOOKS
/* Host-harness visibility into the transaction state machine (the test
 * suite compiles hid.c with HID_CANCEL_TEST_HOOKS to print this state in
 * its failure dumps). */
void hid_cancel_test_state(uint8_t *txn_state, uint32_t *txn_cid, unsigned *def_count, bool *cancel_marker) {
    *txn_state = hid_txn_state;
    *txn_cid = hid_txn_cid;
    *def_count = deferred_count;
    *cancel_marker = cancel_button;
}
#endif

static void hid_txn_sync(void) {
    if (hid_txn_state == HID_TXN_BUSY && !is_busy()) {
        hid_txn_state = HID_TXN_IDLE;
    }
    else if (hid_txn_state == HID_TXN_UNWINDING && !exec_finished_cancelled) {
        hid_txn_state = HID_TXN_IDLE;
        hid_txn_cid = 0;
    }
}

static void hid_txn_start(uint32_t cid) {
    hid_txn_state = HID_TXN_BUSY;
    hid_txn_cid = cid;
}

/* Record the unwind of transaction `cid`. The caller arms the
 * cancellation: cancel_button for the wait, the exec_finished_cancelled
 * marker for the late completion and the admission gate, timeout_stop()
 * for the response timeout. */
static void hid_txn_unwind_begin(uint32_t cid) {
    hid_txn_state = HID_TXN_UNWINDING;
    hid_txn_cid = cid;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    // This example doesn't use multiple report and report ID
    (void) itf;
    (void) report_id;
    (void) report_type;
    printf("set_report %d %d %d\n", itf, report_id, report_type);
    if (!hid_set_report_cb || hid_set_report_cb(itf, report_id, report_type, buffer, bufsize) == 0) {
        //usb_rx(itf, buffer, bufsize);
        if (itf == ITF_HID_CTAP) {
            if (bufsize != HID_RPT_SIZE) {
                return;
            }
            if (exec_finished_cancelled) {
                /* Unwind window: the ring belongs to the unwinding channel
                 * (hid_txn_cid), see the comment above the ring. */
                CTAPHID_FRAME const *frame = (CTAPHID_FRAME const *) buffer;
                if (FRAME_TYPE(frame) == TYPE_INIT &&
                    (frame->init.cmd == CTAPHID_CANCEL || frame->init.cmd == CTAPHID_INIT)) {
                    /* CANCEL is harmless while unwinding and INIT is the
                     * resync path: both are handled by the normal dispatch
                     * below and never touch the ring. */
                }
                else if (frame->cid != hid_txn_cid) {
                    /* A competing channel must not consume the capacity
                     * reserved for the protected retry: answer it without
                     * buffering and without disturbing the buffered
                     * message. The raw frame's CID is used because ctap_req
                     * still holds the last processed report here. */
                    ctap_error_cid(frame->cid, CTAP1_ERR_CHANNEL_BUSY);
                    return;
                }
                else if (deferred_count == HID_DEFERRED_MAX) {
                    /* By construction one complete maximum-size message of
                     * the unwinding channel always fits; a full ring is a
                     * second message on that channel. Answer instead of
                     * dropping silently. */
                    ctap_error_cid(frame->cid, CTAP1_ERR_CHANNEL_BUSY);
                    return;
                }
                else {
                    memcpy(deferred_reports[deferred_head], buffer, HID_RPT_SIZE);
                    deferred_head = (deferred_head + 1) % HID_DEFERRED_MAX;
                    deferred_count++;
                    return;
                }
            }
            memcpy(hid_rx[itf].buffer + hid_rx[itf].w_ptr, buffer, bufsize);
            hid_rx[itf].w_ptr += bufsize;
            int proc_pkt = driver_process_usb_packet_hid(bufsize);
            if (proc_pkt == 0) {
                driver_process_usb_nopacket_hid();
            }
        }
    }
}

uint32_t last_cmd_time = 0, last_packet_time = 0;
/* CTAPHID_ERROR with an explicit channel: the admission gate answers raw
 * reports before they are copied into ctap_req, so the stale ctap_req->cid
 * must not be used there. The frame is built in the tail buffer and
 * transmitted directly (like send_keepalive): the TX ring may already hold
 * a pending frame (e.g. the fabricated keepalive-cancel waiting for the
 * next hid_task flush), and driver_init_hid() resets the ring pointers per
 * received packet - a ring-queued error could be wiped before it is ever
 * flushed. */
static int ctap_error_cid(uint32_t cid, uint8_t error) {
    CTAPHID_FRAME *resp = (CTAPHID_FRAME *) (hid_tx[ITF_HID_CTAP].buffer + sizeof(hid_tx[ITF_HID_CTAP].buffer) - 64);
    memset((uint8_t *)resp, 0, sizeof(CTAPHID_FRAME));
    resp->cid = cid;
    resp->init.cmd = CTAPHID_ERROR;
    resp->init.bcntl = 1;
    resp->init.data[0] = error;
    driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)resp, 64));
    last_packet_time = 0;
    return 0;
}

int ctap_error(uint8_t error) {
    return ctap_error_cid(ctap_req->cid, error);
}

uint8_t last_cmd = 0;
uint8_t last_seq = 0;
CTAPHID_FRAME last_req = { 0 };
uint32_t lock = 0;
uint32_t lock_cid = 0;
static uint32_t next_cid = 1;

static uint32_t allocate_cid(void) {
    uint32_t cid;
    do {
        cid = next_cid++;
        if (next_cid == 0 || next_cid == CID_BROADCAST) {
            next_cid = 1;
        }
    } while (cid == 0 || cid == CID_BROADCAST);
    return cid;
}

uint8_t thread_type = 0; //1 is APDU, 2 is CBOR
extern int cbor_process(uint8_t last_cmd, const uint8_t *data, size_t len);
static uint32_t last_keepalive_time = 0;

int driver_process_usb_nopacket_hid(void) {
    if (last_packet_time > 0 && last_packet_time + 500 < board_millis()) {
        ctap_error(CTAP1_ERR_MSG_TIMEOUT);
        last_packet_time = 0;
        msg_packet.len = msg_packet.current_len = 0;
    }
    return 0;
}

extern const uint8_t fido_aid[], u2f_aid[], oath_aid[];
extern void *cbor_thread(void *);

uint16_t *get_send_buffer_size(uint8_t itf) {
    return &send_buffer_size[itf];
}

int driver_process_usb_packet_hid(uint16_t read) {
    int apdu_sent = 0;
    hid_txn_sync();
    if (read == HID_RPT_SIZE) {
        driver_init_hid();

        hid_rx[ITF_HID_CTAP].r_ptr += HID_RPT_SIZE;
        if (hid_rx[ITF_HID_CTAP].r_ptr >= hid_rx[ITF_HID_CTAP].w_ptr) {
            hid_rx[ITF_HID_CTAP].r_ptr = hid_rx[ITF_HID_CTAP].w_ptr = 0;
        }
        last_packet_time = board_millis();
        DEBUG_PAYLOAD((uint8_t *)ctap_req, HID_RPT_SIZE);
        if (FRAME_TYPE(ctap_req) == TYPE_CONT && msg_packet.len == 0) {
            last_packet_time = 0;
            return 0;
        }
        if (ctap_req->cid == 0x0 ||
            (ctap_req->cid == CID_BROADCAST && (FRAME_TYPE(ctap_req) != TYPE_INIT || ctap_req->init.cmd != CTAPHID_INIT))) {
            return ctap_error(CTAP1_ERR_INVALID_CHANNEL);
        }
        if (board_millis() < lock && ctap_req->cid != lock_cid &&
            !(ctap_req->cid == CID_BROADCAST && ctap_req->init.cmd == CTAPHID_INIT)) {
            return ctap_error(CTAP1_ERR_CHANNEL_BUSY);
        }
        if (FRAME_TYPE(ctap_req) == TYPE_INIT && ctap_req->init.cmd == CTAPHID_CANCEL) {
            bool active_transaction = is_busy();
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
            cancel_button = true;
            res_APDU_size = 0;
            hid_tx[ITF_HID_CTAP].r_ptr = hid_tx[ITF_HID_CTAP].w_ptr = 0;
            send_buffer_size[ITF_HID_CTAP] = 0;
            if (active_transaction && last_cmd == CTAPHID_CBOR) {
                finished_data_size = 0;
                apdu.sw = 0;
                apdu.rlen = 0;
                memset((uint8_t *)ctap_resp, 0, sizeof(CTAPHID_FRAME));
                ctap_resp->cid = ctap_req->cid;
                ctap_resp->init.cmd = CTAPHID_CBOR;
                ctap_resp->init.bcntl = 1;
                ctap_resp->init.data[0] = CTAPHID_KEEPALIVE_CANCEL_STATUS;
                hid_write(64);
                timeout_stop();
                /* The card thread unwinds the cancelled request and queues
                 * its own EV_EXEC_FINISHED afterwards. card_status() drops
                 * that late completion instead of delivering its frame. */
                exec_finished_cancelled = true;
                hid_txn_unwind_begin(ctap_req->cid);
            }
            return 0;
        }
        if (FRAME_TYPE(ctap_req) == TYPE_INIT) {
            if (MSG_LEN(ctap_req) > CTAP_MAX_PACKET_SIZE) {
                return ctap_error(CTAP1_ERR_INVALID_LEN);
            }
            if (msg_packet.len > 0 && last_cmd_time + 100 > board_millis() &&
                ctap_req->init.cmd != CTAPHID_INIT) {
                if (last_req.cid != ctap_req->cid) { //We are in a transaction
                    return ctap_error(CTAP1_ERR_CHANNEL_BUSY);
                }
                else {
                    return ctap_error(CTAP1_ERR_INVALID_SEQ);
                }
            }
            printf("command %x\n", FRAME_CMD(ctap_req));
            printf("len %d\n", MSG_LEN(ctap_req));
            msg_packet.len = msg_packet.current_len = 0;
            if (MSG_LEN(ctap_req) > 64 - 7) {
                msg_packet.len = MSG_LEN(ctap_req);
                memcpy(msg_packet.data + msg_packet.current_len, ctap_req->init.data, 64 - 7);
                msg_packet.current_len += 64 - 7;
            }
            memcpy(&last_req, ctap_req, sizeof(CTAPHID_FRAME));
            last_cmd = ctap_req->init.cmd;
            last_seq = 0;
            last_cmd_time = board_millis();
        }
        else {
            if (last_seq != ctap_req->cont.seq) {
                return ctap_error(CTAP1_ERR_INVALID_SEQ);
            }
            if (last_req.cid == ctap_req->cid) {
                memcpy(msg_packet.data + msg_packet.current_len, ctap_req->cont.data,
                       MIN(64 - 5, msg_packet.len - msg_packet.current_len));
                msg_packet.current_len += MIN(64 - 5, msg_packet.len - msg_packet.current_len);
                memcpy(&last_req, ctap_req, sizeof(CTAPHID_FRAME));
                last_seq++;
            }
            else if (last_cmd_time + 100 > board_millis()) {
                return ctap_error(CTAP1_ERR_CHANNEL_BUSY);
            }
        }
        if (ctap_req->init.cmd == CTAPHID_INIT) {
            hid_txn_sync();
            bool resync_owner = false;
            if (hid_txn_state == HID_TXN_IDLE) {
                card_exit();
                /* The resync dropped the aborted session: buffered packets
                 * belong to it and are discarded with the drained queues. */
                hid_deferred_flush();
                resync_owner = true;
            }
            else if (ctap_req->cid == hid_txn_cid) {
                /* Non-blocking abort of this channel's transaction (see the
                 * comment above the ring): the UP wait - active or about to
                 * start - discards every non-button event, so the blocking
                 * EV_EXIT handshake of card_exit() would deadlock core0.
                 * Arm the cancellation for the wait, mark the transaction's
                 * late completion for dropping (also when no CTAPHID_CANCEL
                 * was seen), discard the now-obsolete buffered reports and
                 * answer INIT immediately; the unwind completes on the
                 * normal paths (button_task delivers the cancellation,
                 * card_status() drops the late completion). */
                cancel_button = true;
                timeout_stop();
                if (!exec_finished_cancelled) {
                    exec_finished_cancelled = true;
                }
                hid_txn_unwind_begin(ctap_req->cid);
                hid_deferred_flush();
                send_buffer_size[ITF_HID_CTAP] = 0;
                /* A report of the aborted session may still be in flight on
                 * the USB stack; its completion callback would otherwise
                 * advance the reset ring and swallow the INIT response
                 * written below. Invalidate the write credit so the stale
                 * completion is ignored. */
                last_write_result[ITF_HID_CTAP] = WRITE_FAILED;
                resync_owner = true;
            }
            /* else: an INIT of a different channel while a transaction of
             * hid_txn_cid is in flight only allocates that channel - it is
             * answered below without the blocking handshake and without
             * touching the transaction, the unwind or the ring. */
            if (resync_owner) {
                hid_tx[ITF_HID_CTAP].r_ptr = hid_tx[ITF_HID_CTAP].w_ptr = 0;
                init_fido();
            }
            CTAPHID_INIT_REQ *req = (CTAPHID_INIT_REQ *) ctap_req->init.data;
            /* Inline answer in the tail frame (see send_keepalive): the
             * allocation of another channel happens while a transaction
             * may be in flight and must not overwrite the TX ring's
             * pending response data. */
            CTAPHID_FRAME *init_tx = (CTAPHID_FRAME *) (hid_tx[ITF_HID_CTAP].buffer + sizeof(hid_tx[ITF_HID_CTAP].buffer) - 64);
            memset((uint8_t *) init_tx, 0, sizeof(CTAPHID_FRAME));
            CTAPHID_INIT_RESP *resp = (CTAPHID_INIT_RESP *) init_tx->init.data;
            memcpy(resp->nonce, req->nonce, sizeof(resp->nonce));
            resp->cid = ctap_req->cid == CID_BROADCAST ? allocate_cid() : ctap_req->cid;
            resp->versionInterface = CTAPHID_IF_VERSION;
            resp->versionMajor = get_version_major ? get_version_major() : PICOKEYS_SDK_VERSION_MAJOR;
            resp->versionMinor = get_version_minor ? get_version_minor() : PICOKEYS_SDK_VERSION_MINOR;
            resp->capFlags = CAPFLAG_WINK | CAPFLAG_CBOR;

            init_tx->cid = ctap_req->cid;
            init_tx->init.cmd = CTAPHID_INIT;
            init_tx->init.bcntl = 17;
            init_tx->init.bcnth = 0;
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)init_tx, 64));
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if (ctap_req->init.cmd == CTAPHID_WINK) {
            if (MSG_LEN(ctap_req) != 0) {
                return ctap_error(CTAP1_ERR_INVALID_LEN);
            }
            last_packet_time = 0;
            memcpy(ctap_resp, ctap_req, sizeof(CTAPHID_FRAME));
#if defined(PICO_PLATFORM) || defined(ESP_PLATFORM)
            sleep_ms(1000); //For blinking the device during 1 seg
#endif
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)ctap_resp, 64));
            msg_packet.len = msg_packet.current_len = 0;
        }
        else if ((last_cmd == CTAPHID_PING || last_cmd == CTAPHID_SYNC) &&
                 (msg_packet.len == 0 ||
                  (msg_packet.len == msg_packet.current_len && msg_packet.len > 0))) {
            if (msg_packet.current_len == msg_packet.len && msg_packet.len > 0) {
                memcpy(ctap_resp->init.data, msg_packet.data, msg_packet.len);
                resp_cid = ctap_req->cid;
                resp_cmd = last_cmd;
                driver_exec_finished_hid(msg_packet.len);
            }
            else {
                /* Inline echo in the tail frame (see send_keepalive): a
                 * PING of another channel is answered while a transaction
                 * may be in flight, and must not overwrite the TX ring's
                 * pending response data. */
                CTAPHID_FRAME *echo = (CTAPHID_FRAME *) (hid_tx[ITF_HID_CTAP].buffer + sizeof(hid_tx[ITF_HID_CTAP].buffer) - 64);
                memcpy(echo->init.data, ctap_req->init.data, MSG_LEN(ctap_req));
                echo->cid = ctap_req->cid;
                echo->init.cmd = last_cmd;
                echo->init.bcnth = MSG_LEN(ctap_req) >> 8;
                echo->init.bcntl = MSG_LEN(ctap_req) & 0xff;
                driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)echo, 64));
            }
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if (ctap_req->init.cmd == CTAPHID_LOCK) {
            if (MSG_LEN(ctap_req) != 1) {
                return ctap_error(CTAP1_ERR_INVALID_LEN);
            }
            if (ctap_req->init.data[0] > 10) {
                return ctap_error(CTAP1_ERR_INVALID_PARAMETER);
            }
            if (board_millis() < lock && ctap_req->cid != lock_cid) {
                return ctap_error(CTAP1_ERR_CHANNEL_BUSY);
            }
            if (ctap_req->init.data[0] == 0) {
                lock = 0;
                lock_cid = 0;
            }
            else {
                lock = board_millis() + ctap_req->init.data[0] * 1000;
                lock_cid = ctap_req->cid;
            }
            ctap_resp->cid = ctap_req->cid;
            ctap_resp->init.cmd = ctap_req->init.cmd;
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)ctap_resp, 64));
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if (ctap_req->init.cmd == CTAPHID_UUID) {
            ctap_resp->cid = ctap_req->cid;
            ctap_resp->init.cmd = ctap_req->init.cmd;
            memcpy(ctap_resp->init.data, pico_serial.id, sizeof(pico_serial.id));
            ctap_resp->init.bcntl = 16;
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)ctap_resp, 64));
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if (ctap_req->init.cmd == CTAPHID_VERSION) {
            ctap_resp->cid = ctap_req->cid;
            ctap_resp->init.cmd = ctap_req->init.cmd;
            ctap_resp->init.data[0] = PICOKEYS_SDK_VERSION_MAJOR;
            ctap_resp->init.data[1] = PICOKEYS_SDK_VERSION_MINOR;
            ctap_resp->init.bcntl = 4;
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)ctap_resp, 64));
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if (ctap_req->init.cmd == CTAPHID_ADMIN) {
            ctap_resp->cid = ctap_req->cid;
            ctap_resp->init.cmd = ctap_req->init.cmd;
            if (ctap_req->init.data[0] == 0x80) { // Status
                memcpy(ctap_resp->init.data, "\x00\xff\xff\xff\x00", 5);
                ctap_resp->init.bcntl = 5;
            }
            driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)ctap_resp, 64));
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if ((last_cmd == CTAPHID_MSG || last_cmd == CTAPHID_OTP) &&
                 (msg_packet.len == 0 ||
                  (msg_packet.len == msg_packet.current_len && msg_packet.len > 0))) {
            if (last_cmd == CTAPHID_OTP) {
                is_nk = true;
#ifdef ENABLE_OATH_APP
                select_app(CONST_BYTE_ARRAY(oath_aid + 1, oath_aid[0]));
#endif
            }
            else {
                select_app(CONST_BYTE_ARRAY(u2f_aid + 1, u2f_aid[0]));
            }

            thread_type = 1;
            resp_cid = ctap_req->cid;
            resp_cmd = last_cmd;

            if (msg_packet.current_len == msg_packet.len && msg_packet.len > 0) {
                apdu_sent = apdu_process(ITF_HID_CTAP, CONST_BYTE_ARRAY(msg_packet.data, msg_packet.len));
            }
            else {
                apdu_sent = apdu_process(ITF_HID_CTAP, CONST_BYTE_ARRAY(ctap_req->init.data, MSG_LEN(ctap_req)));
            }
            DEBUG_PAYLOAD(apdu.data, (int) apdu.nc);
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
        }
        else if ((last_cmd == CTAPHID_CBOR || last_cmd >= CTAPHID_VENDOR_FIRST) &&
                 (msg_packet.len == 0 || (msg_packet.len == msg_packet.current_len && msg_packet.len > 0))) {
            thread_type = 2;
            resp_cid = ctap_req->cid;
            resp_cmd = last_cmd;
            /* The admission-deferral gate above keeps this branch out of a
             * cancelled transaction's unwind window, so a pending
             * cancellation is never cleared here before the old UP wait has
             * observed it; the guard documents that invariant. */
            if (!exec_finished_cancelled) {
                cancel_button = false;
            }
            select_app(CONST_BYTE_ARRAY(fido_aid + 1, fido_aid[0]));
            if (msg_packet.current_len == msg_packet.len && msg_packet.len > 0) {
                apdu_sent = cbor_process(last_cmd, msg_packet.data, msg_packet.len);
            }
            else {
                apdu_sent = cbor_process(last_cmd, ctap_req->init.data, MSG_LEN(ctap_req));
            }
            msg_packet.len = msg_packet.current_len = 0;
            last_packet_time = 0;
            if (apdu_sent < 0) {
                return ctap_error((uint8_t)(-apdu_sent));
            }
            last_keepalive_time = 0;
            send_keepalive();
        }
        else {
            if (msg_packet.len == 0) {
                return ctap_error(CTAP1_ERR_INVALID_CMD);
            }
        }
        // echo back anything we received from host
        //tud_hid_report(0, buffer, bufsize);
        //printf("END\n");
        if (apdu_sent > 0) {
            if (apdu_sent == 1) {
                card_start(ITF_HID, apdu_thread);
            }
            else if (apdu_sent == 2) {
                card_start(ITF_HID, cbor_thread);
            }
            usb_send_event(EV_CMD_AVAILABLE);
            hid_txn_start(ctap_req->cid);
        }
    }
    return apdu_sent;
}

static void send_keepalive(void) {
    if (thread_type == 1 || cancel_button) {
        return;
    }
    uint32_t now = board_millis();
    if (last_keepalive_time != 0 && now - last_keepalive_time < 250) {
        return;
    }
    CTAPHID_FRAME *resp = (CTAPHID_FRAME *) (hid_tx[ITF_HID_CTAP].buffer + sizeof(hid_tx[ITF_HID_CTAP].buffer) - 64);
    //memset(ctap_resp, 0, sizeof(CTAPHID_FRAME));
    resp->cid = resp_cid;
    resp->init.cmd = CTAPHID_KEEPALIVE;
    resp->init.bcntl = 1;
    resp->init.data[0] = is_req_button_pending() ? 2 : 1;
    //send_buffer_size[ITF_HID_CTAP] = 0;
    if (driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY((const uint8_t *)resp, 64)) > 0) {
        last_keepalive_time = now;
    }
}

void driver_exec_finished_hid(uint16_t size_next) {
    if (size_next > 0) {
        if (thread_type == 2 && apdu.sw != 0) {
            ctap_error_cid(resp_cid, apdu.sw & 0xff);
        }
        else {
            if (is_nk) {
                memmove(apdu.rdata + 2, apdu.rdata, size_next - 2);
                put_uint16_be(apdu.sw, apdu.rdata);
            }
            driver_exec_finished_cont_hid(ITF_HID_CTAP, size_next, 7);
        }
    }
    apdu.sw = 0;
}

void driver_exec_finished_cont_hid(uint8_t itf, uint16_t size_next, uint16_t offset) {
    offset -= 7;
    ctap_resp = (CTAPHID_FRAME *) (hid_tx[itf].buffer + offset);
    ctap_resp->cid = resp_cid;
    ctap_resp->init.bcnth = size_next >> 8;
    ctap_resp->init.bcntl = size_next & 0xff;
    send_buffer_size[itf] = size_next;
    ctap_resp->init.cmd = resp_cmd;
    if (hid_write_offset(size_next+7, offset) > 0) {
        //ctap_resp = (CTAPHID_FRAME *) ((uint8_t *) ctap_resp + 64 - 5);
        //send_buffer_size[ITF_HID_CTAP] -= MIN(64 - 7, send_buffer_size[ITF_HID_CTAP]);
    }
}

void hid_task(void) {
    const uint32_t status_poll_interval_ms = 1;
    static uint32_t last_status_poll_ms = 0;
#ifdef ENABLE_EMULATION
    uint16_t rx_len = emul_read(ITF_HID);
    if (rx_len) {
        uint16_t rptr = 0;
        while (rx_len > 0) {
            tud_hid_set_report_cb(ITF_HID, 0, 0, emul_rx + rptr, 64);
            rx_len -= 64;
            rptr += 64;
        }
        emul_rx_size = 0;
    }
#endif
    int proc_pkt = 0;
    if (hid_rx[ITF_HID_CTAP].w_ptr - hid_rx[ITF_HID_CTAP].r_ptr >= 64) {
        //proc_pkt = driver_process_usb_packet_hid(64);
    }
    if (proc_pkt == 0) {
        driver_process_usb_nopacket_hid();
    }
    uint32_t now_ms = board_millis();
    if (now_ms - last_status_poll_ms >= status_poll_interval_ms) {
        last_status_poll_ms = now_ms;
        int status = card_status(ITF_HID);
        if (status == PICOKEYS_OK) {
            driver_exec_finished_hid(finished_data_size);
        }
        else if (status == PICOKEYS_ERR_BLOCKED) {
            send_keepalive();
        }
    }
    /* A packet buffered while a cancelled transaction was unwinding is
     * replayed once the marker cleared: the cancellation has been delivered
     * and the late completion consumed, so the replay's EV_CMD_AVAILABLE
     * can only be consumed by the worker's command loop. The ring held one
     * complete maximum-size message of the unwinding channel, so the replay
     * never reassembles a partial one. */
    hid_txn_sync();
    if (!exec_finished_cancelled && deferred_count > 0) {
        uint8_t replay[HID_RPT_SIZE];
        while (deferred_count > 0 && !exec_finished_cancelled) {
            memcpy(replay, deferred_reports[deferred_tail], HID_RPT_SIZE);
            deferred_tail = (deferred_tail + 1) % HID_DEFERRED_MAX;
            deferred_count--;
            tud_hid_set_report_cb(ITF_HID_CTAP, 0, 0, replay, HID_RPT_SIZE);
        }
    }
    if (hid_tx[ITF_HID_CTAP].w_ptr > hid_tx[ITF_HID_CTAP].r_ptr && last_write_result[ITF_HID_CTAP] != WRITE_PENDING) {
        if (driver_write_hid(ITF_HID_CTAP, CONST_BYTE_ARRAY(hid_tx[ITF_HID_CTAP].buffer + hid_tx[ITF_HID_CTAP].r_ptr, 64)) > 0) {

        }
    }
#ifndef ENABLE_EMULATION
    /* Keyboard ITF */
    // Poll every 10ms
    const uint32_t interval_ms = 10;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms) {
        return;
    }
    start_ms += interval_ms;

    // Remote wakeup
    if (tud_suspended() && kb_tx_typing()) {
        tud_remote_wakeup();
    }
    else {
        send_hid_report(REPORT_ID_KEYBOARD);
    }
#endif
}
