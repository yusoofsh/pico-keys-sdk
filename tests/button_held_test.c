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

#include "button_mocks.h"
#include "button.h"
#include "usb.h"
#include "signal.h"
#include "led/led.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/*
 * Host regression for the firmware (PICO_PLATFORM) UP wait state machine in
 * src/button.c, compiled UNMODIFIED against mocked Pico SDK registers,
 * queues and clock. It pins the no-stale rule: a BOOT level that is already
 * held when a wait starts (a press that began before the request, or one
 * carried over from a cancelled or timed-out request) never authorizes it;
 * the wait only completes on a fresh press/release that begins after the
 * wait started. One scenario per process (argv[1]); each run drives
 * button_task()/button_wait_start() exactly as core0 and the UP caller do.
 */

mock_phy_t phy_data;
queue_t usb_to_card_q, card_to_usb_q;
static mock_ioqspi_t qspi_registers;
static mock_sio_t sio_registers;
mock_ioqspi_t *ioqspi_hw = &qspi_registers;
mock_sio_t *sio_hw = &sio_registers;
static uint32_t now_ms, led_mode = MODE_MOUNTED;
static unsigned completed, cancelled, timed_out;
uint32_t board_millis(void) { return now_ms; }
bool is_busy(void) { return is_req_button_pending(); }
uint32_t led_get_mode(void) { return led_mode; }
void led_set_mode(uint32_t mode) { led_mode = mode; }
uint32_t save_and_disable_interrupts(void) { return 0; }
void restore_interrupts(uint32_t flags) { (void)flags; }
void hw_write_masked(uint32_t *address, uint32_t value, uint32_t mask) {
    *address = (*address & ~mask) | (value & mask);
}
bool multicore_lockout_start_timeout_us(uint64_t us) { (void)us; return true; }
bool multicore_lockout_end_timeout_us(uint64_t us) { (void)us; return true; }
bool queue_try_add(queue_t *queue, const void *value) {
    assert(queue == &usb_to_card_q);
    assert(queue->count < 16);
    queue->events[queue->count++] = *(const uint32_t *)value;
    return true;
}
int signal_emit_param(signal_code_t code, void *data) {
    assert(code == SIGNAL_USER_PRESENCE_REQUEST);
    assert(((signal_user_presence_request_data_t *)data)->timeout > 0);
    return 0;
}
int signal_emit(signal_code_t code) {
    if (code == SIGNAL_USER_PRESENCE_COMPLETED) completed++;
    else if (code == SIGNAL_USER_PRESENCE_CANCELLED) cancelled++;
    else if (code == SIGNAL_USER_PRESENCE_TIMEOUT) timed_out++;
    else assert(false);
    return 0;
}
static void level(bool pressed) {
    /* BOOTSEL is active-low on production RP2040 SIO QSPI-CS bit 1. */
    sio_registers.gpio_hi_in = pressed ? 0u : (1u << 1);
}
static void tick(uint32_t at, bool pressed) {
    now_ms = at;
    level(pressed);
    button_task();
}
static void expect_last(uint32_t event) {
    assert(usb_to_card_q.count > 0);
    assert(usb_to_card_q.events[usb_to_card_q.count - 1] == event);
    assert(!is_req_button_pending());
}
/* No event beyond the watermark: the active wait stayed pending. */
static unsigned queue_watermark = 0;
static void expect_no_new_event(void) {
    assert(usb_to_card_q.count == queue_watermark);
    assert(is_req_button_pending());
}

int main(int argc, char **argv) {
    assert(argc == 2);
    now_ms = 2000; /* past production button_task's 1-second startup guard */
    force_button_wait = true;
    phy_data.up_btn_present = false; /* real fresh-device configured timeout=0 */
    if (strcmp(argv[1], "preheld") == 0) {
        tick(2000, true); /* press began BEFORE any request */
        now_ms = 2100;
        button_wait_start();
        assert(is_req_button_pending());
        queue_watermark = usb_to_card_q.count;
        tick(2110, true);
        expect_no_new_event();
        tick(2120, false); /* release only; no fresh press in this request */
        expect_no_new_event();
        assert(completed == 0);
        tick(2130, true); /* a fresh press, begun after the wait started */
        expect_no_new_event();
        tick(2140, false); /* ...and its release complete the wait */
        expect_last(EV_BUTTON_PRESSED);
        assert(completed == 1);
        printf("preheld: pre-request hold ignored until release, then a fresh press/release completed (EV_BUTTON_PRESSED=%u)\n", EV_BUTTON_PRESSED);
    } else if (strcmp(argv[1], "cancelled_hold") == 0) {
        level(false);
        button_wait_start();
        tick(2010, true); /* press belongs to request A */
        assert(usb_to_card_q.count == 0);
        cancel_button = true;
        tick(2020, true);
        expect_last(EV_BUTTON_CANCELLED);
        queue_watermark = usb_to_card_q.count;
        now_ms = 2030;
        button_wait_start(); /* request B begins with A's press still held */
        tick(2040, true);
        expect_no_new_event();
        tick(2050, false); /* releasing A's hold must not authorize B */
        expect_no_new_event();
        assert(cancelled == 1 && completed == 0);
        tick(2060, true); /* fresh press in B */
        expect_no_new_event();
        tick(2070, false);
        expect_last(EV_BUTTON_PRESSED);
        assert(completed == 1);
        printf("cancelled_hold: request A cancelled (EV_BUTTON_CANCELLED=%u); its held press did not authorize B; a fresh press/release did\n", EV_BUTTON_CANCELLED);
    } else if (strcmp(argv[1], "timedout_hold") == 0) {
        phy_data.up_btn_present = true;
        phy_data.up_btn = 1;
        level(false);
        button_wait_start();
        tick(2010, true); /* press belongs to request A */
        assert(usb_to_card_q.count == 0);
        tick(3010, true); /* A times out while still held */
        expect_last(EV_BUTTON_TIMEOUT);
        queue_watermark = usb_to_card_q.count;
        now_ms = 3020;
        button_wait_start(); /* B starts with the same press still held */
        tick(3030, true);
        expect_no_new_event();
        tick(3040, false); /* releasing A's hold must not authorize B */
        expect_no_new_event();
        assert(timed_out == 1 && completed == 0);
        tick(3050, true); /* fresh press in B */
        expect_no_new_event();
        tick(3060, false);
        expect_last(EV_BUTTON_PRESSED);
        assert(completed == 1);
        printf("timedout_hold: request A timed out (EV_BUTTON_TIMEOUT=%u); its held press did not authorize B; a fresh press/release did\n", EV_BUTTON_TIMEOUT);
    } else if (strcmp(argv[1], "fresh_press") == 0) {
        level(false);
        button_wait_start();
        tick(2010, true);
        queue_watermark = usb_to_card_q.count;
        expect_no_new_event();
        tick(2020, false);
        expect_last(EV_BUTTON_PRESSED);
        assert(completed == 1);
        printf("fresh_press: fresh press/release -> EV_BUTTON_PRESSED=%u, completed=%u\n", EV_BUTTON_PRESSED, completed);
    } else if (strcmp(argv[1], "no_touch") == 0) {
        level(false);
        button_wait_start();
        tick(2100, false);
        assert(usb_to_card_q.count == 0 && is_req_button_pending());
        tick(32010, false); /* FORCE=true and timeout=0 resolves to 30 seconds */
        expect_last(EV_BUTTON_TIMEOUT);
        assert(timed_out == 1 && completed == 0);
        printf("no_touch: forced configured timeout=0 -> 30s -> EV_BUTTON_TIMEOUT=%u, completed=0\n", EV_BUTTON_TIMEOUT);
    } else {
        return 2;
    }
    return 0;
}
