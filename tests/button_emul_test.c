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

#include "usb/emulation/button_emul.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _MSC_VER
#include "compat/pthread_win32.h"
#else
#include <pthread.h>
#include <unistd.h>
#endif

#include "compat/board.h"

/*
 * Host tests for the emulated BOOT button (ENABLE_EMULATION builds only).
 * The parser cases are pure; the state-machine cases drive the real
 * control-file polling through temporary files and real time. All waits in
 * this file are well below a second.
 */

static char btn_file[256];
static char btn_tmp[260];

static void write_btn(const char *text) {
    FILE *f = fopen(btn_tmp, "wb");
    assert(f != NULL);
    fputs(text, f);
    fclose(f);
    assert(rename(btn_tmp, btn_file) == 0);
    /* Guarantee a distinct modification time for consecutive commands. */
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 2000000 };
    nanosleep(&ts, NULL);
}

static void env_setup(void) {
    snprintf(btn_file, sizeof(btn_file), "/tmp/picokeys_button_emul_test.cmd");
    snprintf(btn_tmp, sizeof(btn_tmp), "%s.tmp", btn_file);
    setenv("PICOKEYS_EMULATION_BUTTON_FILE", btn_file, 1);
    unsetenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT");
    remove(btn_file);
    remove(btn_tmp);
    /* Deterministic baseline for every state-machine case. */
    write_btn("timeout:0");
    write_btn("auto");
}

static void env_teardown(void) {
    remove(btn_file);
    remove(btn_tmp);
    unsetenv("PICOKEYS_EMULATION_BUTTON_FILE");
    unsetenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT");
}

static void msleep_test(unsigned long ms) {
    struct timespec ts = { .tv_sec = (time_t) (ms / 1000), .tv_nsec = (long) (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* One main-loop tick: sleep past the poll throttle, then poll. Commands are
 * picked up by this poll, exactly as the emulator main loop does. */
static void pump(void) {
    msleep_test(15);
    emul_button_task();
}

/* Poll the active wait until it yields an event or timeout_ms elapse. */
static button_event_t poll_until(uint32_t timeout_ms) {
    uint32_t started = board_millis();
    while (board_millis() - started < timeout_ms) {
        button_event_t ev = emul_button_wait_poll();
        if (ev != BUTTON_EV_NONE) {
            return ev;
        }
        msleep_test(5);
    }
    return BUTTON_EV_NONE;
}

static void test_parse_commands(void) {
    emul_button_cmd_t cmd;
    uint32_t param = 0;
    assert(emul_button_parse("auto", 4, &cmd, &param) && cmd == EMUL_BTN_AUTO && param == 0);
    assert(emul_button_parse("none\n", 5, &cmd, &param) && cmd == EMUL_BTN_NONE && param == 0);
    assert(emul_button_parse("press\r\n", 7, &cmd, &param) && cmd == EMUL_BTN_PRESS);
    assert(emul_button_parse("  press  ", 9, &cmd, &param) && cmd == EMUL_BTN_PRESS);
    assert(emul_button_parse("cancel", 6, &cmd, &param) && cmd == EMUL_BTN_CANCEL);
    assert(emul_button_parse("press-after:1000", 16, &cmd, &param) && cmd == EMUL_BTN_PRESS_AFTER && param == 1000);
    assert(emul_button_parse("press-after:0\n", 14, &cmd, &param) && cmd == EMUL_BTN_PRESS_AFTER && param == 0);
    assert(emul_button_parse("timeout:2", 9, &cmd, &param) && cmd == EMUL_BTN_TIMEOUT && param == 2);
    assert(emul_button_parse("timeout:0", 9, &cmd, &param) && cmd == EMUL_BTN_TIMEOUT && param == 0);

    /* Unknown, truncated, non-numeric and overflowing commands are rejected. */
    assert(!emul_button_parse("", 0, &cmd, &param));
    assert(!emul_button_parse("presses", 7, &cmd, &param));
    assert(!emul_button_parse("press-after:", 12, &cmd, &param));
    assert(!emul_button_parse("press-after:1x", 14, &cmd, &param));
    assert(!emul_button_parse("timeout:", 8, &cmd, &param));
    assert(!emul_button_parse("timeout:abc", 11, &cmd, &param));
    assert(!emul_button_parse("AUTO", 4, &cmd, &param));
    assert(!emul_button_parse("press-after:99999999999", 23, &cmd, &param));
    assert(!emul_button_parse(NULL, 0, &cmd, &param));
}

static void test_timeout_override_resolution(void) {
    env_setup();
    /* No override anywhere: the fresh-device default applies. */
    assert(emul_button_timeout_seconds() == 0);
    /* The environment override applies when no file command is in effect. */
    setenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT", "3", 1);
    assert(emul_button_timeout_seconds() == 3);
    /* A file command wins over the environment, and timeout:0 clears it. */
    write_btn("timeout:1");
    pump();
    assert(emul_button_timeout_seconds() == 1);
    write_btn("timeout:0");
    pump();
    assert(emul_button_timeout_seconds() == 0);
    unsetenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT");
    /* Garbage environment values fall back to the default. */
    setenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT", "not-a-number", 1);
    assert(emul_button_timeout_seconds() == 0);
    unsetenv("PICOKEYS_EMULATION_BUTTON_TIMEOUT");
    env_teardown();
}

static void test_press_delivered_once_after_wait_starts(void) {
    env_setup();
    /* An idle poll must not deliver or consume the press. */
    write_btn("press");
    pump();
    assert(emul_button_wait_poll() == BUTTON_EV_NONE); /* no wait active */
    /* The next wait receives the press promptly. */
    emul_button_wait_start(5000);
    assert(poll_until(2000) == BUTTON_EV_PRESSED);
    emul_button_wait_end();
    /* The same command is consumed by exactly one wait: the second wait
     * never completes by press and ends in a timeout. */
    emul_button_wait_start(200);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    /* A rewritten press command is a new command for the next wait. */
    write_btn("press");
    emul_button_wait_start(5000);
    assert(poll_until(2000) == BUTTON_EV_PRESSED);
    emul_button_wait_end();
    env_teardown();
}

static void test_press_after_delivered_while_waiting(void) {
    env_setup();
    write_btn("none");
    write_btn("timeout:0");
    write_btn("press-after:100");
    /* The command may be written before or while the wait runs; it is
     * delivered only because a wait is active when it fires. */
    emul_button_wait_start(3000);
    uint32_t started = board_millis();
    button_event_t ev = poll_until(2000);
    uint32_t elapsed = board_millis() - started;
    assert(ev == BUTTON_EV_PRESSED);
    assert(elapsed >= 80);
    assert(elapsed < 2000);
    emul_button_wait_end();
    env_teardown();
}

static void test_press_after_discarded_without_wait(void) {
    env_setup();
    write_btn("none");
    write_btn("timeout:0");
    write_btn("press-after:80");
    /* The press fires while idle and must be discarded (no stale press). */
    for (int i = 0; i < 30; i++) {
        pump();
        msleep_test(10);
    }
    emul_button_wait_start(500);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    env_teardown();
}

static void test_press_after_boundary_no_idle_poll(void) {
    env_setup();
    write_btn("none");
    write_btn("timeout:0");
    write_btn("press-after:80");
    /* Advance past the press deadline WITHOUT any idle poll: neither
     * emul_button_task nor a wait poll may run in between, so the command
     * is still pending when the wait activates. */
    uint32_t written = board_millis();
    while (board_millis() - written < 150) {
        msleep_test(5);
    }
    /* The press fired while no wait was active: it belongs to no request
     * and must never authorize this wait (the no-stale rule), even though
     * no idle poll discarded it. */
    emul_button_wait_start(400);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    /* A fresh press written while the next wait runs still succeeds. */
    write_btn("press-after:100");
    emul_button_wait_start(3000);
    assert(poll_until(2000) == BUTTON_EV_PRESSED);
    emul_button_wait_end();
    env_teardown();
}

static void test_cancel_aborts_only_the_active_wait(void) {
    env_setup();
    write_btn("none");
    write_btn("timeout:0");
    /* A cancel with no active wait is discarded, not queued. */
    write_btn("cancel");
    for (int i = 0; i < 5; i++) {
        pump();
        msleep_test(10);
    }
    emul_button_wait_start(300);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    /* A cancel written while a wait is active aborts it. */
    write_btn("cancel");
    emul_button_wait_start(30000);
    assert(emul_button_wait_poll() == BUTTON_EV_CANCELLED);
    emul_button_wait_end();
    env_teardown();
}

static void test_none_never_presses(void) {
    env_setup();
    write_btn("none");
    emul_button_wait_start(150);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    env_teardown();
}

static void test_absent_file_keeps_last_mode(void) {
    env_setup(); /* ends with mode auto */
    pump();      /* the auto command is now the last parsed mode */
    remove(btn_file);
    /* An absent or unreadable file keeps the last mode; on a fresh boot
     * (no command ever read) that mode is auto. */
    assert(emul_button_auto() == true);
    assert(emul_button_controlled() == true);
    env_teardown();
}

static void *cancel_writer(void *arg) {
    (void) arg;
    msleep_test(100);
    write_btn("cancel");
    return NULL;
}

static void test_wait_local_press_timeout_and_cancel(void) {
    env_setup();
    emul_button_note_main_thread();
    write_btn("press");
    assert(emul_button_wait_local(5000) == 0);
    write_btn("none");
    assert(emul_button_wait_local(150) == 1);
    /* A cancel written by another thread while the local wait runs. */
    pthread_t th;
    assert(pthread_create(&th, NULL, cancel_writer, NULL) == 0);
    assert(emul_button_wait_local(30000) == 2);
    pthread_join(th, NULL);
    env_teardown();
}

static void test_inject_seam_is_synchronous(void) {
    env_setup();
    /* Remove the control file entirely: the injected state must stand on
     * its own, and the absent file must never disturb it (keeps last
     * mode). */
    remove(btn_file);
    remove(btn_tmp);

    /* none: waits run to their timeout. */
    emul_button_inject(EMUL_BTN_NONE, 0);
    emul_button_wait_start(150);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();

    /* press: delivered on the very next poll with no sleep in between,
     * and consumed by exactly one wait. */
    emul_button_inject(EMUL_BTN_PRESS, 0);
    emul_button_wait_start(5000);
    assert(emul_button_wait_poll() == BUTTON_EV_PRESSED);
    assert(emul_button_wait_poll() == BUTTON_EV_NONE);
    emul_button_wait_end();
    emul_button_wait_start(150);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();

    /* A press injected while idle stays pending for the NEXT wait (same
     * semantics as the file command), and a later none clears it before
     * it can authorize anything (no stale press). */
    emul_button_inject(EMUL_BTN_PRESS, 0);
    emul_button_inject(EMUL_BTN_NONE, 0);
    emul_button_wait_start(150);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();

    /* cancel: discarded when injected with no active wait, delivered when
     * injected while one runs (the synchronous analogue of writing the
     * file while the wait is active). */
    emul_button_inject(EMUL_BTN_CANCEL, 0);
    emul_button_wait_start(150);
    assert(poll_until(2000) == BUTTON_EV_TIMEOUT);
    emul_button_wait_end();
    emul_button_wait_start(30000);
    emul_button_inject(EMUL_BTN_CANCEL, 0);
    assert(emul_button_wait_poll() == BUTTON_EV_CANCELLED);
    emul_button_wait_end();

    /* timeout: the override applies and clears, synchronously. */
    emul_button_inject(EMUL_BTN_TIMEOUT, 2);
    assert(emul_button_timeout_seconds() == 2);
    emul_button_inject(EMUL_BTN_TIMEOUT, 0);
    assert(emul_button_timeout_seconds() == 0);

    env_teardown();
}

int main(void) {
    test_parse_commands();
    test_timeout_override_resolution();
    test_press_delivered_once_after_wait_starts();
    test_press_after_delivered_while_waiting();
    test_press_after_discarded_without_wait();
    test_press_after_boundary_no_idle_poll();
    test_cancel_aborts_only_the_active_wait();
    test_none_never_presses();
    test_absent_file_keeps_last_mode();
    test_wait_local_press_timeout_and_cancel();
    test_inject_seam_is_synchronous();
    printf("button_emul_test: all cases passed\n");
    return 0;
}
