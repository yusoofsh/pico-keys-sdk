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

#ifndef _BUTTON_EMUL_H_
#define _BUTTON_EMUL_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "button.h"

/*
 * Emulated BOOT button, ENABLE_EMULATION builds only.
 *
 * The simulated user is driven through a control file named by the
 * environment variable PICOKEYS_EMULATION_BUTTON_FILE. When the variable is
 * unset, or the file does not exist (yet), is empty or holds `auto`, user
 * presence is auto-accepted exactly like the plain upstream emulation.
 *
 * The file holds one command; rewriting it atomically (write a temporary
 * file, then rename) issues a new command. Each command is recognized by the
 * file modification time and is consumed once:
 *
 *   auto               upstream auto-accept (default mode)
 *   none               the user never touches the button; waits end in a
 *                      timeout or a cancel
 *   press              the user presses during the next wait; the press is
 *                      delivered only after a wait has started and is
 *                      consumed by exactly one wait
 *   press-after:<ms>   a press <ms> after this command was written; it is
 *                      delivered only if a wait is active at that moment and
 *                      discarded otherwise (no stale press)
 *   cancel             aborts the active wait; discarded when no wait is
 *                      active
 *   timeout:<seconds>  emulation-only override of the user-presence wait
 *                      timeout; timeout:0 restores the default, which is the
 *                      PICOKEYS_EMULATION_BUTTON_TIMEOUT environment variable
 *                      when set, and 0 (the fresh-device default) otherwise
 *
 * Commands are polled about every 10 ms while the main loop runs, so a
 * command must be written before the request that should observe it.
 */

typedef enum {
    EMUL_BTN_AUTO = 0,
    EMUL_BTN_NONE,
    EMUL_BTN_PRESS,
    EMUL_BTN_PRESS_AFTER,
    EMUL_BTN_CANCEL,
    EMUL_BTN_TIMEOUT,
} emul_button_cmd_t;

/* Pure: parse one command line. press-after yields milliseconds, timeout
 * yields seconds, in *param. Returns false for unknown commands. */
bool emul_button_parse(const char *text, size_t len, emul_button_cmd_t *cmd, uint32_t *param);

/* Pure: effective user-presence timeout override in seconds (0 = default). */
uint32_t emul_button_timeout_seconds(void);

/* Runtime interface, used by src/button.c and the application. */
bool emul_button_controlled(void); /* PICOKEYS_EMULATION_BUTTON_FILE is set */
bool emul_button_auto(void);       /* auto-accept: env unset, or mode auto */
void emul_button_note_main_thread(void); /* record the main-loop thread */
bool emul_button_on_main_thread(void);
void emul_button_wait_start(uint32_t timeout_ms);
bool emul_button_wait_active(void);
void emul_button_wait_end(void);
button_event_t emul_button_wait_poll(void);
void emul_button_task(void);

/* Synchronous injection seam, for tests only: apply one command as if it
 * had just been parsed from a freshly written control file, but without
 * the file, its mtime dedup (whose coarse-clock granularity drops
 * commands written less than about one kernel tick apart) or the 10 ms
 * poll throttle. The state is updated before the call returns. */
void emul_button_inject(emul_button_cmd_t cmd, uint32_t param);

/* Blocking wait for callers that run on the main-loop thread (under
 * emulation the CCID and keyboard-HID transports are served there, so those
 * callers must not block on the button queues). Returns 0 pressed,
 * 1 timeout, 2 cancelled. */
int emul_button_wait_local(uint32_t timeout_ms);

#endif // _BUTTON_EMUL_H_
