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

#ifdef _MSC_VER
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include "compat/pthread_win32.h"
#else
#include <pthread.h>
#include <time.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "button_emul.h"
#include "compat/board.h"

/*
 * Emulated BOOT button. This file is compiled only in ENABLE_EMULATION
 * builds; firmware builds never contain it. The button state is simulated
 * from a control file; see button_emul.h for the exact semantics.
 *
 * Threading: the wait state below is only ever mutated on the main-loop
 * thread (button_task, and emul_button_wait_local for transports that are
 * served there). emul_button_auto()/emul_button_timeout_seconds() may also
 * be read from command threads; like force_button_wait, a slightly stale
 * read is harmless because the main-loop side re-evaluates before any wait
 * starts.
 */

#define ENV_BUTTON_FILE "PICOKEYS_EMULATION_BUTTON_FILE"
#define ENV_BUTTON_TIMEOUT "PICOKEYS_EMULATION_BUTTON_TIMEOUT"
#define POLL_INTERVAL_MS 10
#define MAX_PARAM_DIGITS 9

static emul_button_cmd_t current_cmd = EMUL_BTN_AUTO;
static bool cmd_consumed = true; /* no command seen yet */
static bool have_cmd_mtime = false;
static uint64_t cmd_mtime_key = 0;
static bool press_pending = false; /* `press` armed for the next wait */
static uint32_t press_due_ms = 0;  /* `press-after` absolute deadline */
static bool timeout_override_present = false;
static uint32_t timeout_override = 0;
static bool wait_active_flag = false;
static uint32_t wait_started_ms = 0;
static uint32_t wait_timeout_ms = 0;
static uint32_t last_poll_ms = 0;
static bool last_poll_valid = false;
static pthread_t main_thread_id;
static bool main_thread_known = false;

static bool parse_u32(const char *s, size_t len, uint32_t *out) {
    uint32_t v = 0;
    if (len == 0 || len > MAX_PARAM_DIGITS) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
        v = v * 10 + (uint32_t) (s[i] - '0');
    }
    *out = v;
    return true;
}

bool emul_button_parse(const char *text, size_t len, emul_button_cmd_t *cmd, uint32_t *param) {
    if (text == NULL || cmd == NULL) {
        return false;
    }
    if (param) {
        *param = 0;
    }
    size_t begin = 0, end = len;
    while (begin < end && (text[begin] == ' ' || text[begin] == '\t')) {
        begin++;
    }
    while (end > begin && (text[end - 1] == ' ' || text[end - 1] == '\t' ||
                           text[end - 1] == '\n' || text[end - 1] == '\r')) {
        end--;
    }
    const char *s = text + begin;
    const size_t n = end - begin;
    if (n == 4 && memcmp(s, "auto", 4) == 0) {
        *cmd = EMUL_BTN_AUTO;
        return true;
    }
    if (n == 4 && memcmp(s, "none", 4) == 0) {
        *cmd = EMUL_BTN_NONE;
        return true;
    }
    if (n == 5 && memcmp(s, "press", 5) == 0) {
        *cmd = EMUL_BTN_PRESS;
        return true;
    }
    if (n == 6 && memcmp(s, "cancel", 6) == 0) {
        *cmd = EMUL_BTN_CANCEL;
        return true;
    }
    if (n > 12 && memcmp(s, "press-after:", 12) == 0) {
        uint32_t ms = 0;
        if (!parse_u32(s + 12, n - 12, &ms)) {
            return false;
        }
        *cmd = EMUL_BTN_PRESS_AFTER;
        if (param) {
            *param = ms;
        }
        return true;
    }
    if (n > 8 && memcmp(s, "timeout:", 8) == 0) {
        uint32_t sec = 0;
        if (!parse_u32(s + 8, n - 8, &sec)) {
            return false;
        }
        *cmd = EMUL_BTN_TIMEOUT;
        if (param) {
            *param = sec;
        }
        return true;
    }
    return false;
}

uint32_t emul_button_timeout_seconds(void) {
    if (timeout_override_present) {
        return timeout_override;
    }
    const char *tov = getenv(ENV_BUTTON_TIMEOUT);
    if (tov != NULL && *tov != '\0') {
        char *end = NULL;
        long v = strtol(tov, &end, 10);
        if (end != tov && *end == '\0' && v > 0 && v <= 0x7fffffffL) {
            return (uint32_t) v;
        }
    }
    return 0;
}

bool emul_button_controlled(void) {
    return getenv(ENV_BUTTON_FILE) != NULL;
}

bool emul_button_auto(void) {
    return !emul_button_controlled() || current_cmd == EMUL_BTN_AUTO;
}

void emul_button_note_main_thread(void) {
    main_thread_id = pthread_self();
    main_thread_known = true;
}

bool emul_button_on_main_thread(void) {
    return main_thread_known && pthread_equal(pthread_self(), main_thread_id) != 0;
}

/* Milliseconds resolution would collapse two commands written in the same
 * millisecond; the nanosecond field keeps consecutive commands distinct. */
static uint64_t mtime_key(const struct stat *st) {
#ifdef _MSC_VER
    return (uint64_t) st->st_mtime * 1000000000ull;
#else
    return (uint64_t) st->st_mtim.tv_sec * 1000000000ull + (uint64_t) st->st_mtim.tv_nsec;
#endif
}

/* Apply one parsed command to the emulated button state. Shared by the
 * control-file poll and the synchronous emul_button_inject() hook, so both
 * entry points apply identical semantics. press_due_ms is the absolute
 * `press-after` deadline: derived from the file mtime by the poll and from
 * board_millis() by the hook. */
static void emul_button_apply(emul_button_cmd_t cmd, uint32_t param, uint32_t press_due) {
    current_cmd = cmd;
    press_pending = false;
    press_due_ms = press_due;
    cmd_consumed = false;
    switch (cmd) {
        case EMUL_BTN_AUTO:
        case EMUL_BTN_NONE:
            cmd_consumed = true;
            break;
        case EMUL_BTN_TIMEOUT:
            timeout_override_present = true;
            timeout_override = param;
            cmd_consumed = true;
            break;
        case EMUL_BTN_CANCEL:
            if (!wait_active_flag) {
                cmd_consumed = true; /* nothing to abort: discard */
            }
            break;
        case EMUL_BTN_PRESS:
            press_pending = true; /* delivered during the next wait */
            break;
        case EMUL_BTN_PRESS_AFTER:
            break;
    }
}

static void process_file(void) {
    const char *path = getenv(ENV_BUTTON_FILE);
    if (path == NULL || *path == '\0') {
        return;
    }
    struct stat st;
    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        /* Absent or unreadable file: keep the previous state. */
        return;
    }
    uint64_t key = mtime_key(&st);
    if (have_cmd_mtime && key == cmd_mtime_key) {
        return; /* command already seen */
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return;
    }
    char buf[64] = { 0 };
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    emul_button_cmd_t cmd;
    uint32_t param = 0;
    if (n == 0 || !emul_button_parse(buf, n, &cmd, &param)) {
        /* Still empty (being written) or unparsable: ignore until it parses. */
        return;
    }
    cmd_mtime_key = key;
    have_cmd_mtime = true;
    emul_button_apply(cmd, param, (uint32_t) (key / 1000000ull) + param);
}

/* Shared polling step: consumes idle cancels and press-after commands that
 * fired with no wait active (the no-stale rule). */
static void emul_button_update(void) {
    process_file();
    uint32_t now = board_millis();
    if (!cmd_consumed && current_cmd == EMUL_BTN_CANCEL && !wait_active_flag) {
        cmd_consumed = true;
    }
    if (!cmd_consumed && current_cmd == EMUL_BTN_PRESS_AFTER && !wait_active_flag &&
        (int32_t) (now - press_due_ms) >= 0) {
        cmd_consumed = true; /* discarded: no wait was active */
    }
}

void emul_button_task(void) {
    uint32_t now = board_millis();
    if (last_poll_valid && (int32_t) (now - last_poll_ms) < POLL_INTERVAL_MS) {
        return;
    }
    last_poll_ms = now;
    last_poll_valid = true;
    emul_button_update();
}

/* Synchronous test hook (tests may call it from any thread that owns the
 * main-loop side of the button state): apply one command exactly like a
 * freshly written control file would be parsed by process_file(), but with
 * no file, no mtime dedup and no poll throttle. The control file's command
 * dedup compares modification times, whose granularity is the kernel's
 * coarse clock tick (about CONFIG_HZ, e.g. 4 ms at HZ=250): two commands
 * written less than one tick apart are indistinguishable and the second is
 * silently dropped. Tests that need delivery racing neither the clock nor
 * the 10 ms poll throttle inject here instead. The file poll keeps working
 * alongside it: an absent file is a no-op, and a genuinely new file
 * command still overrides the injected state. */
void emul_button_inject(emul_button_cmd_t cmd, uint32_t param) {
    emul_button_apply(cmd, param, board_millis() + param);
}

void emul_button_wait_start(uint32_t timeout_ms) {
    wait_active_flag = true;
    wait_started_ms = board_millis();
    wait_timeout_ms = timeout_ms;
}

bool emul_button_wait_active(void) {
    return wait_active_flag;
}

void emul_button_wait_end(void) {
    wait_active_flag = false;
}

button_event_t emul_button_wait_poll(void) {
    if (!wait_active_flag) {
        return BUTTON_EV_NONE;
    }
    emul_button_update();
    uint32_t now = board_millis();
    if (!cmd_consumed && current_cmd == EMUL_BTN_PRESS && press_pending) {
        press_pending = false;
        cmd_consumed = true; /* consumed by exactly one wait */
        return BUTTON_EV_PRESSED;
    }
    if (!cmd_consumed && current_cmd == EMUL_BTN_PRESS_AFTER) {
        if ((int32_t) (press_due_ms - wait_started_ms) < 0) {
            /* The press fired before this wait started: no wait was active
             * at that moment (or it belonged to a previous request).
             * Discard it even when no idle poll ran in between, so a press
             * that precedes a request never authorizes it (no-stale rule). */
            cmd_consumed = true;
        }
        else if ((int32_t) (now - press_due_ms) >= 0) {
            cmd_consumed = true; /* delivered: a wait is active */
            return BUTTON_EV_PRESSED;
        }
    }
    if (!cmd_consumed && current_cmd == EMUL_BTN_CANCEL) {
        cmd_consumed = true;
        return BUTTON_EV_CANCELLED;
    }
    if (wait_started_ms + wait_timeout_ms < now) {
        return BUTTON_EV_TIMEOUT;
    }
    return BUTTON_EV_NONE;
}

#ifdef _MSC_VER
static void emul_msleep(uint32_t ms) {
    Sleep(ms);
}
#else
static void emul_msleep(uint32_t ms) {
    struct timespec ts = { .tv_sec = (time_t) (ms / 1000), .tv_nsec = (long) (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}
#endif

int emul_button_wait_local(uint32_t timeout_ms) {
    emul_button_wait_start(timeout_ms);
    int ret = 1;
    while (true) {
        button_event_t ev = emul_button_wait_poll();
        if (ev == BUTTON_EV_PRESSED) {
            ret = 0;
            break;
        }
        else if (ev == BUTTON_EV_CANCELLED) {
            ret = 2;
            break;
        }
        else if (ev == BUTTON_EV_TIMEOUT) {
            ret = 1;
            break;
        }
        else if (!emul_button_wait_active()) {
            /* Defensive: the wait was completed elsewhere. */
            ret = 0;
            break;
        }
        emul_msleep(2);
    }
    emul_button_wait_end();
    return ret;
}
