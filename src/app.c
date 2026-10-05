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

// Application registry: AID selection and per-app dispatch. Kept apart from
// main.c so host tests can link the dispatcher stack without the emulator
// entry point.

#include "picokeys.h"
#include "apdu.h"

app_t apps[16];
uint8_t num_apps = 0;

app_t *current_app = NULL;

// ATR the current application reports over CCID (declared in ccid/ccid.h);
// each application constructor points it at its own ATR.
const uint8_t *ccid_atr = NULL;

bool app_exists(const_byte_array_t aid) {
    for (int a = 0; a < num_apps; a++) {
        if (aid.len >= apps[a].aid[0] && !memcmp(apps[a].aid + 1, aid.data, apps[a].aid[0])) {
            return true;
        }
    }
    return false;
}

int register_app(int (*select_aid)(app_t *, uint8_t), const uint8_t *aid) {
    if (app_exists(CONST_BYTE_ARRAY(aid + 1, aid[0]))) {
        return 1;
    }
    if (num_apps < sizeof(apps) / sizeof(app_t)) {
        apps[num_apps].select_aid = select_aid;
        apps[num_apps].aid = aid;
        num_apps++;
        return 1;
    }
    return 0;
}

int select_app(const_byte_array_t aid) {
    if (current_app && current_app->aid && (current_app->aid + 1 == aid.data || (aid.len >= current_app->aid[0] && !memcmp(current_app->aid + 1, aid.data, current_app->aid[0])))) {
        current_app->select_aid(current_app, 0);
        return PICOKEYS_OK;
    }
    for (int a = 0; a < num_apps; a++) {
        if (aid.len >= apps[a].aid[0] && !memcmp(apps[a].aid + 1, aid.data, apps[a].aid[0])) {
            if (current_app) {
                if (current_app->aid && aid.len >= current_app->aid[0] && !memcmp(current_app->aid + 1, aid.data, current_app->aid[0])) {
                    current_app->select_aid(current_app, 1);
                    return PICOKEYS_OK;
                }
                if (current_app->unload) {
                    current_app->unload();
                }
            }
            current_app = &apps[a];
            if (current_app->select_aid(current_app, 1) == PICOKEYS_OK) {
                return PICOKEYS_OK;
            }
        }
    }
    return PICOKEYS_ERR_FILE_NOT_FOUND;
}
