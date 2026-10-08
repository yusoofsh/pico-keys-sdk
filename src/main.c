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

#include "picokeys.h"
#include "button.h"
#include <stdio.h>
#if !defined(ENABLE_EMULATION)
#include "tusb.h"
#endif
#if defined(ENABLE_EMULATION)
#include "emulation.h"
#elif defined(ESP_PLATFORM)
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "tinyusb.h"
#elif defined(PICO_PLATFORM)
#include "bsp/board.h"
#include "hardware/structs/ioqspi.h"
#include "pico/stdio.h"
#endif

#include "random.h"
#include "hwrng.h"
#include "apdu.h"
#include "usb.h"
#include "flash.h"
#include "otp.h"
#include "led/led.h"
#include "pico_time.h"
#include "serial.h"
#include "mbedtls/sha256.h"

extern int rescue_migrate_keydev(void);

// The application registry (apps, select_app, register_app) and the CCID ATR
// pointer live in app.c.

WEAK int picokey_init(void) {
    return 0;
}

/* Boot-stage hook: called once from main(), immediately before usb_init()
 * (and therefore before tusb_init()), after led_init(). A board or product
 * layer may provide a strong definition to latch early-boot state (for
 * example: hold the companion off for this boot); the default does nothing. */
WEAK void picokey_early_init(void) {
}

/* Main-loop hook: called once per core0_loop iteration, immediately after
 * button_task(). Must never block or sleep. */
WEAK void picokey_task(void) {
}

void execute_tasks(void);
void execute_tasks(void) {
#if !defined(ENABLE_EMULATION) && !defined(ESP_PLATFORM)
    tud_task(); // tinyusb device task
#endif
#ifdef USB_ITF_LWIP
#if !defined(ENABLE_EMULATION)
    service_traffic();
#endif
    rest_task();
#endif
    usb_task();
    led_blinking_task();
#ifdef ENABLE_LVGL_UI
    platform_ui_task();
#endif
}

static void core0_loop(void *arg) {
    (void)arg;
#if defined(ESP_PLATFORM) && defined(USB_ITF_LWIP)
    if (ITF_LWIP_TOTAL > 0) {
        lwip_itf_init();
    }
#endif
    while (1) {
        execute_tasks();
        hwrng_task();
        flash_task();
        button_task();
        picokey_task();
#ifdef PICO_PLATFORM
        // Avoid a pure busy loop on core0; gives the system a scheduling hint.
        tight_loop_contents();
#endif
#ifdef ESP_PLATFORM
        vTaskDelay(pdMS_TO_TICKS(10));
#endif
    }
}

#ifdef ESP_PLATFORM
extern tinyusb_config_t tusb_cfg;
extern const uint8_t desc_config[];
extern char *string_desc_arr[];
extern char *string_desc_itf[];
TaskHandle_t hcore0 = NULL, hcore1 = NULL;
int app_main(void) {
#else
int main(void) {
#endif

    serial_init();

#ifndef ENABLE_EMULATION
#ifdef PICO_PLATFORM
    board_init();
    stdio_init_all();
#endif

#else
    emul_init("127.0.0.1", 35963);
#endif

    random_init();

    otp_init();

    low_flash_init();

    // Storage-locked boots publish no bounds: the flash scan and the keydev
    // migration (both dereference the pool bounds through the file layer)
    // must not run at all. The device keeps booting into usb_init below.
    if (!low_flash_storage_locked()) {
        file_scan_flash();

        if (rescue_migrate_keydev() != PICOKEYS_OK) {
            printf("Device attestation key migration failed\n");
        }
    }

    init_rtc();

#ifndef ENABLE_EMULATION
    phy_init();
#endif

    led_init();

    picokey_early_init();

    usb_init();

#ifndef ENABLE_EMULATION
#ifdef ESP_PLATFORM
    gpio_pad_select_gpio(BOOT_PIN);
    gpio_set_direction(BOOT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(BOOT_PIN);

    tusb_cfg.string_descriptor[3] = pico_serial_str;
    if (phy_data.usb_product_present) {
        tusb_cfg.string_descriptor[2] = phy_data.usb_product;
    }
    static char tmps[5][32];
    const int max_desc_slots = 8 - 6;
    const int itf_desc_count = ITF_TOTAL < max_desc_slots ? ITF_TOTAL : max_desc_slots;
    for (int i = 0; i < itf_desc_count; i++) {
        strlcpy(tmps[i], tusb_cfg.string_descriptor[2], sizeof(tmps[0]));
        strlcat(tmps[i], " ", sizeof(tmps[0]));
        strlcat(tmps[i], string_desc_itf[i], sizeof(tmps[0]));
        tusb_cfg.string_descriptor[i+6] = tmps[i];
    }
    tusb_cfg.string_descriptor_count = 6 + itf_desc_count;
    tusb_cfg.configuration_descriptor = desc_config;

    tinyusb_driver_install(&tusb_cfg);
#else
    tusb_init();
#endif
#endif

#ifndef ENABLE_EMULATION
    picokey_init();
#endif

#ifdef ESP_PLATFORM
    xTaskCreatePinnedToCore(core0_loop, "core0", 4096*ITF_TOTAL*2, NULL, CONFIG_TINYUSB_TASK_PRIORITY - 1, &hcore0, ESP32_CORE0);
#else
    core0_loop(NULL);
#endif

    return 0;
}
