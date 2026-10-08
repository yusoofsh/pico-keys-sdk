#ifndef BUTTON_MOCKS_H
#define BUTTON_MOCKS_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
/* Host mocks for compiling the UNMODIFIED production src/button.c firmware
 * path (PICO_PLATFORM + PICO_RP2040). Supplied before any production include
 * (-include), they suppress only the broad platform/RTC headers via their own
 * include guards (_PICOKEYS_H_, TIME_H) and replace the Pico SDK registers;
 * production button.h, led.h, signal.h and usb.h, and all button.c logic,
 * remain unchanged. The stub headers in this directory (pico/multicore.h,
 * pico/util/queue.h, and the hardware headers) just include this file. */
#define _PICOKEYS_H_
#define TIME_H
typedef unsigned int uint;
typedef struct { bool up_btn_present; uint8_t up_btn; } mock_phy_t;
extern mock_phy_t phy_data;
typedef struct { uint32_t events[16]; unsigned count; } queue_t;
bool queue_try_add(queue_t *q, const void *value);
uint32_t board_millis(void);
typedef struct { struct { uint32_t ctrl; } io[6]; } mock_ioqspi_t;
typedef struct { uint32_t gpio_hi_in; } mock_sio_t;
extern mock_ioqspi_t *ioqspi_hw;
extern mock_sio_t *sio_hw;
#define __no_inline_not_in_flash_func(name) name
/* OE register fields affect only the mocked register; pin input is injected
 * separately into SIO gpio_hi_in bit 1, exactly as production reads BOOTSEL. */
#define GPIO_OVERRIDE_LOW 2u
#define GPIO_OVERRIDE_NORMAL 0u
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB 12u
#define IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS (3u << 12)
uint32_t save_and_disable_interrupts(void);
void restore_interrupts(uint32_t flags);
void hw_write_masked(uint32_t *address, uint32_t value, uint32_t mask);
bool multicore_lockout_start_timeout_us(uint64_t timeout_us);
bool multicore_lockout_end_timeout_us(uint64_t timeout_us);
#endif
