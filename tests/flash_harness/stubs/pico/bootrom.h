#pragma once
#include <stdint.h>
void reset_usb_boot(uint32_t a, uint32_t b);
int rom_load_partition_table(uint8_t *wa, uint32_t sz, bool force);
int rom_get_partition_table_info(uint32_t *out, uint32_t words, uint32_t flags);
#define PT_INFO_PARTITION_LOCATION_AND_FLAGS 0x0010
#define PT_INFO_SINGLE_PARTITION 0x8000
