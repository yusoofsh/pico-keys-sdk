#pragma once
#include <stdint.h>
#include <stddef.h>
#ifndef XIP_BASE
#define XIP_BASE 0x10000000u
#endif
#define FLASH_SECTOR_SIZE 4096u
#define FLASH_PAGE_SIZE 256u
void flash_range_erase(uint32_t flash_offs, size_t count);
void flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
void flash_do_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t count);
