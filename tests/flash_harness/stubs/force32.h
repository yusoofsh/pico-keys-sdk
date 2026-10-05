/* Forced-include: emulate the 32-bit ARM uintptr_t so flash_journal_t keeps its 32-byte layout on x86_64. */
#pragma once
#include <stdint.h>
#define uintptr_t uint32_t
#undef UINTPTR_MAX
#define UINTPTR_MAX UINT32_MAX
