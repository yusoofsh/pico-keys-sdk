#pragma once
#include <stdbool.h>
#include <stdint.h>
bool multicore_lockout_start_timeout_us(uint64_t us);
bool multicore_lockout_end_timeout_us(uint64_t us);
void multicore_lockout_victim_init(void);
unsigned get_core_num(void);
static inline void tight_loop_contents(void) {}
