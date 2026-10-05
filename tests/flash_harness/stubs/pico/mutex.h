#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct { int owned; } mutex_t;
static inline void mutex_init(mutex_t *m) { m->owned = 0; }
static inline bool mutex_try_enter(mutex_t *m, uint32_t *o) { (void)o; if (m->owned) return false; m->owned = 1; return true; }
static inline void mutex_enter_blocking(mutex_t *m) { m->owned = 1; }
static inline void mutex_exit(mutex_t *m) { m->owned = 0; }
