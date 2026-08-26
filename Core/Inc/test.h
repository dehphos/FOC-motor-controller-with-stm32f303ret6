#ifndef TEST_H
#define TEST_H

#include "main.h"
#include "math.h"
#include "control.h"
#include "main.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"

#if TEST
void test(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick);
#endif

#endif /* TEST_H */
