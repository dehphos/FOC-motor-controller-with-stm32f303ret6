
#ifndef TEST_SPD_H
#define TEST_SPD_H

#include "main.h"
#include "math.h"
#include "control.h"
#include "main.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"


void test_speed_pi(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick);

#endif /* TEST_SPD_H */
