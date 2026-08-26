#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test.h"

#if TEST
void test(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{
    if (!m->STATUS.ALIGNED || m->STATUS.STOPPED_FAULT) {
        return;
    }
    if (*sweep_done == 0)
    {
        if (!(*sweep_started))
        {
            if (HAL_GetTick() - system_start_tick >= 5000)
            {
                *sweep_started = 1;
                m->REF.RPM = 0.0f;
                *sweep_last_tick = HAL_GetTick();
            }
        }
        else
        {
            if (HAL_GetTick() - *sweep_last_tick >= 20)
            {
                *sweep_last_tick = HAL_GetTick();

                if (m->REF.RPM < m->PARAMS.MAX_RPM) {
                    m->REF.RPM += 10.0f;
                } else {
                    *sweep_done = 1;
                }
            }
        }
    }
    // STATE 1: Tepe noktadan hızı sertçe düşür
    else if (*sweep_done == 1)
    {
        if (HAL_GetTick() - *sweep_last_tick >= 2000)
        {
            *sweep_last_tick = HAL_GetTick();

            if (m->REF.RPM > -m->PARAMS.MAX_RPM) {
                m->REF.RPM -= m->PARAMS.MAX_RPM / 10.0f;
            } else {
                *sweep_done = 2;
            }
        }
    }
    // STATE 2: Hızı tekrar 0'a çek ve testleri sıfırla/ilerlet
    else if (*sweep_done == 2)
    {
        if (HAL_GetTick() - *sweep_last_tick >= 20)
        {
            *sweep_last_tick = HAL_GetTick();

            if (m->REF.RPM < 0.0f) {
                m->REF.RPM += 10.0f;
            }
            else
            {

				m->REF.RPM = 0.0f;
				*sweep_done = 4;

            }
        }
    }
}
#endif
