#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test_spd.h"


#if SPEED_TEST
PI_Test_Params pi_test_array[] = {
    // GRUP 1: Hantal ve Yumuşak (Overshoot hiç olmamalı ama hedefe geç ulaşmalı)
    {0.0005f, 0.00001f},
    {0.0005f, 0.00005f},

    // GRUP 2: Mevcut Referans Noktası ve Çevresi
    {0.0010f, 0.00002f},
    {0.0010f, 0.00005f}, // Senin şu anki ayarın
    {0.0010f, 0.00010f}, // İntegrali biraz hızlandıralım

    // GRUP 3: Agresif (Hedefe zımba gibi oturmalı, hafif overshoot olabilir)
    {0.0020f, 0.00005f},
    {0.0020f, 0.00010f},
    {0.0030f, 0.00010f},

    // GRUP 4: Çok Agresif / Sınırları Zorlayan (Motor titreyebilir veya rezonansa girebilir)
    {0.0050f, 0.00020f},
    {0.0080f, 0.00050f}
};
uint8_t current_test_index = 0;

void test_speed_pi(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{

	static uint16_t NUM_TESTS = sizeof(pi_test_array)/sizeof(pi_test_array[0]);

	  if (*sweep_done == 0 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	        {

	            if (!*sweep_started)
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

	                    if (m->REF.RPM < m->PARAMS.MAX_RPM)
	                    {
	                        m->REF.RPM += 100.0f;
	                    }
	                    else
	                    {
	                        *sweep_done = 1;
	                    }
	                }
	            }
	        }
	  if (*sweep_done == 1 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	        {
	                if (HAL_GetTick() - *sweep_last_tick >= 1000)
	                {
	                    *sweep_last_tick = HAL_GetTick();

	                    if (m->REF.RPM > -m->PARAMS.MAX_RPM)
	                    {
	                        m->REF.RPM -= m->PARAMS.MAX_RPM/10;
	                    }
	                    else
	                    {

	                        *sweep_done = 2;
	                    }
	                }
	            }
	  if (*sweep_done == 2 && m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
	 	        {

	 	                if (HAL_GetTick() - *sweep_last_tick >= 20)
	 	                {
	 	                    *sweep_last_tick = HAL_GetTick();

	 	                    if (m->REF.RPM < 0.0f)
	 	                    {
	 	                        m->REF.RPM += 100.0f;
	 	                    }
	 	                    else
	 	                    {
	 	                    	if(current_test_index < NUM_TESTS){
	 	                        m->REF.RPM = 0.0f;
	 	                        *sweep_done = 0;
	 	                        m->SPEED_PI_PARAMS.kp = pi_test_array[current_test_index].kp;
	 	                        m->SPEED_PI_PARAMS.ki = pi_test_array[current_test_index].ki;
	 	                        m->SPEED_PI_PARAMS.Speed_integral = 0.0f;
	 	                        current_test_index++;
	 	                    	}else{
		 	                        m->REF.RPM = 0.0f;
		 	                        *sweep_done = 4;
	 	                    	}
	 	                    }
	 	                }
	 	            }
}
#endif
