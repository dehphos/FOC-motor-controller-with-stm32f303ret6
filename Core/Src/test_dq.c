#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test_dq.h"

#if DQ_TEST
PI_Test_Params pi_test_array[] = {
		    {0.0100f, 0.0020f},
		    {0.0200f, 0.0040f},
		    {0.0300f, 0.0060f},
		    {0.0400f, 0.0080f},
		    {0.0500f, 0.0100f},
		    {0.0550f, 0.0110f},
		    {0.0600f, 0.0120f},
		    {0.0642f, 0.0126f},
		    {0.0700f, 0.0150f},
		    {0.0750f, 0.0175f},
		    {0.0780f, 0.0180f},
		    {0.0800f, 0.0200f},
		    {0.0820f, 0.0210f},
		    {0.0850f, 0.0220f},
		    {0.0900f, 0.0250f},
		    {0.0950f, 0.0280f},
		    {0.1000f, 0.0300f},
		    {0.1100f, 0.0350f},
		    {0.1300f, 0.0400f},
		    {0.1500f, 0.0500f}
};

uint8_t current_test_index = 0;
uint8_t step_test_state = 0;

void test_dq_pi(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{
	static uint16_t NUM_TESTS = sizeof(pi_test_array)/sizeof(pi_test_array[0]);
	  m->REF.RPM = 5000.0f;
	  m->REF.RPM_cur = 5000.0f;
  uint32_t now = HAL_GetTick();

  if (m->STATUS.ALIGNED && !m->STATUS.STOPPED_FAULT)
  {

      if (!*sweep_started)
      {
          if (now - system_start_tick >= 2000)
          {
              *sweep_started = 1;
              step_test_state = 0;

              m->DQ_PI_PARAMS.Iq_kp = pi_test_array[current_test_index].kp;
              m->DQ_PI_PARAMS.Iq_ki = pi_test_array[current_test_index].ki;
              m->DQ_PI_PARAMS.Id_kp = pi_test_array[current_test_index].kp;
              m->DQ_PI_PARAMS.Id_ki = pi_test_array[current_test_index].ki;

              m->DQ_PI_PARAMS.Iq_integral = 0.0f;
              m->DQ_PI_PARAMS.Id_integral = 0.0f;
              m->REF.Iq = 0.0f;

              *sweep_last_tick = now;
          }
      }
      else if (*sweep_done == 0)
      {

          if (step_test_state == 0)
          {
              m->REF.Iq = 0.0f;
              if (now - *sweep_last_tick >= 500)
              {
                  step_test_state = 1;
                  m->REF.Iq = 2.0f;
              }
          }

          else if (step_test_state == 1)
          {
              if (now - *sweep_last_tick >= 650)
              {
                  step_test_state = 2;
                  m->REF.Iq = 0.0f;
              }
          }

          else if (step_test_state == 2)
          {
              if (now - *sweep_last_tick >= 1000)
              {
                  current_test_index++;

                  if (current_test_index < NUM_TESTS)
                  {

                      m->DQ_PI_PARAMS.Iq_kp = pi_test_array[current_test_index].kp;
                      m->DQ_PI_PARAMS.Iq_ki = pi_test_array[current_test_index].ki;
                      m->DQ_PI_PARAMS.Id_kp = pi_test_array[current_test_index].kp;
                      m->DQ_PI_PARAMS.Id_ki = pi_test_array[current_test_index].ki;

                      m->DQ_PI_PARAMS.Iq_integral = 0.0f;
                      m->DQ_PI_PARAMS.Id_integral = 0.0f;

                      *sweep_last_tick = now;
                      step_test_state = 0;
                  }
                  else
                  {

                      m->REF.Iq = 0.0f;
                      *sweep_done = 4;
                  }
              }
          }
      }
  }
}
#endif
