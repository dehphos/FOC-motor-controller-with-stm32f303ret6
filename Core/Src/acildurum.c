#include "acildurum.h"
#include "math.h"
#include "control.h"
extern TIM_HandleTypeDef htim1;

void acildurum(motor *m){
		if(!m->ALIGNED){
		  Align_Motor(m);
		};
		if(m->STOPPED_FAULT || m->STOPPED_FAULT_COUNT > 20000 || m->HALL_ERROR_0 > 0 || m->HALL_ERROR_7 > 0)
		{
			m->STOPPED_FAULT = true;
			m->ALIGNED = false;
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, 900);
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, 900);
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, 900);
			while(1) {
				if(!m->STOPPED_FAULT && m->HALL_ERROR_0 == 0 && m->HALL_ERROR_7 == 0){
					Align_Motor(m);
					m->STOPPED_FAULT_COUNT = 0;
					m->REF.RPM_cur = 0;
					m->SPEED_PI_PARAMS.Speed_integral = 0;
					m->DQ_PI_PARAMS.Id_integral = 0;
					m->DQ_PI_PARAMS.Iq_integral = 0;
					m->SPEED_PI_PARAMS.E = 0;
					break;
				}
				HAL_Delay(100);
			}

		}
}

