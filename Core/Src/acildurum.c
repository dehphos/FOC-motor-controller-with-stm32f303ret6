#include "acildurum.h"
#include "math.h"
#include "control.h"
extern TIM_HandleTypeDef htim1;

void acildurum(motor *m){
		if(!m->STATUS.ALIGNED){
		  Align_Motor(m);
		};
		if(m->STATUS.STOPPED_FAULT || m->STATUS.STOPPED_FAULT_COUNT > 100000 || m->STATUS.HALL_ERROR_0 > 0 || m->STATUS.HALL_ERROR_7 > 0)
		{
			m->STATUS.STOPPED_FAULT = true;
			m->STATUS.ALIGNED = false;
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, 900);
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, 900);
			__HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, 900);
			while(1) {
				if(!m->STATUS.STOPPED_FAULT && m->STATUS.HALL_ERROR_0 == 0 && m->STATUS.HALL_ERROR_7 == 0){
					Align_Motor(m);
					m->STATUS.STOPPED_FAULT_COUNT = 0;
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

