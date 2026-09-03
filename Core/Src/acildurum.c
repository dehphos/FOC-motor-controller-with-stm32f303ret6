/**
 * @file    acildurum.c
 * @brief   Acil durum / arıza izleme ve alan zayıflatma (Field Weakening)
 *          eşiği kontrolünün uygulaması.
 */

#include "acildurum.h"
#include "math.h"
#include "control.h"



extern TIM_HandleTypeDef htim1;

/**
 * @brief  Motorun hizalama durumunu, arıza/acil durdurma koşullarını ve
 *         alan zayıflatma gerekliliğini kontrol eder; gerekirse motoru
 *         güvenli duruma sokar.
 *
 * @param  m  Kontrol edilecek motor yapısına işaretçi.
 *
 * @see    acildurum.h dosyasındaki fonksiyon açıklamasına bakınız.
 */
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
				if((!m->STATUS.STOPPED_FAULT && m->STATUS.HALL_ERROR_0 == 0 && m->STATUS.HALL_ERROR_7 == 0)){
					Align_Motor(m);
					m->STATUS.STOPPED_FAULT_COUNT = 0;
					m->REF.RPM_cur = 0;
					m->SPEED_PI_PARAMS.Speed_integral = 0;
					m->DQ_PI_PARAMS.Id_integral = 0;
					m->DQ_PI_PARAMS.Iq_integral = 0;
					m->SPEED_PI_PARAMS.E = 0;
					m->STATUS.HALL_ERROR_0 = 0;
					m->STATUS.HALL_ERROR_7 = 0;
					m->STATUS.STOPPED_FAULT = false;
					break;
				}
				HAL_Delay(100);
			}

		}
		if(m->PARAMS.MAX_RPM > MAX_WITHOUT_FW && fabsf(m->REF.RPM_cur) > MAX_WITHOUT_FW){
			m->PARAMS.FW = true;
			m->PARAMS.CIRCULAR_LIM = false;
		}else{
			m->PARAMS.FW = false;
			m->PARAMS.CIRCULAR_LIM = true;
		}
}
