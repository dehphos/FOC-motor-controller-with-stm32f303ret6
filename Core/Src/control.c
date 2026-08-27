#include "control.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "clampf.h"
#include "map.h"
extern TIM_HandleTypeDef htim1;


void calculate_speed_pi(motor *m) {
	if(m->STATUS.ALIGNED){
	float_t RPM = m->REF.RPM_cur;
	if(fabsf(m->REF.RPM_cur) < m->PARAMS.MIN_RPM && fabsf(m->REF.RPM) <m->PARAMS.MIN_RPM){
		RPM = 0.0f;
	}
	m->REF.RPM = clampf(m->REF.RPM, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);
	ramp(m);
	m->SPEED_PI_PARAMS.E = RPM - m->STATUS.rotor_rpm;
	m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM =(m->SPEED_PI_PARAMS.IQ_REF_LIMIT / m->SPEED_PI_PARAMS.ki);


	float_t next_integral = m->SPEED_PI_PARAMS.Speed_integral + m->SPEED_PI_PARAMS.E;
	float_t predicted_Iq = (m->SPEED_PI_PARAMS.kp * m->SPEED_PI_PARAMS.E) + (m->SPEED_PI_PARAMS.ki * next_integral);
	if (!(predicted_Iq > m->SPEED_PI_PARAMS.IQ_REF_LIMIT && m->SPEED_PI_PARAMS.E > 0.0f) &&
		!(predicted_Iq < -m->SPEED_PI_PARAMS.IQ_REF_LIMIT && m->SPEED_PI_PARAMS.E < 0.0f)) {
		m->SPEED_PI_PARAMS.Speed_integral = clampf(next_integral, -m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM, m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM);
	}


	m->REF.Iq = clampf((m->SPEED_PI_PARAMS.kp * m->SPEED_PI_PARAMS.E) + (m->SPEED_PI_PARAMS.ki * m->SPEED_PI_PARAMS.Speed_integral),
						   -m->SPEED_PI_PARAMS.IQ_REF_LIMIT, m->SPEED_PI_PARAMS.IQ_REF_LIMIT);
	if(!(m->PARAMS.FW)){
		m->REF.Id = 0.0f;
	}

}}



void Align_Motor(motor *m)
{
    m->STATUS.ALIGNED = false;

		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, 972.331472f);
		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, 827.678589f);
		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, 827.678589);

    HAL_Delay(1000);


    uint8_t hA = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.A);
    uint8_t hB = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.B);
    uint8_t hC = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.C);
    uint8_t observed_state = (hC << 2) | (hB << 1) | hA;

    uint16_t observed_angle;
    switch (observed_state) {
        case 1: observed_angle = 0;   break;
        case 2: observed_angle = 120; break;
        case 3: observed_angle = 60;  break;
        case 4: observed_angle = 240; break;
        case 5: observed_angle = 300; break;
        case 6: observed_angle = 180; break;
        case 0: m->STATUS.HALL_ERROR_0++; observed_angle = 0; break;
        case 7: m->STATUS.HALL_ERROR_7++; observed_angle = 0; break;
        default: m->STATUS.HALL_ERROR_7++;m->STATUS.HALL_ERROR_0++;observed_angle = 0; break;
    }

//    m->HALL_OFSET = (uint16_t)(((int32_t)(360 - observed_angle) + m->HALL_SECTOR_OFFSET + 360) % 360);
    m->STATUS.rotor_angle = observed_angle;
    m->STATUS.rotor_angle_interp = observed_angle;
    m->STATUS.last_hall_edge_tick = HAL_GetTick();
    m->STATUS.STOPPED = true;
    m->STATUS.ALIGNED = true;

}
