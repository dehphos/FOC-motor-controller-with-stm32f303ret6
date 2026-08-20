#include "control.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "clampf.h"
#include "map.h"
extern TIM_HandleTypeDef htim1;

#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f
#define SQRT3_BY_2   0.866025403f
#define PI 3.14159265359f


#define V_dc 28.0f
#define I_max 33.132f

void calculate_speed_pi(motor *MOTOR) {

	float_t RPM = MOTOR->REF.RPM_cur;
	if(fabsf(MOTOR->REF.RPM_cur) < 200.0f && MOTOR->REF.RPM <200.0f) RPM = 0.0f;
	MOTOR->REF.RPM = clampf(MOTOR->REF.RPM, -MOTOR->REF.RPM_lim, MOTOR->REF.RPM_lim);
	ramp(MOTOR);
	MOTOR->SPEED_PI_PARAMS.E = RPM - MOTOR->rotor_rpm;
	MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM = MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT / MOTOR->SPEED_PI_PARAMS.ki;
	float_t next_integral = MOTOR->SPEED_PI_PARAMS.Speed_integral + MOTOR->SPEED_PI_PARAMS.E;

	float_t predicted_Iq = (MOTOR->SPEED_PI_PARAMS.kp * MOTOR->SPEED_PI_PARAMS.E) + (MOTOR->SPEED_PI_PARAMS.ki * next_integral);
	if (!(predicted_Iq > MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT && MOTOR->SPEED_PI_PARAMS.E > 0.0f) &&
	    !(predicted_Iq < -MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT && MOTOR->SPEED_PI_PARAMS.E < 0.0f)) {
		MOTOR->SPEED_PI_PARAMS.Speed_integral = clampf(next_integral, -MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM, MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM);
	}

	MOTOR->REF.Iq = clampf((MOTOR->SPEED_PI_PARAMS.kp * MOTOR->SPEED_PI_PARAMS.E) + (MOTOR->SPEED_PI_PARAMS.ki * MOTOR->SPEED_PI_PARAMS.Speed_integral), -MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT, MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT);
	MOTOR->REF.Id = 0.0f;
}

void clarke_park(float_t Ia, float_t Ib, float_t sin_theta, float_t cos_theta, float_t *Id, float_t *Iq)
{

    float_t I_alpha = Ia;
    float_t I_beta  = (Ia * ONE_BY_SQRT3) + (Ib * TWO_BY_SQRT3);

    *Id =  (I_alpha * cos_theta) + (I_beta * sin_theta);
    *Iq = -(I_alpha * sin_theta) + (I_beta * cos_theta);
}

void inv_clarke_park(float_t Vd, float_t Vq, float_t sin_theta, float_t cos_theta, float_t *Va, float_t *Vb, float_t *Vc)
{

    float_t V_alpha = (Vd * cos_theta) - (Vq * sin_theta);
    float_t V_beta  = (Vd * sin_theta) + (Vq * cos_theta);

    *Va = V_alpha;
    *Vb = (-0.5f * V_alpha) + (SQRT3_BY_2 * V_beta);
    *Vc = (-0.5f * V_alpha) - (SQRT3_BY_2 * V_beta);
}


void ramp(motor *MOTOR) {

    if (MOTOR->REF.RPM > MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur += MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur > MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
        }
    }
    else if (MOTOR->REF.RPM < MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur -= MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur < MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
        }
    }
}

void Align_Motor(motor *m)
{
    m->ALIGNED = false;

		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, 972.331472f);
		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, 827.678589f);
		__HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, 827.678589);

    HAL_Delay(1000);


    uint8_t hA = HAL_GPIO_ReadPin(m->IN.HAL.CHANNEL, m->IN.HAL.A);
    uint8_t hB = HAL_GPIO_ReadPin(m->IN.HAL.CHANNEL, m->IN.HAL.B);
    uint8_t hC = HAL_GPIO_ReadPin(m->IN.HAL.CHANNEL, m->IN.HAL.C);
    uint8_t observed_state = (hC << 2) | (hB << 1) | hA;

    uint16_t observed_angle;
    switch (observed_state) {
        case 1: observed_angle = 0;   break;
        case 2: observed_angle = 120; break;
        case 3: observed_angle = 60;  break;
        case 4: observed_angle = 240; break;
        case 5: observed_angle = 300; break;
        case 6: observed_angle = 180; break;
        case 0: m->HALL_ERROR_0++; break;
        case 7: m->HALL_ERROR_7++; break;
    }

//    m->HALL_OFSET = (uint16_t)(((int32_t)(360 - observed_angle) + m->HALL_SECTOR_OFFSET + 360) % 360);
    m->rotor_angle = observed_angle;
    m->rotor_angle_interp = observed_angle;
    m->last_hall_edge_tick = HAL_GetTick();
    m->STOPPED = true;
    m->ALIGNED = true;

}
