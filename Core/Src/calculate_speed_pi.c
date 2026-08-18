#include "calculate_speed_pi.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "clampf.h"
#include "map.h"
#include "ramp.h"

void calculate_speed_pi(motor *MOTOR) {

	float_t RPM = MOTOR->REF.RPM_cur;
	if(fabsf(MOTOR->REF.RPM_cur) < 200 ) RPM = 0;
	MOTOR->REF.RPM = clampf(MOTOR->REF.RPM, -MOTOR->REF.RPM_lim, MOTOR->REF.RPM_lim);
	ramp(MOTOR);

	    float_t dt_speed = (float_t)MOTOR->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS / 1000.0f;
	    MOTOR->SPEED_PI_PARAMS.E = RPM - MOTOR->rotor_rpm;

	MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM = map((fabsf(MOTOR->REF.RPM_cur) * MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT) / (MOTOR->REF.RPM_lim * MOTOR->SPEED_PI_PARAMS.ki), 0.0f, (MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT/MOTOR->SPEED_PI_PARAMS.ki), 500.0f, 800.0f);

	float_t next_integral = MOTOR->SPEED_PI_PARAMS.Speed_integral + (( MOTOR->SPEED_PI_PARAMS.E) * dt_speed);
	float_t predicted_Iq = MOTOR->SPEED_PI_PARAMS.kp * ( MOTOR->SPEED_PI_PARAMS.E) + (MOTOR->SPEED_PI_PARAMS.ki * next_integral);

	if (!(predicted_Iq > MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT &&  MOTOR->SPEED_PI_PARAMS.E > 0.0f) && !(predicted_Iq < -MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT &&  MOTOR->SPEED_PI_PARAMS.E < 0.0f)) {
		MOTOR->SPEED_PI_PARAMS.Speed_integral = clampf(next_integral, -MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM, MOTOR->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM);
	}

	MOTOR->REF.Iq = clampf((MOTOR->SPEED_PI_PARAMS.kp *  MOTOR->SPEED_PI_PARAMS.E + (MOTOR->SPEED_PI_PARAMS.ki * MOTOR->SPEED_PI_PARAMS.Speed_integral)), -MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT, MOTOR->SPEED_PI_PARAMS.IQ_REF_LIMIT);
	MOTOR->REF.Id = 0.0f;

}
