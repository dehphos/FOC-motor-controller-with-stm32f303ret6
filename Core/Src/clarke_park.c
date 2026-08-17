#include "ramp.h"
#include "math.h"


#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f
#define SQRT3_BY_2   0.866025403f
#define PI 3.14159265359f


#define V_dc 28.0f
#define I_max 33.132f


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
