#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include "math.h"

void calculate_speed_pi(motor *m);

void clarke_park(float_t Ia, float_t Ib, float_t sin_theta, float_t cos_theta, float_t *Id, float_t *Iq);

void inv_clarke_park(float_t Vd, float_t Vq, float_t sin_theta, float_t cos_theta, float_t *Va, float_t *Vb, float_t *Vc);

void ramp(motor *m);

void Align_Motor(motor *m);


#endif /* CONTROL_H */
