#ifndef FOC_INTERRUPT_H
#define FOC_INTERRUPT_H

#include "main.h"
#include "math.h"
#include "control.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"


void Foc_Loop(motor *m, ADC_HandleTypeDef *hadc, float_t V_dc);

#endif /* FOC_INTERRUPT_H */
