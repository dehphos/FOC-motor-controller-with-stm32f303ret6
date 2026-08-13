#ifndef ANALOG_VERI_OKUMA_H
#define ANALOG_VERI_OKUMA_H

#include "main.h"
#include <stdbool.h>
#include <math.h>

void Analog_Read_Currents(ADC_HandleTypeDef *hadc, bool simulate,
                          float_t *raw_Ia, float_t *raw_Ib, float_t *raw_Ic,
                          float_t *Ia_map, float_t *Ib_map, float_t *Ic_map,
                          float_t i_max);

void Analog_Calibrate_Offsets(ADC_HandleTypeDef *hadc, uint16_t calib_samples);

#endif /* ANALOG_VERI_OKUMA_H */
