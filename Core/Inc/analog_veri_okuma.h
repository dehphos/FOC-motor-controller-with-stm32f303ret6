
#ifndef ANALOG_VERI_OKUMA_H
#define ANALOG_VERI_OKUMA_H

#include "main.h"
#include "map.h"

void Analog_Read_Currents(motor *m, bool simulate);

void Analog_Calibrate_Offsets(motor *m, uint16_t calib_samples);

#endif /* ANALOG_VERI_OKUMA_H */
