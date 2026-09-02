/**
 * @file    foc_interrupt.h
 * @brief   FOC (Field Oriented Control) ana kontrol kesmesi ile ilgili
 *          bağımlılıkların toplandığı üst düzey (umbrella) header.
 *
 * @note    Gerçek kesme geri çağırımı olan
 *          `HAL_ADCEx_InjectedConvCpltCallback()` bu header'da değil,
 *          HAL kütüphanesinin enjekte edilmiş ADC arayüzünde tanımlıdır
 *          (bkz. foc_interrupt.c).
 */

#ifndef FOC_INTERRUPT_H
#define FOC_INTERRUPT_H

#include "main.h"
#include "math.h"
#include "control.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"
#include "hall_interrupt.h"


#endif /* FOC_INTERRUPT_H */
