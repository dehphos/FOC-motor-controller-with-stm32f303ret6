/**
 * @file    hall_interrupt.h
 * @brief   Hall sensör giriş yakalama (input capture) kesmesi ile ilgili
 *          bağımlılıkların toplandığı üst düzey (umbrella) header.
 *
 * @note    Gerçek kesme geri çağırımı olan `HAL_TIM_IC_CaptureCallback()`
 *          bu header'da değil, HAL kütüphanesinin zamanlayıcı input-capture
 *          arayüzünde tanımlıdır (bkz. hall_interrupt.c).
 */

#ifndef HALL_INTERRUPT_H
#define HALL_INTERRUPT_H

#include "main.h"
#include "math.h"
#include "control.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"
#include "hall_interrupt.h"


#endif /* HALL_INTERRUPT_H */
