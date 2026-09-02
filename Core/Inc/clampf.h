/**
 * @file    clampf.h
 * @brief   Kayan noktalı (float) bir değeri belirli bir [lo, hi] aralığına
 *          sınırlayan (clamp/saturate) yardımcı fonksiyonu.
 */

#ifndef CLAMPF_H
#define CLAMPF_H

#include "main.h"
#include "math.h"
#include "control.h"

/**
 * @brief  Verilen değeri [lo, hi] aralığında sınırlar (saturasyon).
 *
 * @param  x   Sınırlanacak girdi değeri.
 * @param  lo  İzin verilen alt sınır.
 * @param  hi  İzin verilen üst sınır.
 *
 * @return `x < lo` ise `lo`, `x > hi` ise `hi`, aksi halde `x` döner.
 */
static inline float_t clampf(float_t x, float_t lo, float_t hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}


#endif /* CLAMPF_H */
