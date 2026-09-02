/**
 * @file    map.h
 * @brief   Bir değeri bir aralıktan başka bir aralığa doğrusal olarak
 *          ölçekleyen (map) yardımcı fonksiyonu.
 */

#ifndef MAP_H
#define MAP_H

#include "main.h"
#include "math.h"

/**
 * @brief  Bir değeri [min_fm, max_fm] aralığından [min_to, max_to] aralığına
 *         doğrusal (lineer) olarak dönüştürür.
 *
 * @param  variable  Dönüştürülecek girdi değeri.
 * @param  min_fm    Girdi aralığının alt sınırı.
 * @param  max_fm    Girdi aralığının üst sınırı.
 * @param  min_to    Çıktı aralığının alt sınırı.
 * @param  max_to    Çıktı aralığının üst sınırı.
 *
 * @return Ölçeklenmiş (map edilmiş) değer. Girdi aralık dışına taşarsa
 *         sonuç da çıktı aralığının dışına taşabilir (kırpma yapılmaz).
 *
 * @note   Fonksiyon `static inline` olarak tanımlıdır; bu header'ı dahil eden
 *         her çeviri biriminde ayrı ayrı derlenir.
 */
static inline float_t map(float_t variable, float_t min_fm, float_t max_fm, float_t min_to, float_t max_to)
{
		  return ((variable - min_fm)/(max_fm - min_fm)*(max_to - min_to) + min_to);
}


#endif /* MAP_H */
