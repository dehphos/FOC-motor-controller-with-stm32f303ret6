/**
 * @file    analog_veri_okuma.h
 * @brief   Faz akımlarının ADC üzerinden okunması, amper birimine
 *          ölçeklenmesi ve akım sensörü ofset kalibrasyonu için arayüz.
 */

#ifndef ANALOG_VERI_OKUMA_H
#define ANALOG_VERI_OKUMA_H

#include "main.h"
#include <stdbool.h>
#include <math.h>

/**
 * @brief  Üç faz akımını (Ia, Ib, Ic) enjekte edilmiş (injected) ADC
 *         kanallarından okur ve önceden belirlenmiş ofset değerleriyle
 *         birlikte amper cinsine ölçekler.
 *
 * @param  m         Akımları okunacak/güncellenecek motor yapısına işaretçi.
 * @param  simulate  `true` ise gerçek ADC yerine sabit orta nokta (2048)
 *                    değerleri kullanılır (simülasyon modu).
 * @param  i_max     Ölçeklemede kullanılacak akım tam skala (maksimum)
 *                    değeri [A].
 *
 * @note   Sonuçlar `m->STATUS.Ia_curr`/`Ib_curr`/`Ic_curr` (ham ADC) ve
 *         `m->STATUS.Ia_curr_map`/`Ib_curr_map`/`Ic_curr_map` (amper
 *         cinsinden) alanlarına yazılır.
 */
void Analog_Read_Currents(motor *m, bool simulate, float_t i_max);

/**
 * @brief  PWM çıkışlarını düşük bir değere sabitleyip belirli sayıda
 *         örnek toplayarak akım ölçüm devresinin (şönt) sıfır-akım
 *         ofsetlerini kalibre eder.
 *
 * @param  m              Ofset değerlerinin yazılacağı motor yapısına
 *                         işaretçi.
 * @param  calib_samples  Ortalaması alınacak örnek sayısı.
 *
 * @note   Kalibrasyon tamamlandığında `m->STATUS.READY = true` olarak
 *         ayarlanır.
 * @warning Fonksiyon, örnekler arasında 1 ms `HAL_Delay` içerir; bu nedenle
 *          bloklayıcıdır (blocking) ve başlangıç (init) aşamasında
 *          çağrılmalıdır.
 */
void Analog_Calibrate_Offsets(motor *m, uint16_t calib_samples);

#endif /* ANALOG_VERI_OKUMA_H */
