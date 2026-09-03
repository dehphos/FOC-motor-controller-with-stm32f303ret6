/**
 * @file    test.c
 * @brief   Genel hız tarama (sweep) testi: hız referansını 0'dan
 *          `MAX_RPM`'e, oradan `-MAX_RPM`'e ve tekrar 0'a adım adım
 *          sürerek sistemin geçici (transient) tepkisini gözlemlemek için
 *          kullanılır. Sadece `TEST` makrosu tanımlıysa derlenir.
 */

#include "main.h"
#include "math.h"
#include "control.h"
#include "foc_interrupt.h"
#include "hall_interrupt.h"
#include "test.h"

#if TEST
/**
 * @brief  Hız referansını durum makinesi ile kademeli olarak tarayan test
 *         fonksiyonu.
 *
 * Durum makinesi (`*sweep_done` üzerinden):
 *  - **State 0:** `system_start_tick`'ten 5000 ms sonra başlar; her 20 ms'de
 *    bir `REF.RPM` değeri 10 RPM artırılarak `MAX_RPM`'e çıkarılır.
 *  - **State 1:** Tepe noktadan itibaren her 2000 ms'de bir `REF.RPM`,
 *    `MAX_RPM/10` kadar azaltılarak `-MAX_RPM`'e indirilir.
 *  - **State 2:** Her 20 ms'de bir `REF.RPM` 10 RPM artırılarak tekrar
 *    0'a çekilir; `PARAMS.FF` (feed-forward) açıksa kapatılıp tüm tarama
 *    baştan (state 0) tekrarlanır, kapalıysa test tamamlanmış
 *    (`*sweep_done = 4`) sayılır.
 *
 * @param  m                  Test edilecek motor yapısına işaretçi.
 * @param  sweep_started      Taramanın başlayıp başlamadığını tutan bayrak
 *                             (giriş/çıkış parametresi).
 * @param  sweep_done         Tarama durum makinesinin aşaması
 *                             (0/1/2 = devam ediyor, 4 = tamamlandı).
 * @param  new_tim            Kullanılmıyor (arayüz uyumluluğu için tutulan
 *                             parametre).
 * @param  sweep_last_tick    Bir önceki durum geçişinin zaman damgası.
 * @param  system_start_tick  Sistemin/testin başlangıç zaman damgası.
 *
 * @note   Motor hizalanmamışsa veya arıza durumundaysa
 *         (`!ALIGNED || STOPPED_FAULT`) fonksiyon hiçbir işlem yapmadan
 *         döner.
 */
void test(motor *m, uint8_t *sweep_started, uint8_t *sweep_done, uint16_t *new_tim, uint32_t *sweep_last_tick, uint32_t system_start_tick)
{
    if (!m->STATUS.ALIGNED || m->STATUS.STOPPED_FAULT) {
        return;
    }
    if (*sweep_done == 0)
    {
        if (!(*sweep_started))
        {
            if (HAL_GetTick() - system_start_tick >= 5000)
            {
                *sweep_started = 1;
                m->REF.RPM = 0.0f;
                *sweep_last_tick = HAL_GetTick();
            }
        }
        else
        {
            if (HAL_GetTick() - *sweep_last_tick >= 20)
            {
                *sweep_last_tick = HAL_GetTick();

                if (m->REF.RPM < m->PARAMS.MAX_RPM-1500) {
                    m->REF.RPM += 20.0f;
                } else {
                    *sweep_done = 1;
                }
            }
        }
    }
    // STATE 1: Tepe noktadan hızı sertçe düşür
    else if (*sweep_done == 1)
    {
        if (HAL_GetTick() - *sweep_last_tick >= 2000)
        {
            *sweep_last_tick = HAL_GetTick();

            if (m->REF.RPM > -(m->PARAMS.MAX_RPM - 1500)) {
                m->REF.RPM -= (m->PARAMS.MAX_RPM -1500) / 10.0f;
            } else {
                *sweep_done = 2;
            }
        }
    }
    // STATE 2: Hızı tekrar 0'a çek ve testleri sıfırla/ilerlet
    else if (*sweep_done == 2)
    {
        if (HAL_GetTick() - *sweep_last_tick >= 20)
        {
            *sweep_last_tick = HAL_GetTick();

            if (m->REF.RPM < 0.0f) {
                m->REF.RPM += 20.0f;
            }
            else
            {
				m->REF.RPM = 0.0f;
				*sweep_done = 4;

            }
        }
    }
}
#endif
