/**
 * @file    hall_interrupt.c
 * @brief   Hall sensör kenar geçişlerinde tetiklenen input-capture kesmesi:
 *          Hall periyodu ölçümü, dönüş yönü tespiti, rotor açısı/hız
 *          hesaplaması ve çok kademeli hız filtrelemesi.
 */

#include "hall_interrupt.h"
#include "control.h"
#include "math.h"
#include "map.h"
#include "clampf.h"

extern motor MOTOR_1;

/**
 * @brief  TIM3 input-capture (Hall sensör) kesmesi geri çağırım (callback)
 *         fonksiyonu. Her Hall kenar geçişinde tetiklenir.
 *
 * İşleyiş özeti:
 *  1. Yeni yakalanan zamanlayıcı değeri (`new_tim_raw`) periyot
 *     biriktiricisine (`period_accumulator`) eklenir; çok küçük (gürültü)
 *     aralıklar (< 20 sayım) yok sayılır (erken çıkış).
 *  2. Bitmiş olan Hall sektörüne ait asimetri düzeltme çarpanı
 *     (`hall_comp_lut`) uygulanarak gerçek periyot (`STATUS.period`)
 *     hesaplanır.
 *  3. Yeni Hall durumu (`hall_state`) GPIO giriş kaydından okunur.
 *  4. Önceki ve yeni Hall durumuna bakılarak dönüş yönü
 *     (`OBSERVER.hall_direction`, +1/-1) belirlenir.
 *  5. Yeni Hall durumuna karşılık gelen rotor açısı (0/60/.../300°) atanır;
 *     geçersiz durumlar (0/7) ilgili hata sayaçlarını artırır.
 *  6. Ölçülen periyottan anlık RPM (`inst_rpm`) hesaplanır, `MAX_RPM` ile
 *     sınırlanır ve geçmiş üç örnek (`prev_rpm`, `prev2_rpm`, `prev3_rpm`)
 *     güncellenir.
 *  7. Hıza bağlı adaptif bir alçak geçiren filtre (`alpha`/`beta`, hız ile
 *     `map()`'lenir) iki kademede uygulanarak `STATUS.rotor_rpm` (ve ondan
 *     türetilen `kama_rpm`) güncellenir.
 *
 * @param  htim  Kesmeyi tetikleyen zamanlayıcı handle'ı (yalnızca TIM3 için
 *               işlenir; ilgili motor `MOTOR_1`'dir).
 *
 * @note   Motor hizalanmamışsa (`m->STATUS.ALIGNED == false`) fonksiyon
 *         erken döner.
 * @warning Bu fonksiyon bir kesme (ISR) bağlamında çalışır; bloklayıcı
 *          çağrı içermemelidir.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
    motor *m = NULL;

    if (htim->Instance == TIM3) {
        m = &MOTOR_1;
    }
    // ileride 2. motor gelirse: else if (htim->Instance == TIM4) { m = &MOTOR_2; }

    if (m == NULL) return;
    if (!m->STATUS.ALIGNED) return;

    if (htim->Instance == TIM3)
    {
        uint32_t new_tim_raw = __HAL_TIM_GET_COMPARE(htim, m->OUT.A);
        if (new_tim_raw <= 0) new_tim_raw += 65536;

        static uint32_t period_accumulator = 0;
        period_accumulator += new_tim_raw;

        if (period_accumulator < 20) return;

        // 1. Önce HANGİ state'ten çıktığımızı (süresini ölçtüğümüz sektörü) bulalım
        uint8_t finished_state = m->STATUS.hall_state;

        // 3. LUT, biten sektörün (finished_state) kendi asimetrisini düzeltmelidir!
        m->STATUS.period = (uint32_t)((float_t)period_accumulator * m->PARAMS.hall_comp_lut[finished_state]);

        period_accumulator = 0;
        m->STATUS.last_hall_edge_tick = HAL_GetTick();
        m->STATUS.STOPPED = false;

        // 4. ŞİMDİ yeni state'i oku ve sisteme kaydet
        m->STATUS.hall_state = (m->IN.HALL.CHANNEL->IDR >> __builtin_ctz(m->IN.HALL.A)) & 0x07;


        if (m->OBSERVER.prev_hall != 0 && m->OBSERVER.prev_hall != m->STATUS.hall_state) {
            if ((m->OBSERVER.prev_hall == 1 && m->STATUS.hall_state == 3) || (m->OBSERVER.prev_hall == 3 && m->STATUS.hall_state == 2) ||
                (m->OBSERVER.prev_hall == 2 && m->STATUS.hall_state == 6) || (m->OBSERVER.prev_hall == 6 && m->STATUS.hall_state == 4) ||
                (m->OBSERVER.prev_hall == 4 && m->STATUS.hall_state == 5) || (m->OBSERVER.prev_hall == 5 && m->STATUS.hall_state == 1)) {
                m->OBSERVER.hall_direction = 1;
            } else if ((m->OBSERVER.prev_hall == 1 && m->STATUS.hall_state == 5) || (m->OBSERVER.prev_hall == 5 && m->STATUS.hall_state == 4) ||
                    (m->OBSERVER.prev_hall == 4 && m->STATUS.hall_state == 6) || (m->OBSERVER.prev_hall == 6 && m->STATUS.hall_state == 2) ||
                    (m->OBSERVER.prev_hall == 2 && m->STATUS.hall_state == 3) || (m->OBSERVER.prev_hall == 3 && m->STATUS.hall_state == 1)) {
                m->OBSERVER.hall_direction = -1;
            }
        }
        m->OBSERVER.prev_hall = m->STATUS.hall_state;

        switch(m->STATUS.hall_state){
            case 1 : m->STATUS.rotor_angle = 0;   break;
            case 2 : m->STATUS.rotor_angle = 120; break;
            case 3 : m->STATUS.rotor_angle = 60;  break;
            case 4 : m->STATUS.rotor_angle = 240; break;
            case 5 : m->STATUS.rotor_angle = 300; break;
            case 6 : m->STATUS.rotor_angle = 180; break;
            case 0 : m->STATUS.HALL_ERROR_0 += 1; break;
            case 7 : m->STATUS.HALL_ERROR_7 += 1; break;
            default: m->STATUS.STOPPED = true;    break;
        }

        m->STATUS.tim = m->STATUS.period;
		float_t inst_rpm = (float_t)m->OBSERVER.hall_direction * (10.0f * (float_t)TIM3_CNT_HZ) / ((float_t)m->STATUS.period * m->PARAMS.NUM_OF_POLE_PAIRS);

		inst_rpm = clampf(inst_rpm, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);

		m->OBSERVER.prev3_rpm = m->OBSERVER.prev2_rpm;
		m->OBSERVER.prev2_rpm = m->OBSERVER.prev_rpm;
		m->OBSERVER.prev_rpm = inst_rpm;


		float_t abs_inst = fabsf(inst_rpm);
		float_t alpha = clampf(map(abs_inst, 300.0f, 2000.0f, 0.1f, 0.7f), 0.1f, 0.7f);
		float_t beta  = 1.0f - alpha;

		m->OBSERVER.rpm_filter_stage1 = (m->OBSERVER.rpm_filter_stage1 * alpha) + (inst_rpm * beta);
		m->STATUS.rotor_rpm = (m->STATUS.rotor_rpm * alpha) + (m->OBSERVER.rpm_filter_stage1 * beta);
		m->STATUS.kama_rpm = m->STATUS.rotor_rpm / 4.5f;
		}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}
