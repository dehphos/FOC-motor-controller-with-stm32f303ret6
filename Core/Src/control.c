/**
 * @file    control.c
 * @brief   Hız PI regülatörü ve motor hizalama (Align) fonksiyonlarının
 *          uygulaması.
 */

#include "control.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "clampf.h"
#include "map.h"


/**
 * @brief  Hız PI regülatörünü çalıştırır: hız referansını rampalar, hız
 *         hatasını (`SPEED_PI_PARAMS.E`) hesaplar ve anti-windup korumalı
 *         integral hesabıyla Iq (moment) referansını (`REF.Iq`) üretir.
 *
 * İşleyiş:
 *  - Referans hız [-MAX_RPM, MAX_RPM] aralığına kırpılır ve `ramp()` ile
 *    yumuşatılır.
 *  - `MIN_RPM` altındaki hem hedef hem de anlık referans hızlar sıfır kabul
 *    edilir (ölü bant / deadband).
 *  - Öngörülen (predicted) Iq çıkışı doyum (saturasyon) sınırını aşacaksa ve
 *    hata bu doyumu büyütecek yöndeyse, integral biriktirici dondurulur
 *    (anti-windup).
 *  - Alan zayıflatma (`PARAMS.FW`) aktif değilse `REF.Id` sıfırlanır.
 *
 * @param  m  Üzerinde işlem yapılacak motor yapısına işaretçi.
 *
 * @note   Motor hizalanmamışsa (`m->STATUS.ALIGNED == false`) fonksiyon
 *         hiçbir işlem yapmaz.
 */
void calculate_speed_pi(motor *m) {
	if(m->STATUS.ALIGNED){
	m->REF.RPM = clampf(m->REF.RPM, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);
	ramp(m);

	float_t RPM = m->REF.RPM_cur;
	if(fabsf(m->REF.RPM_cur) < m->PARAMS.MIN_RPM && fabsf(m->REF.RPM) < m->PARAMS.MIN_RPM){
		RPM = 0.0f;
	}
	m->SPEED_PI_PARAMS.E = RPM - m->STATUS.rotor_rpm;
	m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM =(m->SPEED_PI_PARAMS.IQ_REF_LIMIT / m->SPEED_PI_PARAMS.ki) * 1.2;


	float_t next_integral = m->SPEED_PI_PARAMS.Speed_integral + m->SPEED_PI_PARAMS.E;
	float_t predicted_Iq = (m->SPEED_PI_PARAMS.kp * m->SPEED_PI_PARAMS.E) + (m->SPEED_PI_PARAMS.ki * next_integral);
	if (!(predicted_Iq > m->SPEED_PI_PARAMS.IQ_REF_LIMIT && m->SPEED_PI_PARAMS.E > 0.0f) &&
		!(predicted_Iq < -m->SPEED_PI_PARAMS.IQ_REF_LIMIT && m->SPEED_PI_PARAMS.E < 0.0f)) {
		m->SPEED_PI_PARAMS.Speed_integral = clampf(next_integral, -m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM, m->SPEED_PI_PARAMS.SPEED_INTEGRAL_LIM);
	}


	m->REF.Iq = clampf((m->SPEED_PI_PARAMS.kp * m->SPEED_PI_PARAMS.E) + (m->SPEED_PI_PARAMS.ki * m->SPEED_PI_PARAMS.Speed_integral),
						   -m->SPEED_PI_PARAMS.IQ_REF_LIMIT, m->SPEED_PI_PARAMS.IQ_REF_LIMIT);
	if(!(m->PARAMS.FW)){
		m->REF.Id = 0.0f;
	}

}}


/**
 * @brief  Motoru bilinen bir elektriksel pozisyona sürerek hizalar, ardından
 *         Hall sensör pinlerini okuyup gözlenen Hall durumuna karşılık gelen
 *         başlangıç rotor açısını belirler ve motor durumunu günceller.
 *
 * İşleyiş:
 *  1. `m->STATUS.ALIGNED` geçici olarak `false` yapılır.
 *  2. Sabit bir gerilim vektörü `pwm_write()` ile uygulanır ve rotorun bu
 *     pozisyona oturması için 1000 ms beklenir.
 *  3. Hall A/B/C pinleri okunur, 3 bitlik `observed_state` oluşturulur ve
 *     bu duruma karşılık gelen açı (0/60/120/180/240/300°) belirlenir.
 *  4. Geçersiz durumlar (0 veya 7) ilgili hata sayaçlarını artırır ve açı
 *     0° olarak varsayılır.
 *  5. `rotor_angle`, `rotor_angle_interp`, `last_hall_edge_tick` güncellenir
 *     ve `STOPPED` = true, `ALIGNED` = true olarak ayarlanır.
 *
 * @param  m  Hizalanacak motor yapısına işaretçi.
 *
 * @note   Fonksiyon içinde 1000 ms'lik bir `HAL_Delay` bulunur; bu nedenle
 *         zaman kritik (interrupt) bağlamdan çağrılmamalıdır.
 */
void Align_Motor(motor *m)
{
    m->STATUS.ALIGNED = false;

		pwm_write(m, 972.331472f, 827.678589f, 827.678589f);

    HAL_Delay(1000);


    uint8_t hA = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.A);
    uint8_t hB = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.B);
    uint8_t hC = HAL_GPIO_ReadPin(m->IN.HALL.CHANNEL, m->IN.HALL.C);
    uint8_t observed_state = (hC << 2) | (hB << 1) | hA;

    uint16_t observed_angle;
    switch (observed_state) {
        case 1: observed_angle = 0;   break;
        case 2: observed_angle = 120; break;
        case 3: observed_angle = 60;  break;
        case 4: observed_angle = 240; break;
        case 5: observed_angle = 300; break;
        case 6: observed_angle = 180; break;
        case 0: m->STATUS.HALL_ERROR_0++; observed_angle = 0; break;
        case 7: m->STATUS.HALL_ERROR_7++; observed_angle = 0; break;
        default: m->STATUS.HALL_ERROR_7++;m->STATUS.HALL_ERROR_0++;observed_angle = 0; break;
    }

//    m->HALL_OFSET = (uint16_t)(((int32_t)(360 - observed_angle) + m->HALL_SECTOR_OFFSET + 360) % 360);
    m->STATUS.rotor_angle = observed_angle;
    m->STATUS.rotor_angle_interp = observed_angle;
    m->STATUS.last_hall_edge_tick = HAL_GetTick();
    m->STATUS.STOPPED = true;
    m->STATUS.ALIGNED = true;

}
