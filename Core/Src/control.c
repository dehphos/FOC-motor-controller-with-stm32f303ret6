/**
 * @file    control.c
 * @brief   Hız PI regülatörü, Akım (DQ) PI regülatörü ve motor hizalama (Align)
 *          fonksiyonlarının matematiksel uygulamaları.
 *
 * @details Bu dosya, servo sistemin "Beyin" kısmını oluşturur. FOC kesmesinden
 *          soyutlanmış bu fonksiyonlar; referans rampalama, anti-windup korumaları,
 *          dinamik voltaj limiti (headroom) hesaplamaları ve ileri besleme (FF)
 *          kompanzasyonlarını içerir.
 */
#include "control.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "clampf.h"
#include "map.h"


/**
 * @brief  Hız PI (Proportional-Integral) regülatörünü çalıştırır.
 *
 * @details Referans devir komutunu rampalayarak alır ve motorun anlık devriyle
 *          karşılaştırarak Q ekseni (Tork) için referans akım (`REF.Iq`) üretir.
 *
 * **İşleyiş:**
 *  1. **Rampalama:** Referans hız donanım sınırlarına (`MAX_RPM`) göre kırpılır ve mekanik şokları önlemek için belirlenen adımda (`REF.STEP`) rampalanır.
 *  2. **Ölü Bant:** Referans ve anlık hız, ölü bant (`MIN_RPM`) sınırları içindeyse sıfır kabul edilir.
 *  3. **Hata Hesabı:** Hız hatası (`E`) hesaplanır ve integral biriktiriciye eklenir.
 *  4. **Anti-Windup:** Eğer hesaplanan hedef akım (`predicted_Iq`) doyum limitini aşıyorsa ve hata bu aşımı destekleyecek yöndeyse, integralin büyümesi dondurulur (Overshoot engellenir).
 *  5. **PI Çıkışı:** Matematiksel PI formülü ($K_p \cdot E + K_i \cdot \int E$) ile hedef tork akımı hesaplanır ve `IQ_REF_LIMIT` ile sınırlandırılır.
 *  6. **Alan Zayıflatma:** İlgili bayrak (`PARAMS.FW`) aktif değilse Id (Akı) referansı sıfırlanır.
 *
 * @param  m  Üzerinde işlem yapılacak motor yapısına işaretçi.
 */
void calculate_speed_pi(motor *m) {
	if(m->STATUS.ALIGNED){
	m->REF.RPM = clampf(m->REF.RPM, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);
	ramp(m);

	float_t RPM = m->REF.RPM_cur;
	if(fabsf(m->REF.RPM_cur) < m->PARAMS.MIN_RPM && fabsf(m->REF.RPM) < m->PARAMS.MIN_RPM){
		RPM = 0.0f;
	}
	m->PARAMS.SPEED_PI.E = RPM - m->STATUS.rotor_rpm;
	m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM =(m->PARAMS.SPEED_PI.IQ_REF_LIMIT / m->PARAMS.SPEED_PI.ki);


	float_t next_integral = m->PARAMS.SPEED_PI.Speed_integral + m->PARAMS.SPEED_PI.E;
	float_t predicted_Iq = (m->PARAMS.SPEED_PI.kp * m->PARAMS.SPEED_PI.E) + (m->PARAMS.SPEED_PI.ki * next_integral);
	if (!(predicted_Iq > m->PARAMS.SPEED_PI.IQ_REF_LIMIT && m->PARAMS.SPEED_PI.E > 0.0f) &&
		!(predicted_Iq < -m->PARAMS.SPEED_PI.IQ_REF_LIMIT && m->PARAMS.SPEED_PI.E < 0.0f)) {
		m->PARAMS.SPEED_PI.Speed_integral = clampf(next_integral, -m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM, m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM);
	}


	m->REF.Iq = clampf((m->PARAMS.SPEED_PI.kp * m->PARAMS.SPEED_PI.E) + (m->PARAMS.SPEED_PI.ki * m->PARAMS.SPEED_PI.Speed_integral),
						   -m->PARAMS.SPEED_PI.IQ_REF_LIMIT, m->PARAMS.SPEED_PI.IQ_REF_LIMIT);
	if(!(m->PARAMS.FW)){
		m->REF.Id = 0.0f;
	}

}}


/**
 * @brief  Akım döngüsü için D ve Q ekseni PI kontrolcülerini hesaplar.
 *
 * @details Sistemdeki elektriksel gecikmeleri ve zıt motor gerilimini
 *          (BEMF) yenmek için Feed-Forward voltajları ve PI hesaplamaları yürütülür.
 *
 * **İşleyiş:**
 *  1. **İleri Besleme Öngörüsü:** Özellik aktifse (`FF`), motorun elektriksel hızına bağlı olarak D ve Q eksenlerindeki zıt-EMK ve endüktif kuplaj gerilimleri (`Vd_ff`, `Vq_ff`) hesaplanır.
 *  2. **Dinamik Headroom (Boşluk) Hesabı:** DC baranın (`V_dc`) ileri besleme tarafından işgal edilen kısmı mutlak değerle (`fabsf`) hesaplanıp çıkarılarak PI kontrolcüsünün kullanabileceği net voltaj boşluğu (`avail_vd`, `avail_vq`) bulunur. Sıfır altı düşüşler `fmaxf` ile engellenir.
 *  3. **Dinamik Limitasyon:** İntegral limitleri, sadece yukarıda hesaplanan net voltaj boşluğuna kadar şişebilecek şekilde daraltılır (Yüksek hızda PWM doyumu önlenir).
 *  4. **PI Hesabı:** D ve Q ekseni akım hataları hesaplanıp çıkış komutları (`E_d`, `E_q`) üretilir.
 *  5. **Toplama ve Dairesel Sınır:** PI çıkışlarına FF gerilimleri eklenir ve sonuç, uzay vektörü genliğine ($V_{dc} / \sqrt{3}$) göre dairesel olarak kırpılır.
 *
 * @param  m     Motor yapısına işaretçi.
 * @param  V_dc  Anlık ölçülen DC bara voltajı (Limit hesaplamaları için).
 */
void calculate_dq_pi(motor *m, float_t V_dc){
    // ==============================================================================
    // İleri Besleme (Feed Forward)
    // ==============================================================================
    if(m->PARAMS.FF){
        m->PARAMS.omega_e = m->STATUS.rotor_rpm * (PI / 30.0f) * m->PARAMS.NUM_OF_POLE_PAIRS;
        m->PARAMS.DQ_PI.Vd_ff = -m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Iq_curr;
        m->PARAMS.DQ_PI.Vq_ff = (m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Id_curr) + (m->PARAMS.omega_e * m->PARAMS.psi_m);
    }else{
    	m->PARAMS.DQ_PI.Vd_ff = 0;
    	m->PARAMS.DQ_PI.Vq_ff = 0;
    }


    // ==============================================================================
    // Akım PI Döngüleri
    // ==============================================================================

    float_t bara_gerilimi = fmaxf(0.0f, V_dc - fabsf(m->PARAMS.DQ_PI.Vq_ff)); // 0-V_dc arası elde bara gerilimi
    m->PARAMS.DQ_PI.Iq_E = (m->REF.Iq - m->STATUS.Iq_curr);
    m->PARAMS.DQ_PI.Iq_integral_lim = bara_gerilimi / m->PARAMS.DQ_PI.Iq_ki;
    m->PARAMS.DQ_PI.Iq_integral += m->PARAMS.DQ_PI.Iq_E;
    m->PARAMS.DQ_PI.Iq_integral = clampf(m->PARAMS.DQ_PI.Iq_integral, - m->PARAMS.DQ_PI.Iq_integral_lim, m->PARAMS.DQ_PI.Iq_integral_lim);
    m->OUT.E_q = m->PARAMS.DQ_PI.Iq_kp * m->PARAMS.DQ_PI.Iq_E + m->PARAMS.DQ_PI.Iq_ki * m->PARAMS.DQ_PI.Iq_integral;

    m->PARAMS.DQ_PI.Id_E = (m->REF.Id - m->STATUS.Id_curr);
    m->PARAMS.DQ_PI.Id_integral_lim = bara_gerilimi / m->PARAMS.DQ_PI.Id_ki;
    m->PARAMS.DQ_PI.Id_integral += m->PARAMS.DQ_PI.Id_E;
    m->PARAMS.DQ_PI.Id_integral = clampf(m->PARAMS.DQ_PI.Id_integral, - m->PARAMS.DQ_PI.Id_integral_lim, m->PARAMS.DQ_PI.Id_integral_lim);
    m->OUT.E_d = m->PARAMS.DQ_PI.Id_kp * m->PARAMS.DQ_PI.Id_E + m->PARAMS.DQ_PI.Id_ki * m->PARAMS.DQ_PI.Id_integral;


    // ==============================================================================
    // FF ile PI toplamları
    // ==============================================================================

    m->OUT.E_d += m->PARAMS.DQ_PI.Vd_ff;
    m->OUT.E_q += m->PARAMS.DQ_PI.Vq_ff;

    // ==============================================================================
    // Bara Voltajı (DC-Link) Sınırlaması
    // ==============================================================================
    float_t V_rms;
    if(m->PARAMS.CIRCULAR_LIM){
        V_rms = V_dc * ONE_BY_SQRT3;
    }else{
        V_rms = V_dc;
    }
    m->OUT.E_d = clampf(m->OUT.E_d, -V_rms, V_rms);
    float_t Eq_max = sqrtf((V_rms * V_rms) - (m->OUT.E_d * m->OUT.E_d));
    m->OUT.E_q = clampf(m->OUT.E_q, -Eq_max, Eq_max);
}

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
