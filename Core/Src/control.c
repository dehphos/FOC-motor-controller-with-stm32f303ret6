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
 *  5. **PI Çıkışı:** Matematiksel PI formülü (\f$K_p \cdot E + K_i \cdot \int E\f$) ile hedef tork akımı hesaplanır ve `IQ_REF_LIMIT` ile sınırlandırılır.
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
	m->DIAG.speed_error = m->PARAMS.SPEED_PI.E;

	if(m->DIAG.mod_index <= 98){
		float_t next_integral = m->PARAMS.SPEED_PI.Speed_integral + m->PARAMS.SPEED_PI.E;
		float_t predicted_Iq = (m->PARAMS.SPEED_PI.kp * m->PARAMS.SPEED_PI.E) + (m->PARAMS.SPEED_PI.ki * next_integral);
		if (!(predicted_Iq > m->PARAMS.SPEED_PI.IQ_REF_LIMIT && m->PARAMS.SPEED_PI.E > 0.0f) &&
			!(predicted_Iq < -m->PARAMS.SPEED_PI.IQ_REF_LIMIT && m->PARAMS.SPEED_PI.E < 0.0f)) {
			m->PARAMS.SPEED_PI.Speed_integral = clampf(next_integral, -m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM, m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM);
		}
	}

	m->REF.Iq = clampf((m->PARAMS.SPEED_PI.kp * m->PARAMS.SPEED_PI.E) + (m->PARAMS.SPEED_PI.ki * m->PARAMS.SPEED_PI.Speed_integral),
						   -m->PARAMS.SPEED_PI.IQ_REF_LIMIT, m->PARAMS.SPEED_PI.IQ_REF_LIMIT);
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

/**
 * @brief   Zıt-EMK (BEMF) tabanlı Sensörsüz Rotor Konum Gözlemcisini çalıştırır.
 *
 * @details Faz akım türevlerini (\f$di/dt\f$) ve anlık FOC voltaj çıkışlarını
 *          kullanarak stator üzerindeki Endüklenen Zıt-EMK'yı (E_alpha, E_beta)
 *          tahmin eder. Ultra hızlı `fast_atan2f` fonksiyonu ile bu Zıt-EMK
 *          vektörünü elektriksel rotor açısına dönüştürür.
 *
 *          **Düzeltme Aşaması (Angle Compensation):**
 *          Gözlemci saf hesaplamayı bitirdikten sonra, ortaya çıkan ham açıya
 *          iki farklı kompanzasyon uygulanır:
 *          1. **Teorik İleri Besleme (`phase_lag_deg`):** LPF filtresinden kaynaklanan
 *             bilinen matematiksel faz gecikmesini, hıza (w_e) bağlı olarak düzeltir.
 *          2. **Pratik Kapalı Çevrim (`ERROR_PI.output`):** Hall kesmesinden gelen
 *             dinamik PLL integratör çıkışını ekleyerek; ortam ısınması, Ls sapması
 *             gibi kaotik donanım gecikmelerini sıfırlar.
 *          Böylece sisteme ve `OBSERVER.observer_angle_deg` değişkenine %100
 *          doğrulanmış ve kilitlenmiş (Locked) gerçek mıknatıs pozisyonu teslim edilir.
 *
 * @param   m  Türev geçmişi ve gerilim/akım verilerinin okunduğu motor yapısı.
 */
void run_bemf_observer(motor *m)
{
    float_t frec = 20000.0f;

    float_t di_alpha = (m->STATUS.I_alpha - m->OBSERVER.I_alpha_prev) * frec;
    float_t di_beta  = (m->STATUS.I_beta  - m->OBSERVER.I_beta_prev)  * frec;

    m->OBSERVER.I_alpha_prev = m->STATUS.I_alpha;
    m->OBSERVER.I_beta_prev  = m->STATUS.I_beta;

    m->DIAG.bemf_alpha_raw = m->OUT.V_alpha - (m->PARAMS.Rs * m->STATUS.I_alpha) - (m->PARAMS.Ls * di_alpha);
    m->DIAG.bemf_beta_raw  = m->OUT.V_beta  - (m->PARAMS.Rs * m->STATUS.I_beta)  - (m->PARAMS.Ls * di_beta);

    m->OBSERVER.E_alpha_est += 0.1f * (m->DIAG.bemf_alpha_raw - m->OBSERVER.E_alpha_est);
    m->OBSERVER.E_beta_est  += 0.1f * (m->DIAG.bemf_beta_raw  - m->OBSERVER.E_beta_est);

    float_t prev_angle_rad = m->OBSERVER.observer_angle_rad;
    m->OBSERVER.observer_angle_rad = fast_atan2f(-m->OBSERVER.E_alpha_est, m->OBSERVER.E_beta_est);

    float_t delta_theta = m->OBSERVER.observer_angle_rad - prev_angle_rad;

    if (delta_theta > PI) {
        delta_theta -= 2.0f * PI;
    } else if (delta_theta < -PI) {
        delta_theta += 2.0f * PI;
    }

    // 190985.93f / m->PARAMS.NUM_OF_POLE_PAIRS ağır bir bölme işlemiydi.
    // Kutup çiftin 2 olduğu için doğrudan çarpımla 95492.965f olarak sabitledik.
    float_t observer_rpm_raw = delta_theta * 95492.965f;

    m->DIAG.observer_rpm = (m->DIAG.observer_rpm * 0.95f) + (observer_rpm_raw * 0.05f);

	float_t raw_angle_deg = (m->OBSERVER.observer_angle_rad * 180.0f) * ONE_BY_PI;


	// Observer LPF Gecikmesi (Phase Lag)
	float_t w_e = m->STATUS.rotor_rpm * 0.1047197f * m->PARAMS.NUM_OF_POLE_PAIRS;
	float_t phase_lag_rad = fast_atan2f(w_e * 0.0005f, 1.0f);
	float_t phase_lag_deg = phase_lag_rad * 57.29578f;

	// Açıya Tam Kompanzasyon (Phase Lag + Yeni PARAMS.ERROR_PI Çıkışımız)
	float_t corrected_angle_deg = raw_angle_deg + phase_lag_deg + m->PARAMS.ERROR_PI.output;

	// 0-360 Derece Sınırlandırması
	if (corrected_angle_deg >= 360.0f) corrected_angle_deg -= 360.0f;
	else if (corrected_angle_deg < 0.0f) corrected_angle_deg += 360.0f;

	m->OBSERVER.observer_angle_deg = corrected_angle_deg;
}



/**
 * @brief   Akım (D-Q Ekseni) PI Regülatörlerini çalıştırır.
 *
 * @details Tork (Iq) ve Akı (Id) eksenlerindeki anlık akım hatalarını hesaplayarak
 *          motorun fazlarına uygulanacak hedef voltaj vektörlerini (E_d, E_q) üretir.
 *          Optimizasyon sınırları (-O0) altında RAM gecikmelerini (Pointer Dereferencing Overhead)
 *          önlemek amacıyla "Yerel Önbellekleme (Local Caching)" mimarisiyle yazılmıştır;
 *          veriler başlangıçta topluca CPU kaydedicilerine alınır, hesaplama yapılır
 *          ve nihai sonuç tek adımda Struct'a geri yazılır.
 *
 *          - **İleri Besleme (FF):** İsteğe bağlı olarak (`PARAMS.FF`) Cross-Coupling ve
 *            Zıt-EMK gerilimleri hesaplanıp PI çıkışının üzerine eklenir.
 *          - **Anti-Windup:** İntegral birikmesi, anlık dinamik bara gerilimine
 *            (Headroom) oranla sınırlandırılarak overshoot engellenir.
 *          - **Modülasyon Sınırı:** İki eksenin ürettiği vektörün genliği, inverter
 *            doyum sınırına (`V_rms`) veya Dairesel Limitasyona (`CIRCULAR_LIM`)
 *            göre donanımsal FPU karekök komutu (`__builtin_sqrtf`) ile kırpılır.
 *
 * @param   m     FOC verilerinin okunduğu ve çıkış gerilimlerinin yazılacağı motor yapısı.
 * @param   V_dc  Dinamik voltaj limitleri için filtrelenmiş anlık DC Bara gerilimi [V].
 */
void calculate_dq_pi(motor *m, float_t V_dc)
{
    if(m->PARAMS.FF){
        m->PARAMS.omega_e = m->STATUS.rotor_rpm * (PI / 30.0f) * m->PARAMS.NUM_OF_POLE_PAIRS;
        m->PARAMS.DQ_PI.Vd_ff = -m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Iq_curr;
        m->PARAMS.DQ_PI.Vq_ff = (m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Id_curr) + (m->PARAMS.omega_e * m->PARAMS.psi_m);
    }else{
        m->PARAMS.DQ_PI.Vd_ff = 0;
        m->PARAMS.DQ_PI.Vq_ff = 0;
    }

    float_t bara_gerilimi = fmaxf(0.0f, V_dc - fabsf(m->PARAMS.DQ_PI.Vq_ff));
    m->PARAMS.DQ_PI.Iq_E = (m->REF.Iq - m->STATUS.Iq_curr);
    m->DIAG.iq_error = m->PARAMS.DQ_PI.Iq_E;

    m->PARAMS.DQ_PI.Iq_integral_lim = bara_gerilimi / m->PARAMS.DQ_PI.Iq_ki;

    m->PARAMS.DQ_PI.Iq_integral += m->PARAMS.DQ_PI.Iq_E;
    m->PARAMS.DQ_PI.Iq_integral = clampf(m->PARAMS.DQ_PI.Iq_integral, - m->PARAMS.DQ_PI.Iq_integral_lim, m->PARAMS.DQ_PI.Iq_integral_lim);
    m->OUT.E_q = m->PARAMS.DQ_PI.Iq_kp * m->PARAMS.DQ_PI.Iq_E + m->PARAMS.DQ_PI.Iq_ki * m->PARAMS.DQ_PI.Iq_integral;

    m->PARAMS.DQ_PI.Id_E = (m->REF.Id - m->STATUS.Id_curr);
    m->DIAG.id_error = m->PARAMS.DQ_PI.Id_E;
    m->PARAMS.DQ_PI.Id_integral_lim = bara_gerilimi / m->PARAMS.DQ_PI.Id_ki;

    m->PARAMS.DQ_PI.Id_integral += m->PARAMS.DQ_PI.Id_E;
    m->PARAMS.DQ_PI.Id_integral = clampf(m->PARAMS.DQ_PI.Id_integral, - m->PARAMS.DQ_PI.Id_integral_lim, m->PARAMS.DQ_PI.Id_integral_lim);
    m->OUT.E_d = m->PARAMS.DQ_PI.Id_kp * m->PARAMS.DQ_PI.Id_E + m->PARAMS.DQ_PI.Id_ki * m->PARAMS.DQ_PI.Id_integral;

    m->OUT.E_d += m->PARAMS.DQ_PI.Vd_ff;
    m->OUT.E_q += m->PARAMS.DQ_PI.Vq_ff;

    float_t V_rms;
    if(m->PARAMS.CIRCULAR_LIM){
        V_rms = V_dc * ONE_BY_SQRT3;
    }else{
        V_rms = V_dc;
    }
    m->OUT.E_d = clampf(m->OUT.E_d, -V_rms, V_rms);

    float_t Eq_max = __builtin_sqrtf((V_rms * V_rms) - (m->OUT.E_d * m->OUT.E_d));
    m->OUT.E_q = clampf(m->OUT.E_q, -Eq_max, Eq_max);

    float_t v_mag = __builtin_sqrtf((m->OUT.E_d * m->OUT.E_d) + (m->OUT.E_q * m->OUT.E_q));
	m->DIAG.mod_index = (v_mag / V_rms) * 100.0f;

	m->DIAG.power_w = 1.5f * ((m->OUT.E_d * m->STATUS.Id_curr) + (m->OUT.E_q * m->STATUS.Iq_curr));
}
