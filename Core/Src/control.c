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
    if(m->STATUS.ALIGNED) {


        // --- 1. RAM'DEN YEREL DEĞİŞKENLERE OKUMA (LOAD) ---
        float_t max_rpm = m->PARAMS.MAX_RPM;
        float_t ref_rpm = m->REF.RPM;

        // Hız referansını güvenli sınırlara çekip, ramp() için geri yazalım
        ref_rpm = clampf(ref_rpm, -max_rpm, max_rpm);
        m->REF.RPM = ref_rpm;
		if(fabsf(m->REF.RPM) < m->PARAMS.MIN_RPM) ref_rpm = 0;
        ramp(m); // Bu fonksiyon m->REF.RPM_cur değerini günceller

        // Kalan okumaları yapalım
        float_t rpm_cur        = m->REF.RPM_cur;
        float_t min_rpm        = m->PARAMS.MIN_RPM;
        float_t rotor_rpm      = m->STATUS.rotor_rpm;
        float_t kp             = m->PARAMS.SPEED_PI.kp;
        float_t ki             = m->PARAMS.SPEED_PI.ki;
        float_t iq_ref_limit   = m->PARAMS.SPEED_PI.IQ_REF_LIMIT;
        float_t speed_integral = m->PARAMS.SPEED_PI.Speed_integral;
        float_t mod_index      = m->DIAG.mod_index;
        bool    fw_active      = m->PARAMS.FW;

        // --- 2. YEREL MATEMATİKSEL HESAPLAMALAR (FPU REGISTERS) ---
        float_t rpm_target = rpm_cur;

        // Ölü bant kontrolü (if içi RAM okumalarından kurtarıldı)
        if(fabsf(rpm_cur) < min_rpm && fabsf(ref_rpm) < min_rpm) {
            rpm_target = 0.0f;
        }

        float_t error = rpm_target - rotor_rpm;
        float_t speed_integral_lim = iq_ref_limit / ki;

        if(mod_index <= 98.0f) {
            float_t next_integral = speed_integral + error;
            float_t predicted_iq  = (kp * error) + (ki * next_integral);

            // Anti-Windup Mantığı
            bool over_limit_pos = (predicted_iq > iq_ref_limit) && (error > 0.0f);
            bool over_limit_neg = (predicted_iq < -iq_ref_limit) && (error < 0.0f);

            if (!over_limit_pos && !over_limit_neg) {
                speed_integral = clampf(next_integral, -speed_integral_lim, speed_integral_lim);
            }
        }

        float_t final_iq_ref = clampf((kp * error) + (ki * speed_integral), -iq_ref_limit, iq_ref_limit);

        // --- 3. SONUÇLARI RAM'E TEK SEFERDE YAZMA (STORE) ---
        m->PARAMS.SPEED_PI.E = error;
        m->PARAMS.SPEED_PI.SPEED_INTEGRAL_LIM = speed_integral_lim;
        m->DIAG.speed_error = error;
        m->PARAMS.SPEED_PI.Speed_integral = speed_integral;
        m->REF.Iq = final_iq_ref;

        if(!fw_active) {
            m->REF.Id = 0.0f;
        }
    }
}

/**
 * @brief  Hedef hıza (REF.RPM_cur) göre Hız PI kazançlarını (kp/ki) 0-1
 *         arası bir "blend" katsayısıyla kademeli olarak karıştırır.
 *
 * @details LOW_SPEED_RPM_THRESH ve üzerinde blend=0 (tamamen normal/yüksek
 *          hız kazançları), 0 RPM'de blend=1 (tamamen düşük hız kazançları)
 *          olacak şekilde lineer interpolasyon yapar. rotor_rpm yerine
 *          REF.RPM_cur kullanılır çünkü rampalanmış referans, ölçülen
 *          hızdan çok daha az gürültülü/kararlıdır.
 */
static inline float_t speed_gain_blend(motor *m) {
	float_t thresh = m->PARAMS.SPEED_PI.LOW_SPEED_RPM_THRESH;
	if (thresh <= 0.0f) return 0.0f;
	float_t blend = 1.0f - (fabsf(m->REF.RPM_cur) / thresh);
	return clampf(blend, 0.0f, 1.0f);
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

    float_t bemf_y = -m->OBSERVER.E_alpha_est;
    float_t bemf_x = m->OBSERVER.E_beta_est;

    // Motor geri dönüyorsa indüklenen voltaj ters döner!
    // Vektörü 180 derece geri çevirerek gerçek açıyı buluyoruz.
    if (m->OBSERVER.hall_direction < 0) {
        bemf_y = -bemf_y;
        bemf_x = -bemf_x;
    }

    float_t prev_angle_rad = m->OBSERVER.observer_angle_rad;
    m->OBSERVER.observer_angle_rad = fast_atan2f(bemf_y, bemf_x);

    float_t delta_theta = m->OBSERVER.observer_angle_rad - prev_angle_rad;

    if (delta_theta > PI) {
        delta_theta -= 2.0f * PI;
    } else if (delta_theta < -PI) {
        delta_theta += 2.0f * PI;
    }

    // Kutup çiftin 2 olduğu için doğrudan çarpımla 95492.965f olarak sabitledik.
    float_t observer_rpm_raw = delta_theta * 95492.965f;

    observer_rpm_raw = clampf(observer_rpm_raw, -15000.0f, 15000.0f);

    m->DIAG.observer_rpm = (m->DIAG.observer_rpm * 0.7f) + (observer_rpm_raw * 0.3f);

    // Filtrelenmiş nihai değeri de ekstra bir güvenlik olarak sınırla
    m->DIAG.observer_rpm = clampf(m->DIAG.observer_rpm, -15000.0f, 15000.0f);

    // Ham Açı (Derece cinsinden)
    float_t raw_angle_deg = (m->OBSERVER.observer_angle_rad * 180.0f) * ONE_BY_PI;


    // --- YENİ KOMPANZASYON BLOĞU BURADAN BAŞLIYOR ---

    // 1. Mutlak (Absolute) açısal hız ile faz gecikmesini (pozitif olarak) bul
    float_t abs_rpm = fabsf(m->STATUS.rotor_rpm);
    float_t w_e_abs = abs_rpm * 0.1047197f * m->PARAMS.NUM_OF_POLE_PAIRS;

    // Gecikme daima pozitif bir derecedir
    float_t phase_lag_rad = fast_atan2f(w_e_abs * 0.0005f, 1.0f);
    float_t phase_lag_deg = phase_lag_rad * 57.29578f;

    // 2. Motorun yönüne göre DOĞRU kompanzasyonu yap
    float_t corrected_angle_deg = raw_angle_deg;

//    if (m->OBSERVER.hall_direction >= 0) {
//        corrected_angle_deg += phase_lag_deg;
//    } else {
//        corrected_angle_deg -= phase_lag_deg;
//    }

    corrected_angle_deg += phase_lag_deg;
    // 3. PLL Düzeltmesini ekle
    corrected_angle_deg += m->PARAMS.ERROR_PI.output;

    // 4. 0-360 Derece Sınırlandırması
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
    // --- 1. RAM'DEN YEREL DEĞİŞKENLERE OKUMA (LOAD) ---
    // Bayraklar ve Motor Parametreleri
    bool    use_ff       = m->PARAMS.FF;
    bool    circular_lim = m->PARAMS.CIRCULAR_LIM;
    float_t rotor_rpm    = m->STATUS.rotor_rpm;
    float_t pole_pairs   = m->PARAMS.NUM_OF_POLE_PAIRS;
    float_t ls           = m->PARAMS.Ls;
    float_t psi_m        = m->PARAMS.psi_m;

    // Anlık Akımlar ve Referanslar
    float_t iq_curr      = m->STATUS.Iq_curr;
    float_t id_curr      = m->STATUS.Id_curr;
    float_t iq_ref       = m->REF.Iq;
    float_t id_ref       = m->REF.Id;

    // Q-Ekseni (Tork) PI Katsayıları
    float_t iq_kp        = m->PARAMS.DQ_PI.Iq_kp;
    float_t iq_ki        = m->PARAMS.DQ_PI.Iq_ki;
    float_t iq_integral  = m->PARAMS.DQ_PI.Iq_integral;

    // D-Ekseni (Akı) PI Katsayıları
    float_t id_kp        = m->PARAMS.DQ_PI.Id_kp;
    float_t id_ki        = m->PARAMS.DQ_PI.Id_ki;
    float_t id_integral  = m->PARAMS.DQ_PI.Id_integral;

    // --- 2. YEREL MATEMATİKSEL HESAPLAMALAR (FPU REGISTERS) ---
    float_t omega_e = 0.0f;
    float_t vd_ff   = 0.0f;
    float_t vq_ff   = 0.0f;

    if(use_ff) {
        // (PI / 30.0f) bölmesi statik float değere (0.104719755f) çevrildi
        omega_e = rotor_rpm * 0.104719755f * pole_pairs;
        vd_ff   = -omega_e * ls * iq_curr;
        vq_ff   = (omega_e * ls * id_curr) + (omega_e * psi_m);
    }

    float_t bara_gerilimi = fmaxf(0.0f, V_dc - fabsf(vq_ff));

    // Q Ekseni (Tork) Hesabı
    float_t iq_err = iq_ref - iq_curr;
    float_t iq_integral_lim = bara_gerilimi / iq_ki;

    iq_integral += iq_err;
    iq_integral = clampf(iq_integral, -iq_integral_lim, iq_integral_lim);
    float_t out_eq = (iq_kp * iq_err) + (iq_ki * iq_integral);

    // D Ekseni (Mıknatıslanma/Flux) Hesabı
    float_t id_err = id_ref - id_curr;
    float_t id_integral_lim = bara_gerilimi / id_ki;

    id_integral += id_err;
    id_integral = clampf(id_integral, -id_integral_lim, id_integral_lim);
    float_t out_ed = (id_kp * id_err) + (id_ki * id_integral);

    // İleri Besleme (Feed-Forward) Eklemesi
    out_ed += vd_ff;
    out_eq += vq_ff;

    // Voltaj Limitasyon (Dairesel ve V_rms Sınırları)
    float_t v_rms;
    if(circular_lim) {
        v_rms = V_dc * ONE_BY_SQRT3;
    } else {
        v_rms = V_dc;
    }

    out_ed = clampf(out_ed, -v_rms, v_rms);

    // Donanımsal karekök (__builtin_sqrtf) hesaplaması
    float_t eq_max = __builtin_sqrtf((v_rms * v_rms) - (out_ed * out_ed));
    out_eq = clampf(out_eq, -eq_max, eq_max);

    // --- 3. SONUÇLARI RAM'E TEK SEFERDE YAZMA (STORE) ---
    m->PARAMS.omega_e       = omega_e;
    m->PARAMS.DQ_PI.Vd_ff   = vd_ff;
    m->PARAMS.DQ_PI.Vq_ff   = vq_ff;

    m->PARAMS.DQ_PI.Iq_E    = iq_err;
    m->DIAG.iq_error        = iq_err;
    m->PARAMS.DQ_PI.Iq_integral_lim = iq_integral_lim;
    m->PARAMS.DQ_PI.Iq_integral     = iq_integral;

    m->PARAMS.DQ_PI.Id_E    = id_err;
    m->DIAG.id_error        = id_err;
    m->PARAMS.DQ_PI.Id_integral_lim = id_integral_lim;
    m->PARAMS.DQ_PI.Id_integral     = id_integral;

    m->OUT.E_d = out_ed;
    m->OUT.E_q = out_eq;
}



