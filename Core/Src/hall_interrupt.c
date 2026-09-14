/**
 * @file    hall_interrupt.c
 * @brief   Hall sensör kenar geçişlerinde tetiklenen input-capture kesmesi:
 *          Hall periyodu ölçümü, dönüş yönü tespiti, rotor açısı/hız
 *          hesaplaması ve çok kademeli hız filtrelemesi.
 */

#include "hall_interrupt.h"


extern motor MOTOR_1;

/**
 * @brief   Hall sensör kenar geçişlerinde (Input Capture) tetiklenen zamanlayıcı kesmesi.
 *
 * @details Bu fonksiyon motorun hız ve açısal konumunu tespit etmenin yanı sıra,
 *          Gözlemci (Observer) devredeyken Kapalı Çevrim İntegral Kontrolcüsü (PLL)
 *          olarak çalışır. İşlemci yükünü hafifletmek için hıza bağlı iki farklı akış sunar:
 *
 *          1. **Düşük Hız (Gözlemci Pasif, Blend < 1.0):** Hall sensör periyotlarını ölçer,
 *             asimetri düzeltmesi uygular, dönüş yönünü hesaplar ve alçak geçiren
 *             filtre (LPF) ile kaba hız (`hall_rpm`) üretir. İşlemci yükü yüksektir.
 *
 *          2. **Yüksek Hız / Bypass Kapısı (Gözlemci Aktif, Blend >= 1.0):** Optimizasyon
 *             yasağı (-O0) altında ana FOC döngüsünü aksatmamak için ağır matematiksel
 *             işlemleri atlar. Sadece Hall sensör açısı ile Gözlemcinin tahmin ettiği
 *             açı arasındaki "Kalan Hatayı (Residual Error)" ölçer. Bu hatayı donanım
 *             zaman farkı (`dt`) ile çarparak bir PLL (Phase Locked Loop) integratörünü
 *             (`ERROR_PI.integral`) besler. Böylece termal direnç değişimleri (\f$R_s\f$) ve
 *             donanım gecikmeleri sıfır maliyetle dinamik olarak kompanze edilir.
 *
 * @param   htim  Kesmeyi tetikleyen zamanlayıcı donanım işaretçisi (Yalnızca TIM3 işlenir).
 *
 * @note    Bu fonksiyon içinde C standart kütüphane bölmeleri (`/`) yerine,
 *          önceden hesaplanmış FPU çarpımları (Örn: `* 0.000002f`) kullanılarak
 *          clock cycle tasarrufu sağlanmıştır.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t start_cycles = DWT->CYCCNT;
    motor *m = NULL;

    if (htim->Instance == TIM3) {
        m = &MOTOR_1;
    }

    if (m == NULL) {
        start_cycles = 0;
        return;
    }

    if (!m->STATUS.ALIGNED){
        uint32_t end_cycles = DWT->CYCCNT;
        m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
        return;
    }

    if (htim->Instance == TIM3)
    {
        uint32_t new_tim_raw = __HAL_TIM_GET_COMPARE(htim, m->OUT.A);
        if (new_tim_raw <= 0) new_tim_raw += 65536;

        static uint32_t period_accumulator = 0;
        period_accumulator += new_tim_raw;

        if (period_accumulator < 20){
            uint32_t end_cycles = DWT->CYCCNT;
            m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
            return;
        }

        // --- 1. LOAD: yalnızca bu ISR'ye özel (foc tarafından dokunulmayan) alanlar ---
        uint32_t eski_periyot   = m->STATUS.period;
        float_t  hall_jitter    = m->DIAG.hall_period_jitter;
        uint8_t  prev_hall      = m->OBSERVER.prev_hall;
        int8_t   hall_direction = m->OBSERVER.hall_direction;
        uint16_t hall_error_0   = m->STATUS.HALL_ERROR_0;
        uint16_t hall_error_7   = m->STATUS.HALL_ERROR_7;
        uint16_t rotor_angle    = m->STATUS.rotor_angle;
        float_t  prev_rpm       = m->OBSERVER.prev_rpm;
        float_t  prev2_rpm      = m->OBSERVER.prev2_rpm;
        float_t  rpm_filter_s1  = m->OBSERVER.rpm_filter_stage1;
        float_t  hall_rpm       = m->STATUS.hall_rpm;

        // --- 2. HESAPLAMA ---
        uint32_t period = period_accumulator;
        bool stopped_local = false; // bu blokta STOPPED sadece false'a çekiliyor

        if (eski_periyot > 0 && !m->STATUS.STOPPED) {  // paylaşılan alan: anlık oku
            float_t anlik_jitter = fabsf((float_t)period - (float_t)eski_periyot);
            hall_jitter = (hall_jitter * 0.95f) + (anlik_jitter * 0.05f);
        }

        period_accumulator = 0;
        uint32_t now_tick = HAL_GetTick();

        uint8_t hall_state = (m->IN.HALL.CHANNEL->IDR >> __builtin_ctz(m->IN.HALL.A)) & 0x07;

        if (prev_hall != 0 && prev_hall != hall_state) {
            if ((prev_hall == 1 && hall_state == 3) || (prev_hall == 3 && hall_state == 2) ||
                (prev_hall == 2 && hall_state == 6) || (prev_hall == 6 && hall_state == 4) ||
                (prev_hall == 4 && hall_state == 5) || (prev_hall == 5 && hall_state == 1)) {
                hall_direction = 1;
            } else if ((prev_hall == 1 && hall_state == 5) || (prev_hall == 5 && hall_state == 4) ||
                    (prev_hall == 4 && hall_state == 6) || (prev_hall == 6 && hall_state == 2) ||
                    (prev_hall == 2 && hall_state == 3) || (prev_hall == 3 && hall_state == 1)) {
                hall_direction = -1;
            }
        }
        prev_hall = hall_state;

        switch(hall_state){
            case 1 : rotor_angle = 0;   break;
            case 2 : rotor_angle = 120; break;
            case 3 : rotor_angle = 60;  break;
            case 4 : rotor_angle = 240; break;
            case 5 : rotor_angle = 300; break;
            case 6 : rotor_angle = 180; break;
            case 0 : hall_error_0 += 1; break;
            case 7 : hall_error_7 += 1; break;
            default: stopped_local = true; break; // aşağıda paylaşımlı yazılacak
        }

        // --- 3. STORE (bloka özel, ortak alanlar) ---
        m->STATUS.period           = period;
        m->DIAG.hall_period_jitter = hall_jitter;
        m->STATUS.last_hall_edge_tick = now_tick;
        m->STATUS.hall_state       = hall_state;
        m->OBSERVER.prev_hall      = prev_hall;
        m->OBSERVER.hall_direction = hall_direction;
        m->STATUS.rotor_angle      = rotor_angle;
        m->STATUS.HALL_ERROR_0     = hall_error_0;
        m->STATUS.HALL_ERROR_7     = hall_error_7;
        m->STATUS.STOPPED          = false; // PAYLAŞILAN: anında yaz
        if (stopped_local) m->STATUS.STOPPED = true; // default case -> anında yaz

        // ==========================================================
        // YÜKSEK HIZ BYPASS KAPISI (PLL) — ERROR_PI paylaşımlı, anında oku/yaz
        // ==========================================================
        if (m->DIAG.blend_factor >= 1.0f) {
            float_t true_hall = (float_t)rotor_angle + m->PARAMS.HALL_OFSET;
            if (hall_direction < 0) true_hall += 60.0f;
            if (true_hall >= 360.0f) true_hall -= 360.0f;

            float_t diff = true_hall - m->OBSERVER.observer_angle_deg;
            if (diff > 180.0f) diff -= 360.0f;
            else if (diff < -180.0f) diff += 360.0f;

            m->PARAMS.ERROR_PI.error = diff;

            float_t dt = (float_t)new_tim_raw * 0.000002f;
            float_t integral = m->PARAMS.ERROR_PI.integral + (diff * m->PARAMS.ERROR_PI.ki * dt);
            integral = clampf(integral, -m->PARAMS.ERROR_PI.integral_lim, m->PARAMS.ERROR_PI.integral_lim);
            m->PARAMS.ERROR_PI.integral = integral;
            m->PARAMS.ERROR_PI.output = integral + (diff * m->PARAMS.ERROR_PI.kp);

            if (new_tim_raw < 833) {
//                m->STATUS.tim               = (uint16_t)period;
        		m->STATUS.inv_tim = 1.0f / (float_t)period;
                uint32_t end_cycles = DWT->CYCCNT;
                m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
                return;
            }
        }

        // ==========================================================
        // DÜŞÜK HIZ: AĞIR HESAPLAMALAR (tamamen bu ISR'ye özel)
        // ==========================================================
        float_t inst_rpm = ((float_t)hall_direction * 2500000.0f) / (float_t)period;
        inst_rpm = clampf(inst_rpm, -15000.0f, 15000.0f);

        float_t prev3_rpm_new = prev2_rpm;
        float_t prev2_rpm_new = prev_rpm;
        float_t prev_rpm_new  = inst_rpm;

        float_t abs_inst = fabsf(inst_rpm);
//      float_t alpha = clampf(map(abs_inst, 300.0f, 2000.0f, 0.1f, 0.7f), 0.1f, 0.7f);
        float_t alpha = ((abs_inst - 300.0f) * 0.000352941f) + 0.1f;
		alpha = clampf(alpha, 0.1f, 0.7f);

        float_t beta  = 1.0f - alpha;

        rpm_filter_s1 = (rpm_filter_s1 * alpha) + (inst_rpm * beta);
        hall_rpm = (hall_rpm * alpha) + (rpm_filter_s1 * beta);
        hall_rpm = clampf(hall_rpm, -15000.0f, 15000.0f);

        // --- 4. STORE: ağır hesaplama sonuçları (tek seferde) ---
        m->STATUS.tim               = (uint16_t)period;
        m->OBSERVER.prev3_rpm       = prev3_rpm_new;
        m->OBSERVER.prev2_rpm       = prev2_rpm_new;
        m->OBSERVER.prev_rpm        = prev_rpm_new;
        m->STATUS.inst_rpm          = inst_rpm;
        m->OBSERVER.rpm_filter_stage1 = rpm_filter_s1;
        m->STATUS.hall_rpm          = hall_rpm;
		m->STATUS.inv_tim = 1.0f / (float_t)period; // Bölmeyi burada 1 kez yapıyoruz!
    }

    uint32_t end_cycles = DWT->CYCCNT;
    m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
}
