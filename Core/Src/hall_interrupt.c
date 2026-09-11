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


//        uint8_t finished_state = m->STATUS.hall_state;
        uint32_t eski_periyot = m->STATUS.period;

        // 2. LUT Asimetri düzeltmesi uygula
//        m->STATUS.period = (uint32_t)((float_t)period_accumulator * m->PARAMS.hall_comp_lut[finished_state]);
        m->STATUS.period = (uint32_t)((float_t)period_accumulator);


        if (eski_periyot > 0 && !m->STATUS.STOPPED) {
			float_t anlik_jitter = fabsf((float_t)m->STATUS.period - (float_t)eski_periyot);
			m->DIAG.hall_period_jitter = (m->DIAG.hall_period_jitter * 0.95f) + (anlik_jitter * 0.05f);
		}

        period_accumulator = 0;
        m->STATUS.last_hall_edge_tick = HAL_GetTick(); // Timeout süresini burada da sıfırla
        m->STATUS.STOPPED = false;

        // 3. Yeni state'i oku ve sisteme kaydet
        m->STATUS.hall_state = (m->IN.HALL.CHANNEL->IDR >> __builtin_ctz(m->IN.HALL.A)) & 0x07;

        // 4. Dönüş Yönü (Direction) Tespiti
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

        // 5. Hall State'e göre Saf Açı (Rotor Angle) ataması
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

        // ==========================================================
		// YÜKSEK HIZ BYPASS KAPISI: ZAMAN-BAĞIMSIZ PI KONTROLCÜ (PLL)
		// ==========================================================
		if (m->DIAG.blend_factor >= 1.0f) {
			float_t true_hall = (float_t)m->STATUS.rotor_angle + m->PARAMS.HALL_OFSET;
			if (true_hall >= 360.0f) true_hall -= 360.0f;

			// Kalan Hatayı (Error) Hesapla
			float_t diff = true_hall - m->OBSERVER.observer_angle_deg;
			if (diff > 180.0f) diff -= 360.0f;
			else if (diff < -180.0f) diff += 360.0f;

			m->PARAMS.ERROR_PI.error = diff;

			// Bölme yerine 1 cycle Çarpma Optimizasyonu
			float_t dt = (float_t)new_tim_raw * 0.000002f;

			// İntegral (I) Kısmı ve Anti-Windup
			m->PARAMS.ERROR_PI.integral += m->PARAMS.ERROR_PI.error * m->PARAMS.ERROR_PI.ki * dt;
			m->PARAMS.ERROR_PI.integral = clampf(m->PARAMS.ERROR_PI.integral,
												 -m->PARAMS.ERROR_PI.integral_lim,
												  m->PARAMS.ERROR_PI.integral_lim);

			// Oransal (P) Kısmı ve Çıkış
			float_t proportional_term = m->PARAMS.ERROR_PI.error * m->PARAMS.ERROR_PI.kp;
			m->PARAMS.ERROR_PI.output = m->PARAMS.ERROR_PI.integral + proportional_term;

			// ERKEN UYANIŞ: İşlemciyi sadece 3000 RPM üzerindeyken rahatlat!
			// Donanım Timer periyodunu (new_tim_raw) doğrudan kontrol etmek en güvenlisidir.
			// 2500000 / 3000 RPM = 833 ticks. Eğer periyot 833'ten kısaysa motor 3000 RPM'den hızlıdır.	if (new_tim_raw < 833) {
			if (new_tim_raw < 833) {
				uint32_t end_cycles = DWT->CYCCNT;
				m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
				return;
			}
		}

		// ==========================================================
		// DÜŞÜK HIZ: AĞIR HESAPLAMALAR
		// ==========================================================
		m->STATUS.tim = m->STATUS.period;

		// Sabitleri önceden çarparak (10 * 500.000 / 2) tek bölme
		float_t inst_rpm = ((float_t)m->OBSERVER.hall_direction * 2500000.0f) / (float_t)m->STATUS.period;
		inst_rpm = clampf(inst_rpm, -15000.0f, 15000.0f);

		m->OBSERVER.prev3_rpm = m->OBSERVER.prev2_rpm;
		m->OBSERVER.prev2_rpm = m->OBSERVER.prev_rpm;
		m->OBSERVER.prev_rpm = inst_rpm;
		m->STATUS.inst_rpm = inst_rpm;

		float_t abs_inst = fabsf(inst_rpm);

		float_t alpha = clampf(map(abs_inst, 300.0f, 2000.0f, 0.1f, 0.7f), 0.1f, 0.7f);
		float_t beta  = 1.0f - alpha;

		m->OBSERVER.rpm_filter_stage1 = (m->OBSERVER.rpm_filter_stage1 * alpha) + (inst_rpm * beta);

		// Yeni değişkenimize (hall_rpm) kaydediyoruz:
		m->STATUS.hall_rpm = (m->STATUS.hall_rpm * alpha) + (m->OBSERVER.rpm_filter_stage1 * beta);

		// 17 Milyon hatasına karşı Hard-Limit savunması:
		m->STATUS.hall_rpm = clampf(m->STATUS.hall_rpm, -15000.0f, 15000.0f);
	}

    uint32_t end_cycles = DWT->CYCCNT;
	m->DIAG.hall_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
}
