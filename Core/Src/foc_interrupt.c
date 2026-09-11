/**
 * @file    foc_interrupt.c
 * @brief   Alan Yönlendirmeli Kontrol (FOC) donanım ISR (Interrupt Service Routine) akışı.
 */
#include "foc_interrupt.h"


extern motor MOTOR_1;
extern volatile float_t V_dc;
extern float_t VBUS_DIVIDER_RATIO;
extern DAC_HandleTypeDef hdac1;

extern void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val);

//__attribute__((section(".ccmram")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
    uint32_t start_cycles = DWT->CYCCNT;
    motor *m = NULL;
    m = &MOTOR_1;
    if (m == NULL){
        start_cycles = 0;
    	return;
    }

    // ==============================================================================
    // VBUS Okuma ve Filtreleme
    // ==============================================================================
    if (hadc->Instance == ADC1) {
        uint32_t vbus_raw = HAL_ADCEx_InjectedGetValue(m->TIMER.ADC_TIMER, ADC_INJECTED_RANK_4);
        float_t vbus_instant = (float_t)vbus_raw * 0.01551282f;
		V_dc = (V_dc * 0.9f) + (vbus_instant * 0.1f);
    }
    if (V_dc < 5.0f) {
        m->STATUS.STOPPED_FAULT = true;
    }

    // ==============================================================================
    // Hız Döngüsü (Her 10 döngüde bir çalışır)
    // ==============================================================================
    if(m->STATUS.READY){
        if (m->STATUS.spdcnt == 10){
            static uint32_t last_speed_tick = 0;
            uint32_t now = HAL_GetTick();

#if (SIMULATE_MOTOR == true)
            static uint32_t last_sim_tick = 0;
            static float_t a = 0;
            static float_t b = 0;

            if ((now - last_sim_tick) >= (10000/sim_rpm))
            {
                get_sin_cos_fast(timer, &a, &b);
                timer++;
                if(timer == 360) timer = 0;
                last_sim_tick = now;

                tim_last = tim;
                tim = __HAL_TIM_GET_COUNTER(m->TIMER.HALL_TIMER);

                m->STATUS.STOPPED = false;
                m->STATUS.rotor_angle +=60;
                if (m->STATUS.rotor_angle >= 360) {
                    m->STATUS.rotor_angle = 0;
                }
                m->STATUS.last_hall_edge_tick = now;
            }
#endif

            if ((now - last_speed_tick) >= m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS)
            {
                last_speed_tick = now;

                // --- İvme Filtresi ---
                static float_t prev_loop_rpm = 0.0f;
                float_t fixed_dt = (float_t)m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS / 1000.0f;
                float_t clean_accel_raw = (m->STATUS.rotor_rpm - prev_loop_rpm) / fixed_dt;
                static float_t clean_accel = 0;
                clean_accel = (clean_accel * 0.8f) + (clean_accel_raw * 0.2f);
                m->STATUS.rotor_accel = (m->STATUS.rotor_accel * 0.95f) + (clean_accel * 0.05f);
                prev_loop_rpm = m->STATUS.rotor_rpm;

#if !DQ_TEST
                calculate_speed_pi(m);
#endif
                // --- Fren Durumu ---
                if ((m->STATUS.rotor_rpm > 2000.0f && m->REF.Iq < -0.5f) ||
                    (m->STATUS.rotor_rpm < -2000.0f && m->REF.Iq > 0.5f)) {
                    m->STATUS.BRAKE = true;
                } else {
                    m->STATUS.BRAKE = false;
                }

                float_t V_rms;
                if(m->PARAMS.CIRCULAR_LIM) {
                    V_rms = V_dc * ONE_BY_SQRT3;
                } else {
                    V_rms = V_dc;
                }
                float_t v_mag = __builtin_sqrtf((m->OUT.E_d * m->OUT.E_d) + (m->OUT.E_q * m->OUT.E_q));
                m->DIAG.mod_index = (v_mag / V_rms) * 100.0f;
                m->DIAG.power_w = 1.5f * ((m->OUT.E_d * m->STATUS.Id_curr) + (m->OUT.E_q * m->STATUS.Iq_curr));
            }

#if (SIMULATE_MOTOR)
            float_t dt_sim = m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS / 1000.0f;
            sim_rpm += (K_TORQUE * m->REF.Iq - FRICTION * sim_rpm) * dt_sim;
            m->STATUS.rotor_rpm = (int16_t)sim_rpm;
#endif
            m->STATUS.spdcnt = 0;
        } else {
            m->STATUS.spdcnt += 1;
        }
    }

    if (!m->STATUS.ALIGNED){
        uint32_t end_cycles = DWT->CYCCNT;
    	m->DIAG.foc_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
    	return;
    }

    // ==============================================================================
    // Akım Okuma
    // ==============================================================================
    Analog_Read_Currents(m, SIMULATE_MOTOR);

    // ==============================================================================
    // DÜŞÜK HIZ: Timer Extrapolation (Hall Sensörü Tahmini)
    // ==============================================================================
    if ((HAL_GetTick() - m->STATUS.last_hall_edge_tick) >= m->STATUS.STOPPED_TIMEOUT) {
		m->STATUS.STOPPED = true;
		m->STATUS.rotor_rpm = 0.0f;
		m->STATUS.hall_rpm = 0.0f;
		m->OBSERVER.rpm_filter_stage1 = 0.0f;

		// --- GÖZLEMCİ SIFIRLAMA (Halüsinasyon Önleyici) ---
		m->DIAG.observer_rpm = 0.0f;
		m->OBSERVER.E_alpha_est = 0.0f;
		m->OBSERVER.E_beta_est = 0.0f;
		m->DIAG.blend_factor = 0.0f;
		// --------------------------------------------------

		if(fabsf(m->REF.RPM) > 100.0f){
			m->STATUS.STOPPED_FAULT_COUNT++;
			}
        }

    if (m->STATUS.STOPPED) {
        m->STATUS.rotor_angle_interp = m->STATUS.rotor_angle;
    } else {
        m->STATUS.STOPPED_FAULT_COUNT = 0;
        m->REF.RPM_cur = clampf(m->REF.RPM_cur, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);

        uint32_t current_cnt = __HAL_TIM_GET_COUNTER(m->TIMER.HALL_TIMER);
        uint16_t current_tim = m->STATUS.tim;
        if (current_tim == 0) current_tim = 65535;

        float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
		if (interp_ratio > 1.0f) interp_ratio = 1.0f;

		float_t dTheta;


		if (fabsf(m->STATUS.rotor_rpm) > 500.0f) {
			float_t t_sec = (float_t)current_cnt / (float_t)TIM3_CNT_HZ;
			float_t alpha = m->STATUS.rotor_accel * 6.0f * (float_t)m->PARAMS.NUM_OF_POLE_PAIRS;
			dTheta = (60.0f * interp_ratio) + (0.5f * alpha * (t_sec * t_sec));
		} else {
			// Lineer (İvmesiz) Düz İnterpolasyon
			dTheta = (60.0f * interp_ratio);
		}

		dTheta = clampf(dTheta, 0.0f, 60.0f);

        float_t hall_interp_angle;
        if (m->OBSERVER.hall_direction >= 0) {
            hall_interp_angle = (float_t)m->STATUS.rotor_angle + dTheta;
            if (hall_interp_angle >= 360.0f) hall_interp_angle -= 360.0f;
        } else {
            hall_interp_angle = ((float_t)m->STATUS.rotor_angle + 60.0f) - dTheta;
            if (hall_interp_angle < 0.0f) hall_interp_angle += 360.0f;
            else if (hall_interp_angle >= 360.0f) hall_interp_angle -= 360.0f;
        }
        m->STATUS.rotor_angle_interp = (uint16_t)hall_interp_angle;
    }

    // ==============================================================================
    // FOC MATEMATİĞİ (Clarke/Park Dönüşümleri)
    // ==============================================================================

    // Faz akımlarından I_alpha ve I_beta değerlerini oluştur
    clarke(m);

    // BEMF Sensorless Observer'ı sürekli koşturarak açıyı tahmin et
    run_bemf_observer(m);

    // ==============================================================================
	// HİBRİT HARMANLAMA
	// ==============================================================================

	float_t abs_hall = fabsf(m->STATUS.hall_rpm);


	if (abs_hall < 1000.0f) {
		m->DIAG.observer_rpm = m->STATUS.hall_rpm; // Sahte hız zıplamalarını ez
		m->OBSERVER.E_alpha_est = 0.0f;
		m->OBSERVER.E_beta_est = 0.0f;
	}

		float_t target_blend = clampf((abs_hall - 1500.0f) * 0.001f, 0.0f, 1.0f);

		m->DIAG.blend_factor = (m->DIAG.blend_factor * 0.95f) + (target_blend * 0.05f);

		if (m->DIAG.blend_factor < 0.005f) {
		    m->DIAG.blend_factor = 0.0f;
		} else if (m->DIAG.blend_factor > 0.995f){
		    m->DIAG.blend_factor = 1.0f;
		}

		if (m->DIAG.blend_factor > 0.0f) {
			// Hız (RPM) Harmanlaması
			m->STATUS.rotor_rpm = (m->STATUS.hall_rpm * (1.0f - m->DIAG.blend_factor)) + (m->DIAG.observer_rpm * m->DIAG.blend_factor);
		} else {
			// Hız çok düşükse sadece Hall sensörünü kullan
			m->STATUS.rotor_rpm = m->STATUS.hall_rpm;
		}

	float_t abs_rpm = fabsf(m->STATUS.rotor_rpm);

	if (m->DIAG.blend_factor < 0.7f) {

		m->PARAMS.ERROR_PI.integral = 0.0f;
		m->PARAMS.ERROR_PI.output = 0.0f;
	}

    m->STATUS.kama_rpm = m->STATUS.rotor_rpm * 0.2222222f;

	// 1. Gerçek Hall Açısı
	float_t true_hall_angle = (float_t)m->STATUS.rotor_angle_interp + (float_t)m->PARAMS.HALL_OFSET;
	if (true_hall_angle >= 360.0f) true_hall_angle -= 360.0f;

	// 2. Gözlemci Açısı (Zaten run_bemf_observer içinde kusursuz hesaplandı)
	float_t true_obs_angle = m->OBSERVER.observer_angle_deg;

	// 3. Farkı Bul
	float_t diff = true_obs_angle - true_hall_angle;
	if (diff > 180.0f) diff -= 360.0f;
	else if (diff < -180.0f) diff += 360.0f;

    // Telemetri (İzleme) için açıyı güncelle
	m->DIAG.angle_error = diff;
	m->DIAG.filtered_angle_error = (m->DIAG.filtered_angle_error * 0.8f) + (diff * 0.2f);

	// 4. Harmanla
	float_t final_d_axis_angle = true_hall_angle + (diff * m->DIAG.blend_factor);
	if (final_d_axis_angle >= 360.0f) final_d_axis_angle -= 360.0f;
	else if (final_d_axis_angle < 0.0f) final_d_axis_angle += 360.0f;

	// 5. Sin/Cos Hesabı
	m->STATUS.advance_angle = 0;
	static float_t sin;
	static float_t cos;

	get_sin_cos_fast((uint16_t)final_d_axis_angle + (uint16_t)m->STATUS.advance_angle, &sin, &cos);
	m->STATUS.foc_cos = cos;
	m->STATUS.foc_sin = sin;

	park(m);


    // ==============================================================================
    // Alan Zayıflatma (Field Weakening) - GÜVENLİ MİMARİ
    // ==============================================================================
    if(m->PARAMS.FW){
        // Sadece yüksek devirlerde RPM filtresini çalıştır (Boş yere işlemciyi yorma)
        m->OBSERVER.filtered_fw_rpm = (m->OBSERVER.filtered_fw_rpm * 0.99f) + (abs_rpm * 0.01f);

        // SADECE Hız > MAX_WO_FW olduğunda devreye gir! (Negatif yönde Id patlamasını önler)
        if (m->OBSERVER.filtered_fw_rpm > (float_t)m->PARAMS.MAX_WO_FW) {
            float_t fw_delta_rpm = m->OBSERVER.filtered_fw_rpm - (float_t)m->PARAMS.MAX_WO_FW;
            float_t target_id = -m->PARAMS.FW_CONSTANT * fw_delta_rpm;
            m->REF.Id = clampf(target_id, -20.0f, 0.0f);
        } else {
            m->REF.Id = 0.0f; // Hız düşükse Id kesinlikle SIFIR olmalıdır
        }
    } else {
        m->REF.Id = 0.0f;
    }

    // ==============================================================================
	// Akım PI Döngüleri ve Feed-Forward
	// ==============================================================================
    calculate_dq_pi(m, V_dc);

    // PI çıkışlarını Ters Dönüşümlerle 3 Faza dağıt
    inv_park(m);
	inv_clarke(m);

	// ==============================================================================
	    // Çıkış (SVPWM / PWM) Üretimi - BÖLMELERDEN ARINDIRILMIŞ VERSİYON
	    // ==============================================================================
	#if SVPWM_OUT
	if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
		m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
		m->PARAMS.SPEED_PI.Speed_integral = 0;
		m->PARAMS.DQ_PI.Id_integral = 0;
		m->PARAMS.DQ_PI.Iq_integral = 0;
	}

	float_t va_flat = m->OUT.Va;
	float_t vb_flat = m->OUT.Vb;
	float_t vc_flat = m->OUT.Vc;

	float_t v_max = va_flat;
	float_t v_min = va_flat;

	if (vb_flat > v_max) v_max = vb_flat;
	if (vc_flat > v_max) v_max = vc_flat;
	if (vb_flat < v_min) v_min = vb_flat;
	if (vc_flat < v_min) v_min = vc_flat;

	float_t V_com = -(v_max + v_min) * 0.5f;
	float_t half_vdc = V_dc * 0.5f;
	float_t svpwm_mul = 3600.0f / V_dc; // Tek bir bölme işlemi!

	// (Va + V_com + Vdc/2) * (1800 / Vdc) matematiği doğrudan uygulandı
	m->SVPWM.A = (uint16_t)clampf((m->OUT.Va + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);
	m->SVPWM.B = (uint16_t)clampf((m->OUT.Vb + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);
	m->SVPWM.C = (uint16_t)clampf((m->OUT.Vc + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);

	if (m->STATUS.BRAKE) {
		m->SVPWM.A = 0; m->SVPWM.B = 0; m->SVPWM.C = 0;
		m->PARAMS.DQ_PI.Iq_integral = 0;
		m->PARAMS.DQ_PI.Id_integral = 0;
		m->PARAMS.SPEED_PI.Speed_integral = 0;
	}

	pwm_write(m, m->SVPWM.A, m->SVPWM.B, m->SVPWM.C);

#else
    if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
        m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
        m->PARAMS.SPEED_PI.Speed_integral = 0;
        m->PARAMS.DQ_PI.Id_integral = 0;
        m->PARAMS.DQ_PI.Iq_integral = 0;
    }
    m->PWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 3500);
    m->PWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 3500);
    m->PWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 3500);

    pwm_write(m, m->PWM.A, m->PWM.B, m->PWM.C);
#endif

    // ==============================================================================
    // Opsiyonel DAC Çıkışı (Hata Ayıklama)
    // ==============================================================================
#if DAC_OUT == true
    if (hadc->Instance == ADC1) {
    	float_t iq_curr = m->STATUS.Iq_curr;
		uint32_t dac_iq_val = (uint32_t)clampf(map(iq_curr, -30.0f, 30.0f, 0.0f, 4095.0f), 0.0f, 4095.0f);

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_iq_val);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_iq_val);
    }
#endif
    uint32_t end_cycles = DWT->CYCCNT;
	m->DIAG.foc_time_us = (uint16_t)((end_cycles - start_cycles) / (SystemCoreClock / 1000000));
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}
