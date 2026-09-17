/**
 * @file    foc_interrupt.c
 * @brief   Alan Yönlendirmeli Kontrol (FOC) donanım ISR (Interrupt Service Routine) akışı.
 */
#include "foc_interrupt.h"


extern motor MOTOR_1;
extern volatile float_t V_dc;

/**
 * @brief   Ana Alan Yönlendirmeli Kontrol (FOC) döngüsünü yürüten, enjekte
 *          edilmiş (injected) ADC dönüşümü tamamlandığında (20 kHz'de, PWM
 *          zamanlayıcısının TRGO sinyaliyle senkronize) tetiklenen HAL geri
 *          çağırma (callback) kesmesi.
 *
 * @details Bu fonksiyon, sistemin en yüksek öncelikli ve en zaman-kritik
 *          rutinidir; tüm akım döngüsü (current loop) matematiği burada
 *          gerçekleşir. Akış sırasıyla şu adımlardan oluşur:
 *
 *          1. **VBUS Okuma:** Sadece `ADC1` kaynaklı çağrılarda DC bara
 *             gerilimi (`V_dc`) okunur, ölçeklenir ve %90/%10 ağırlıklı
 *             alçak geçiren filtre (LPF) ile yumuşatılır. `V_dc` 5V altına
 *             düşerse `STOPPED_FAULT` bayrağı derhal set edilir (Düşük
 *             voltaj koruması).
 *          2. **Hız Döngüsü (Alt Örnekleme):** `m->STATUS.READY` iken her
 *             10 FOC çevriminde bir (yaklaşık 2 kHz) ve ayrıca
 *             `SPEED_LOOP_PERIOD_MS` süresi dolduğunda `calculate_speed_pi()`
 *             çağrılır; ivme filtresi, fren (`BRAKE`) durumu, modülasyon
 *             indeksi (`mod_index`) ve anlık elektriksel güç (`power_w`)
 *             burada güncellenir. `SIMULATE_MOTOR` etkinse gerçek donanım
 *             yerine basit bir birinci dereceden motor modeli koşturulur.
 *          3. **Hizalama Kontrolü:** Motor henüz `ALIGNED` değilse, geri
 *             kalan tüm FOC matematiği atlanır ve yalnızca ISR süresi
 *             (`foc_time_us`) ölçülüp fonksiyondan erken çıkılır.
 *          4. **Akım Okuma:** `Analog_Read_Currents()` ile üç faz akımı
 *             (Ia/Ib/Ic) okunup Amper'e ölçeklenir.
 *          5. **Düşük Hız Açı Ekstrapolasyonu:** Hall kenar zaman aşımı
 *             (`STOPPED_TIMEOUT`) dolmuşsa motor `STOPPED` kabul edilip
 *             gözlemci durumu sıfırlanır (halüsinasyon önleyici).
 *             Aksi halde, TIM3 sayacından elde edilen `interp_ratio` ile
 *             ve (yüksek hızda) ivme terimiyle desteklenmiş açısal
 *             interpolasyon (`rotor_angle_interp`) hesaplanır.
 *          6. **Clarke Dönüşümü ve BEMF Gözlemcisi:** `clarke()` ile
 *             `I_alpha`/`I_beta` üretilir, ardından `run_bemf_observer()`
 *             sensörsüz açı/hız tahminini günceller.
 *          7. **Hibrit Harmanlama:** Hall tabanlı hız (`hall_rpm`) ile
 *             gözlemci hızı (`observer_rpm`), mutlak hıza bağlı bir
 *             `blend_factor` (0=Hall, 1=Gözlemci) ile kademeli olarak
 *             karıştırılıp nihai `rotor_rpm` elde edilir. Aynı şekilde Hall
 *             açısı (`true_hall_angle`) ile gözlemci açısı (`true_obs_angle`)
 *             arasındaki fark (`angle_error`) hesaplanır ve `blend_factor`
 *             oranında harmanlanarak nihai d-ekseni açısı (`final_d_axis_angle`)
 *             bulunur; bu açının sin/cos değerleri LUT ile hesaplanır.
 *          8. **Park Dönüşümü:** `park()` ile `Id_curr`/`Iq_curr` üretilir.
 *          9. **Alan Zayıflatma (Field Weakening):** `PARAMS.FW` aktifse ve
 *             filtrelenmiş hız (`filtered_fw_rpm`) `MAX_WO_FW` sınırını
 *             aşıyorsa, negatif yönde sınırlı bir `REF.Id` hedefi üretilir;
 *             aksi halde `REF.Id` sıfırlanır.
 *          10. **Akım PI ve Ters Dönüşümler:** `calculate_dq_pi()` ile
 *              `E_d`/`E_q` gerilimleri hesaplanır, `inv_park()` ve
 *              `inv_clarke()` ile üç faz gerilim komutlarına (`Va/Vb/Vc`)
 *              dönüştürülür.
 *          11. **PWM/SVPWM Üretimi:** Derleme zamanı seçimine göre
 *              (`SVPWM_OUT`/klasik `PWM_OUT`) faz gerilimleri, bölme
 *              işlemlerinden arındırılmış (tek çarpımla) bir formülle
 *              zamanlayıcı compare değerlerine (`SVPWM`/`PWM`) dönüştürülüp
 *              `pwm_write()` ile donanıma yazılır. `RPM` ve `RPM_cur` sıfırsa
 *              çıkışlar ve tüm PI integralleri sıfırlanır; `BRAKE`
 *              aktifse (yalnızca SVPWM modunda) çıkışlar da sıfırlanır.
 *          12. **Opsiyonel DAC Çıkışı:** `DAC_OUT` etkinse anlık `Iq_curr`
 *              değeri hata ayıklama amacıyla iki DAC kanalına yazılır.
 *          13. **Zamanlama Telemetrisi:** Fonksiyonun toplam işlemci
 *              süresi `DWT->CYCCNT` sayaç farkından hesaplanıp
 *              `DIAG.foc_time_us` alanına yazılır (20 kHz döngüde her
 *              zaman 50 µs'nin altında kalmalıdır).
 *
 * @param   hadc  Kesmeyi tetikleyen ADC donanım işaretçisi (yalnızca
 *                `ADC1` VBUS okuması için ayrıca kontrol edilir; akım/FOC
 *                akışı `hadc` değerinden bağımsız olarak her çağrıda koşar).
 *
 * @warning Bu fonksiyon 20 kHz'de (50 µs periyotla) çalışan bir donanım
 *          kesmesidir; içinde bloklayıcı (`HAL_Delay` vb.) çağrı
 *          bulunmamalıdır. `m == NULL` durumunda (teorik olarak
 *          gerçekleşmemesi gereken) güvenlik amaçlı erken çıkış yapılır.
 */
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
//                float_t fixed_dt = (float_t)m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS / 1000.0f;
                float_t inv_dt = 1000.0f / (float_t)m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS ;
                float_t clean_accel_raw = (m->STATUS.rotor_rpm - prev_loop_rpm) * inv_dt;
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
    Analog_Read_Currents(m);

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

//      float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
        float_t interp_ratio = (float_t)current_cnt * m->STATUS.inv_tim;
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
//	m->DIAG.filtered_angle_error = (m->DIAG.filtered_angle_error * 0.8f) + (diff * 0.2f);

	// 4. Harmanla
	float_t final_d_axis_angle = true_hall_angle + (diff * m->DIAG.blend_factor);
	if (final_d_axis_angle >= 360.0f) final_d_axis_angle -= 360.0f;
	else if (final_d_axis_angle < 0.0f) final_d_axis_angle += 360.0f;

	// 5. Sin/Cos Hesabı
	static float_t sin;
	static float_t cos;

	get_sin_cos_fast((uint16_t)final_d_axis_angle, &sin, &cos);
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
	float_t svpwm_mul = 3600.0f / V_dc;

	m->SVPWM.A = (uint16_t)clampf((m->OUT.Va + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);
	m->SVPWM.B = (uint16_t)clampf((m->OUT.Vb + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);
	m->SVPWM.C = (uint16_t)clampf((m->OUT.Vc + V_com + half_vdc) * svpwm_mul, 0.0f, 3500.0f);

	m->STATUS.PWM_A_DUTY = m->SVPWM.A / 36;
	m->STATUS.PWM_B_DUTY = m->SVPWM.B / 36;
	m->STATUS.PWM_C_DUTY = m->SVPWM.C / 36;

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
