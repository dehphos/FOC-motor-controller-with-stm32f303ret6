#include "foc_interrupt.h"
#include "main.h"
#include "control.h"
#include "math.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

extern motor MOTOR_1;
extern volatile float_t V_dc;
extern float_t VBUS_DIVIDER_RATIO;

extern void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val);


//__attribute__((section(".ccmram")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
    motor *m = NULL;
    m = &MOTOR_1;
    if (m == NULL) return;
    if (hadc->Instance == ADC1) {
        uint32_t vbus_raw = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
        float_t vbus_instant = ((float_t)vbus_raw / 4095.0f) * 3.3f * VBUS_DIVIDER_RATIO;
        V_dc = (V_dc * 0.9f) + (vbus_instant * 0.1f);
    }
    if (V_dc < 5.0f) {
        m->STATUS.STOPPED_FAULT = true;
    }


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
                tim = __HAL_TIM_GET_COUNTER(&htim3);

                m->STOPPED = false;
                m->STATUS.rotor_angle +=60;
                if (m->STATUS.rotor_angle >= 360) {
                    m->STATUS.rotor_angle = 0;
                }
                m->last_hall_edge_tick = now;
            }
#endif

            if ((now - last_speed_tick) >= m->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS)
                        {
                            last_speed_tick = now;
                            // ---------------- SABİT ZAMANLI VE GÜRÜLTÜSÜZ İVME HESABI ----------------
							static float_t prev_loop_rpm = 0.0f;
							float_t fixed_dt = (float_t)m->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS / 1000.0f;
							float_t clean_accel_raw = (m->STATUS.rotor_rpm - prev_loop_rpm) / fixed_dt;
							static float_t clean_accel = 0;
							// İvmeyi yumuşak bir Low-Pass filtreden geçir
							clean_accel = (clean_accel * 0.8f) + (clean_accel_raw * 0.2f);
							m->STATUS.rotor_accel = (m->STATUS.rotor_accel * 0.95f) + (clean_accel * 0.05f);
							prev_loop_rpm = m->STATUS.rotor_rpm;
							// --------------------------------------------------------------------------

#if !DQ_TEST
                            calculate_speed_pi(m);
#endif
                            // ---------------- BRAKE DURUMU KONTROLÜ ----------------
							// Motor 10 RPM'den hızlı dönerken, akım talebi ters yönde (0.2A'den fazla) ise
							if ((m->STATUS.rotor_rpm > 2000.0f && m->REF.Iq < -0.2f) ||
								(m->STATUS.rotor_rpm < -2000.0f && m->REF.Iq > 0.2f)) {
								m->STATUS.BRAKE = true;
							} else {
								m->STATUS.BRAKE = false;
							}
							// --------------------------------------------------------------

                        }


#if (SIMULATE_MOTOR)
            float_t dt = m->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS / 1000.0f;
            sim_rpm += (K_TORQUE * m->REF.Iq - FRICTION * sim_rpm) * dt;
            m->STATUS.rotor_rpm = (int16_t)sim_rpm;
#endif
            m->STATUS.spdcnt = 0;
        } else {
            m->STATUS.spdcnt += 1;
        }
    }

    if (!m->STATUS.ALIGNED) return;

    Analog_Read_Currents(m, SIMULATE_MOTOR, I_max);

    // ------------------------ FOC MATEMATİĞİ --------------------------
    uint32_t current_cnt;
    uint16_t current_tim;

    current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
    current_tim = m->STATUS.tim;

    if ((HAL_GetTick() - m->STATUS.last_hall_edge_tick) >= m->STATUS.STOPPED_TIMEOUT) {
        m->STATUS.STOPPED = true;
        m->STATUS.rotor_rpm = 0;
        if(fabsf(m->REF.RPM) > 100){
            m->STATUS.STOPPED_FAULT_COUNT++;
        }
    }

    if (m->STATUS.STOPPED) {
        m->STATUS.rotor_angle_interp = m->STATUS.rotor_angle;
    } else {

        m->STATUS.STOPPED_FAULT_COUNT = 0;
        m->REF.RPM_cur = clampf(m->REF.RPM_cur, -m->PARAMS.MAX_RPM, m->PARAMS.MAX_RPM);
    	// ------------------ EXTRAPOLASYON --------------------------------



        if (current_tim == 0) current_tim = 65535;
        float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
        if (interp_ratio > 1.0f) interp_ratio = 1.0f;

        // ---------------- 1. ve 2. DERECE KİNEMATİK AÇI HESABI ----------------
        float_t dTheta;

		// Sadece 1000 RPM üstünde ivme katsayısını ekle
		if (fabsf(m->STATUS.rotor_rpm) > 100.0f) {
			float_t t_sec = (float_t)current_cnt / (float_t)TIM3_CNT_HZ;
			float_t alpha = m->STATUS.rotor_accel * 6.0f * (float_t)m->PARAMS.NUM_OF_POLE_PAIRS;
			dTheta = (60.0f * interp_ratio) + (0.5f * alpha * (t_sec * t_sec));
			dTheta = clampf(dTheta, 0.0f, 60.0f);
		}else{
			dTheta = (60.0f * interp_ratio);
		}

		// ---------------- AÇIYI UYGULAMA ----------------
		if (m->STATUS.rotor_rpm >= 0.0f) {
			m->STATUS.rotor_angle_interp = m->STATUS.rotor_angle + (uint16_t)dTheta;
			if (m->STATUS.rotor_angle_interp >= 360) m->STATUS.rotor_angle_interp -= 360;
		} else {
			int16_t temp_angle = (m->STATUS.rotor_angle + 60) - (int16_t)dTheta;
			if (temp_angle < 0) temp_angle += 360;
			else if (temp_angle >= 360) temp_angle -= 360;
			m->STATUS.rotor_angle_interp = (uint16_t)temp_angle;
		}

		// ---------------- GERİ DÖNMEYİ ENGELLEME FİLTRESİ ----------------
		int16_t diff = m->STATUS.rotor_angle_interp - m->OBSERVER.prev_angle_interp;

		if (fabsf(diff) < 180 && (diff * m->OBSERVER.hall_direction < 0)) {
			m->STATUS.rotor_angle_interp = m->OBSERVER.prev_angle_interp;
		}


        m->OBSERVER.prev_angle_interp = m->STATUS.rotor_angle_interp;
    }

    float_t sin_angle;
    float_t cos_angle;
    float_t advance_angle = (m->STATUS.rotor_rpm / 7500.0f) * 15.0f;
    get_sin_cos_fast(m->STATUS.rotor_angle_interp + m->PARAMS.HALL_OFSET + advance_angle, &sin_angle, &cos_angle);

    float_t Ia_foc = m->STATUS.Ia_curr_map;
    float_t Ib_foc = m->STATUS.Ib_curr_map;
    float_t Id_raw, Iq_raw;
    clarke_park(Ia_foc, Ib_foc, sin_angle, cos_angle, &Id_raw, &Iq_raw);

    m->STATUS.Id_curr = (m->STATUS.Id_curr * 0.8f) + (Id_raw * 0.2f);
    m->STATUS.Iq_curr = (m->STATUS.Iq_curr * 0.8f) + (Iq_raw * 0.2f);

    // --- FIELD WEAKENING ---
    if(m->PARAMS.FW){
        float_t abs_rpm = fabsf(m->STATUS.rotor_rpm);
        m->OBSERVER.filtered_fw_rpm = (m->OBSERVER.filtered_fw_rpm * 0.99f) + (abs_rpm * 0.01f);
        float_t target_id = -0.0008f * (m->OBSERVER.filtered_fw_rpm - 8500.0f);
        m->REF.Id = clampf(target_id, -20.0f, 0.0f);
    }

    // --- PI Döngüsü ---
    m->DQ_PI_PARAMS.Iq_E = (m->REF.Iq - m->STATUS.Iq_curr);
    m->DQ_PI_PARAMS.Iq_integral_lim = V_dc / m->DQ_PI_PARAMS.Iq_ki;
    m->DQ_PI_PARAMS.Iq_integral += m->DQ_PI_PARAMS.Iq_E;
    m->DQ_PI_PARAMS.Iq_integral = clampf(m->DQ_PI_PARAMS.Iq_integral, - m->DQ_PI_PARAMS.Iq_integral_lim, m->DQ_PI_PARAMS.Iq_integral_lim);
    m->OUT.E_q = m->DQ_PI_PARAMS.Iq_kp * m->DQ_PI_PARAMS.Iq_E + m->DQ_PI_PARAMS.Iq_ki * m->DQ_PI_PARAMS.Iq_integral;

    m->DQ_PI_PARAMS.Id_E = (m->REF.Id - m->STATUS.Id_curr);
    m->DQ_PI_PARAMS.Id_integral_lim = V_dc / m->DQ_PI_PARAMS.Id_ki;
    m->DQ_PI_PARAMS.Id_integral += m->DQ_PI_PARAMS.Id_E;
    m->DQ_PI_PARAMS.Id_integral = clampf(m->DQ_PI_PARAMS.Id_integral, - m->DQ_PI_PARAMS.Id_integral_lim, m->DQ_PI_PARAMS.Id_integral_lim);
    m->OUT.E_d = m->DQ_PI_PARAMS.Id_kp * m->DQ_PI_PARAMS.Id_E + m->DQ_PI_PARAMS.Id_ki * m->DQ_PI_PARAMS.Id_integral;

    // --- Feed Forward ---
    if(m->PARAMS.FF){
    m->PARAMS.omega_e = m->STATUS.rotor_rpm * (PI / 30.0f) * m->PARAMS.NUM_OF_POLE_PAIRS;
    float_t Vd_ff = -m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Iq_curr;
    float_t Vq_ff = (m->PARAMS.omega_e * m->PARAMS.Ls * m->STATUS.Id_curr) + (m->PARAMS.omega_e * m->PARAMS.psi_m);

    m->OUT.E_d += Vd_ff;
    m->OUT.E_q += Vq_ff;
    }
    // --- BARA LİMİTİ ---
    float_t V_rms;
    if(m->PARAMS.CIRCULAR_LIM){
    V_rms = V_dc * ONE_BY_SQRT3;
    }else{V_rms = V_dc;}
    m->OUT.E_d = clampf(m->OUT.E_d, -V_rms, V_rms);
    float_t Eq_max = sqrtf((V_rms * V_rms) - (m->OUT.E_d * m->OUT.E_d));
    m->OUT.E_q = clampf(m->OUT.E_q, -Eq_max, Eq_max);

    inv_clarke_park(m->OUT.E_d, m->OUT.E_q, sin_angle, cos_angle, &m->OUT.Va, &m->OUT.Vb, &m->OUT.Vc);

#if SVPWM_OUT
    // --- SVPWM ---
    if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
        m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
        m->SPEED_PI_PARAMS.Speed_integral = 0;
        m->DQ_PI_PARAMS.Id_integral = 0;
        m->DQ_PI_PARAMS.Iq_integral = 0;
    }
    float_t V_max = m->OUT.Va;
    float_t V_min = m->OUT.Va;

    if (m->OUT.Vb > V_max) {V_max = m->OUT.Vb;}
    if (m->OUT.Vc > V_max) {V_max = m->OUT.Vc;}
    if (m->OUT.Vb < V_min) {V_min = m->OUT.Vb;}
    if (m->OUT.Vc < V_min) {V_min = m->OUT.Vc;}

    float_t V_com = -(V_max + V_min) / 2.0f;

    m->SVPWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 30, 1795);
    m->SVPWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 30, 1795);
    m->SVPWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 30, 1795);

    if (m->STATUS.BRAKE) {
            m->SVPWM.A = 0;
            m->SVPWM.B = 0;
            m->SVPWM.C = 0;

            m->DQ_PI_PARAMS.Iq_integral = 0;
            m->DQ_PI_PARAMS.Id_integral = 0;
            m->SPEED_PI_PARAMS.Speed_integral = 0;
       }

    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, m->SVPWM.A );
    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, m->SVPWM.B );
    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, m->SVPWM.C );
#else
    if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
        m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
        m->SPEED_PI_PARAMS.Speed_integral = 0;
        m->DQ_PI_PARAMS.Id_integral = 0;
        m->DQ_PI_PARAMS.Iq_integral = 0;
    }
    m->PWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
    m->PWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);
    m->PWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 30, 1770);

    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.A, m->PWM.A );
    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.B, m->PWM.B );
    __HAL_TIM_SET_COMPARE(&htim1, m->OUT.C, m->PWM.C );
#endif

#if DAC_OUT == true
    if (hadc->Instance == ADC1) {
        uint32_t dac_ch1_raw_angle = (uint32_t)map((float_t)m->STATUS.rotor_angle, 0.0f, 360.0f, 0.0f, 4095.0f);
        uint32_t dac_ch2_interp_angle = (uint32_t)map((float_t)m->STATUS.rotor_angle_interp, 0.0f, 360.0f, 0.0f, 4095.0f);

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_ch1_raw_angle);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_ch2_interp_angle);
    }
#endif
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}
