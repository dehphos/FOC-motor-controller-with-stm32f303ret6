/**
 * @file    foc_interrupt.c
 * @brief   Alan Yönlendirmeli Kontrol (FOC) donanım ISR (Interrupt Service Routine) akışı.
 */
#include "foc_interrupt.h"
#include "main.h"
#include "control.h"
#include "math.h"
#include "analog_veri_okuma.h"
#include "clampf.h"
#include "map.h"

extern motor MOTOR_1;
extern volatile float_t V_dc;
extern float_t VBUS_DIVIDER_RATIO;
extern DAC_HandleTypeDef hdac1;

extern void get_sin_cos_fast(uint16_t angle_deg, float_t *sin_val, float_t *cos_val);

//__attribute__((section(".ccmram")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
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
        float_t vbus_instant = ((float_t)vbus_raw / 4095.0f) * 3.3f * VBUS_DIVIDER_RATIO;
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
                if ((m->STATUS.rotor_rpm > 2000.0f && m->REF.Iq < -0.2f) ||
                    (m->STATUS.rotor_rpm < -2000.0f && m->REF.Iq > 0.2f)) {
                    m->STATUS.BRAKE = true;
                } else {
                    m->STATUS.BRAKE = false;
                }
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
    Analog_Read_Currents(m, SIMULATE_MOTOR, I_max);

    // ==============================================================================
    // DÜŞÜK HIZ: Timer Extrapolation (Hall Sensörü Tahmini)
    // ==============================================================================
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

        uint32_t current_cnt = __HAL_TIM_GET_COUNTER(m->TIMER.HALL_TIMER);
        uint16_t current_tim = m->STATUS.tim;
        if (current_tim == 0) current_tim = 65535;

        float_t interp_ratio = (float_t)current_cnt / (float_t)current_tim;
        if (interp_ratio > 1.0f) interp_ratio = 1.0f;

        float_t dTheta;
        if (fabsf(m->STATUS.rotor_rpm) > 10.0f) {
            float_t t_sec = (float_t)current_cnt / (float_t)TIM3_CNT_HZ;
            float_t alpha = m->STATUS.rotor_accel * 6.0f * (float_t)m->PARAMS.NUM_OF_POLE_PAIRS;
            dTheta = (60.0f * interp_ratio) + (0.5f * alpha * (t_sec * t_sec));
            dTheta = clampf(dTheta, 0.0f, 60.0f);
        } else {
            dTheta = (60.0f * interp_ratio);
        }

        float_t hall_interp_angle;
        if (m->STATUS.rotor_rpm >= 0.0f) {
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
    // HİBRİT AÇI HARMANLAMA (Hall -> Observer Transition)
    // ==============================================================================
    float_t abs_rpm = fabsf(m->STATUS.rotor_rpm);

    // 1500 RPM ile 2500 RPM arasında Hall açısından Sensörsüz açıya pürüzsüz geçiş (Blending)
    m->DIAG.blend_factor = clampf(map(abs_rpm, 1500.0f, 2500.0f, 0.0f, 1.0f), 0.0f, 1.0f);

    float_t angle_hall = (float_t)m->STATUS.rotor_angle_interp;
    float_t angle_obs  = m->OBSERVER.observer_angle_deg;

    // Dairesel fark alma (-180 / +180 sınırlarına çek)
    float_t diff = angle_obs - angle_hall;
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;

    // Ağırlıklı geçiş
    float_t final_rotor_angle = angle_hall + (diff * m->DIAG.blend_factor);

    if (final_rotor_angle >= 360.0f) final_rotor_angle -= 360.0f;
    else if (final_rotor_angle < 0.0f) final_rotor_angle += 360.0f;

    // Seçilen mükemmel açı ile sinüs ve kosinüsü hesapla
    m->STATUS.advance_angle = 0; // İsteğe bağlı phase advance
    static float_t sin;
    static float_t cos;

    get_sin_cos_fast((uint16_t)final_rotor_angle + m->PARAMS.HALL_OFSET + (uint16_t)m->STATUS.advance_angle, &sin, &cos);
    m->STATUS.foc_cos = cos;
    m->STATUS.foc_sin = sin;

    // Elde edilen sin/cos ile Akımları D-Q düzlemine taşı
    park(m);

    // ==============================================================================
    // Alan Zayıflatma (Field Weakening)
    // ==============================================================================
    if(m->PARAMS.FW){
        m->OBSERVER.filtered_fw_rpm = (m->OBSERVER.filtered_fw_rpm * 0.99f) + (abs_rpm * 0.01f);
        float_t target_id = -0.006f * (m->OBSERVER.filtered_fw_rpm - 8500.0f);
        m->REF.Id = clampf(target_id, -20.0f, 0.0f);
    }

    // ==============================================================================
	// Akım PI Döngüleri ve Feed-Forward
	// ==============================================================================
    calculate_dq_pi(m, V_dc);

    // PI çıkışlarını Ters Dönüşümlerle 3 Faza dağıt
    inv_park(m);
	inv_clarke(m);

    // ==============================================================================
    // Çıkış (SVPWM / PWM) Üretimi
    // ==============================================================================
#if SVPWM_OUT
    if(m->REF.RPM == 0 && m->REF.RPM_cur == 0){
        m->OUT.Va = 0; m->OUT.Vb = 0; m->OUT.Vc = 0;
        m->PARAMS.SPEED_PI.Speed_integral = 0;
        m->PARAMS.DQ_PI.Id_integral = 0;
        m->PARAMS.DQ_PI.Iq_integral = 0;
    }
    float_t V_max = m->OUT.Va;
    float_t V_min = m->OUT.Va;

    if (m->OUT.Vb > V_max) {V_max = m->OUT.Vb;}
    if (m->OUT.Vc > V_max) {V_max = m->OUT.Vc;}
    if (m->OUT.Vb < V_min) {V_min = m->OUT.Vb;}
    if (m->OUT.Vc < V_min) {V_min = m->OUT.Vc;}

    float_t V_com = -(V_max + V_min) / 2.0f;

    m->SVPWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 0, 1700);
    m->SVPWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 0, 1700);
    m->SVPWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc + V_com, - V_dc/2, V_dc/2), (float_t)-V_dc/2, (float_t)V_dc/2, (float_t)0, (float_t)1800), 0, 1700);

    if (m->STATUS.BRAKE) {
        m->SVPWM.A = 0;
        m->SVPWM.B = 0;
        m->SVPWM.C = 0;

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
    m->PWM.A = (uint16_t)clampf(map((float_t)clampf(m->OUT.Va, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 1770);
    m->PWM.B = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vb, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 1770);
    m->PWM.C = (uint16_t)clampf(map((float_t)clampf(m->OUT.Vc, - V_dc, V_dc), (float_t)-V_dc, (float_t)V_dc, (float_t)0, (float_t)1800), 0, 1770);

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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}
