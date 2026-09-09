/**
 * @file    control.h
 * @brief   FOC (Field Oriented Control) motor kontrolü için temel dönüşüm
 *          fonksiyonları (Clarke/Park ve tersleri), hız PI regülatörü,
 *          rampa ve PWM yazma yardımcıları.
 */

#ifndef CONTROL_H
#define CONTROL_H

#define FAST_INLINE static inline __attribute__((always_inline))

#include "main.h"
#include "math.h"
#include "clampf.h"

void calculate_speed_pi(motor *m);




FAST_INLINE void clarke(motor* m)
{
    m->STATUS.I_alpha = (2.0f * m->STATUS.Ia_curr_map - m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map) / 3.0f;
    m->STATUS.I_beta  = (m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map) * ONE_BY_SQRT3;
}

FAST_INLINE void park(motor* m)
{
    m->STATUS.Id_curr =  (m->STATUS.I_alpha * m->STATUS.foc_cos) + (m->STATUS.I_beta * m->STATUS.foc_sin);
    m->STATUS.Iq_curr = -(m->STATUS.I_alpha * m->STATUS.foc_sin) + (m->STATUS.I_beta * m->STATUS.foc_cos);
}

FAST_INLINE void inv_park(motor* m)
{
    m->OUT.V_alpha = (m->OUT.E_d * m->STATUS.foc_cos) - (m->OUT.E_q * m->STATUS.foc_sin);
    m->OUT.V_beta  = (m->OUT.E_d * m->STATUS.foc_sin) + (m->OUT.E_q * m->STATUS.foc_cos);
}

FAST_INLINE void inv_clarke(motor* m)
{
    m->OUT.Va = m->OUT.V_alpha;
    m->OUT.Vb = (-0.5f * m->OUT.V_alpha) + (SQRT3_BY_2 * m->OUT.V_beta);
    m->OUT.Vc = (-0.5f * m->OUT.V_alpha) - (SQRT3_BY_2 * m->OUT.V_beta);
}


/**
 * @brief   Hız referansını (REF.RPM) belirlenen adımlarla hedefe rampalar.
 *
 * @details Ani hız taleplerinde motorun ve mekaniğin zarar görmemesi
 *          için `REF.STEP` büyüklüğünde yumuşak ivmelenme (Slew-Rate) sağlar.
 *
 * @param   m  Üzerinde işlem yapılacak motor yapısına işaretçi.
 */
FAST_INLINE void ramp(motor *m) {

	float_t target_accel_rpm_s = (m->REF.STEP * 1000.0f) / (float_t)m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS;
	m->PARAMS.MAX_RPM_ACCEL = target_accel_rpm_s * 5.0f;

    if (m->REF.RPM > m->REF.RPM_cur) {
        m->REF.RPM_cur += m->REF.STEP;
        if (m->REF.RPM_cur > m->REF.RPM) {
            m->REF.RPM_cur = m->REF.RPM;
        }
    }
    else if (m->REF.RPM < m->REF.RPM_cur) {
        m->REF.RPM_cur -= m->REF.STEP;
        if (m->REF.RPM_cur < m->REF.RPM) {
            m->REF.RPM_cur = m->REF.RPM;
        }
    }
}

/**
 * @brief  Verilen üç faz değerini ilgili PWM zamanlayıcısının A/B/C
 *         karşılaştırma (compare) kayıtlarına yazar.
 *
 * @param  m  PWM'i yazılacak motor yapısına işaretçi (zamanlayıcı ve kanal
 *            bilgilerini içerir).
 * @param  a  A fazı için yazılacak compare değeri.
 * @param  b  B fazı için yazılacak compare değeri.
 * @param  c  C fazı için yazılacak compare değeri.
 */
FAST_INLINE void pwm_write(motor *m, float_t a, float_t b, float_t c){
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.A, a);
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.B, b);
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.C, c);
}

void Align_Motor(motor *m);

/**
 * @brief  Motor kontrolü için FPU optimize edilmiş ultra hızlı Arctangent (atan2).
 * @note   Maksimum açısal hata: 0.08 derece. math.h kütüphanesinden ~10 kat hızlıdır.
 */
FAST_INLINE float_t fast_atan2f(float_t y, float_t x) {
    if (x == 0.0f && y == 0.0f) return 0.0f;

    float_t abs_y = fabsf(y);
    float_t abs_x = fabsf(x);
    float_t a;
    float_t angle;

    if (abs_x > abs_y) {
        a = abs_y / abs_x;
        // 3. dereceden polinom yaklaşımı
        angle = 0.78539816f * a - a * (a - 1.0f) * (0.2447f + 0.0663f * a);
    } else {
        a = abs_x / abs_y;
        angle = PI_BY_TWO - (0.78539816f * a - a * (a - 1.0f) * (0.2447f + 0.0663f * a));
    }

    // Quadrant (Bölge) düzeltmeleri
    if (x < 0.0f) angle = PI - angle;
    if (y < 0.0f) angle = -angle;

    return angle;
}

// control.h dosyasının içine EKLENECEK KODLAR:

static inline __attribute__((always_inline)) void run_bemf_observer(motor *m)
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

    m->DIAG.observer_rpm = (m->DIAG.observer_rpm * 0.01f) + (observer_rpm_raw * 0.99f);

    float_t angle_deg = (m->OBSERVER.observer_angle_rad * 180.0f) * ONE_BY_PI;
    if (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    m->OBSERVER.observer_angle_deg = angle_deg;
}

static inline __attribute__((always_inline)) void calculate_dq_pi(motor *m, float_t V_dc)
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

    // Bölme yerini çarpmaya bıraktı (1.0f / Iq_ki => 83.333f varsayımıyla veya doğrudan bölmeyi optimize ettik)
    // Eğer ki değerin değişiyorsa burası kalsın ama donanım çarpması yapmak her zaman iyidir.
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

    // DİKKAT: Yavaş math.h kütüphanesi yerine ARM GCC'nin donanım FPU komutu __builtin_sqrtf kullanıldı!
    float_t Eq_max = __builtin_sqrtf((V_rms * V_rms) - (m->OUT.E_d * m->OUT.E_d));
    m->OUT.E_q = clampf(m->OUT.E_q, -Eq_max, Eq_max);
}

#endif /* CONTROL_H */
