/**
 * @file    control.h
 * @brief   FOC (Field Oriented Control) motor kontrolü için temel dönüşüm
 *          fonksiyonları (Clarke/Park ve tersleri), hız PI regülatörü,
 *          rampa ve PWM yazma yardımcıları.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include "math.h"

void calculate_speed_pi(motor *m);

void calculate_dq_pi(motor *m, float_t V_dc);




static inline void clarke(motor* m)
{
    m->STATUS.I_alpha = (2.0f * m->STATUS.Ia_curr_map - m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map) / 3.0f;
    m->STATUS.I_beta  = (m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map) * ONE_BY_SQRT3;
}

static inline void park(motor* m)
{
    m->STATUS.Id_curr =  (m->STATUS.I_alpha * m->STATUS.foc_cos) + (m->STATUS.I_beta * m->STATUS.foc_sin);
    m->STATUS.Iq_curr = -(m->STATUS.I_alpha * m->STATUS.foc_sin) + (m->STATUS.I_beta * m->STATUS.foc_cos);
}

static inline void inv_park(motor* m)
{
    m->OUT.V_alpha = (m->OUT.E_d * m->STATUS.foc_cos) - (m->OUT.E_q * m->STATUS.foc_sin);
    m->OUT.V_beta  = (m->OUT.E_d * m->STATUS.foc_sin) + (m->OUT.E_q * m->STATUS.foc_cos);
}

static inline void inv_clarke(motor* m)
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
static inline void ramp(motor *m) {

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
static inline void pwm_write(motor *m, float_t a, float_t b, float_t c){
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.A, a);
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.B, b);
	__HAL_TIM_SET_COMPARE(m->TIMER.PWM_TIMER, m->OUT.C, c);
}

void Align_Motor(motor *m);

void run_bemf_observer(motor *m);

#endif /* CONTROL_H */
