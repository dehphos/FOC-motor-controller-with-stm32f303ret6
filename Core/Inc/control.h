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
#include "clampf.h"
#include "map.h"



/**
 * @brief   Clarke Dönüşümü: Üç fazlı akımları (Ia, Ib, Ic) iki eksenli
 *          sabit (stationary) referans çerçeveye (I_alpha, I_beta) indirger.
 *
 * @param   m  Faz akımlarının okunduğu ve I_alpha/I_beta'nın yazılacağı
 *             motor yapısına işaretçi.
 */
FAST_INLINE void clarke(motor* m)
{
    // --- LOAD ---
    float_t Ia = m->STATUS.Ia_curr_map;
    float_t Ib = m->STATUS.Ib_curr_map;
    float_t Ic = m->STATUS.Ic_curr_map;

    // --- HESAPLA (bölme yerine çarpım) ---
    float_t I_alpha = (2.0f * Ia - Ib - Ic) * 0.3333333f;
    float_t I_beta  = (Ib - Ic) * ONE_BY_SQRT3;

    // --- STORE ---
    m->STATUS.I_alpha = I_alpha;
    m->STATUS.I_beta  = I_beta;
}

/**
 * @brief   Park Dönüşümü: Sabit (I_alpha, I_beta) çerçevesini, rotor açısına
 *          (foc_sin/foc_cos) göre dönen (Id, Iq) referans çerçevesine çevirir.
 *
 * @param   m  I_alpha/I_beta ve foc_sin/foc_cos'un okunduğu, Id_curr/Iq_curr'ın
 *             yazılacağı motor yapısına işaretçi.
 */
FAST_INLINE void park(motor* m)
{
    // --- LOAD ---
    float_t I_alpha = m->STATUS.I_alpha;
    float_t I_beta  = m->STATUS.I_beta;
    float_t c       = m->STATUS.foc_cos;
    float_t s       = m->STATUS.foc_sin;

    // --- HESAPLA ---
    float_t Id_curr =  (I_alpha * c) + (I_beta * s);
    float_t Iq_curr = -(I_alpha * s) + (I_beta * c);

    // --- STORE ---
    m->STATUS.Id_curr = Id_curr;
    m->STATUS.Iq_curr = Iq_curr;
}

/**
 * @brief   Ters Park Dönüşümü: PI çıkışı olan (E_d, E_q) gerilim vektörünü
 *          rotor açısına göre sabit (V_alpha, V_beta) çerçeveye geri çevirir.
 *
 * @param   m  E_d/E_q ve foc_sin/foc_cos'un okunduğu, V_alpha/V_beta'nın
 *             yazılacağı motor yapısına işaretçi.
 */
FAST_INLINE void inv_park(motor* m)
{
    // --- LOAD ---
    float_t Ed = m->OUT.E_d;
    float_t Eq = m->OUT.E_q;
    float_t c  = m->STATUS.foc_cos;
    float_t s  = m->STATUS.foc_sin;

    // --- HESAPLA ---
    float_t V_alpha = (Ed * c) - (Eq * s);
    float_t V_beta  = (Ed * s) + (Eq * c);

    // --- STORE ---
    m->OUT.V_alpha = V_alpha;
    m->OUT.V_beta  = V_beta;
}

/**
 * @brief   Ters Clarke Dönüşümü: Sabit (V_alpha, V_beta) gerilim vektörünü
 *          üç fazlı (Va, Vb, Vc) gerilim komutlarına dağıtır.
 *
 * @param   m  V_alpha/V_beta'nın okunduğu, Va/Vb/Vc'nin yazılacağı motor
 *             yapısına işaretçi.
 */
FAST_INLINE void inv_clarke(motor* m)
{
    // --- LOAD ---
    float_t V_alpha = m->OUT.V_alpha;
    float_t V_beta  = m->OUT.V_beta;

    // --- HESAPLA ---
    float_t Va = V_alpha;
    float_t Vb = (-0.5f * V_alpha) + (SQRT3_BY_2 * V_beta);
    float_t Vc = (-0.5f * V_alpha) - (SQRT3_BY_2 * V_beta);

    // --- STORE ---
    m->OUT.Va = Va;
    m->OUT.Vb = Vb;
    m->OUT.Vc = Vc;
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

    // --- LOAD ---
    float_t step      = m->REF.STEP;
    uint16_t period_ms = m->PARAMS.SPEED_PI.SPEED_LOOP_PERIOD_MS;
    float_t ref_rpm   = m->REF.RPM;
    float_t rpm_cur   = m->REF.RPM_cur;

    // --- HESAPLA ---
    float_t target_accel_rpm_s = (step * 1000.0f) / (float_t)period_ms;
    float_t max_accel = target_accel_rpm_s * 5.0f;

    if (ref_rpm > rpm_cur) {
        rpm_cur += step;
        if (rpm_cur > ref_rpm) {
            rpm_cur = ref_rpm;
        }
    }
    else if (ref_rpm < rpm_cur) {
        rpm_cur -= step;
        if (rpm_cur < ref_rpm) {
            rpm_cur = ref_rpm;
        }
    }

    // --- STORE ---
    m->PARAMS.MAX_RPM_ACCEL = max_accel;
    m->REF.RPM_cur = rpm_cur;
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
    	a = (abs_x == 1.0f) ? abs_y : (abs_y / abs_x);
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

void calculate_speed_pi(motor *m);

void calculate_dq_pi(motor *m, float_t V_dc);

void run_bemf_observer(motor *m);
#endif /* CONTROL_H */
