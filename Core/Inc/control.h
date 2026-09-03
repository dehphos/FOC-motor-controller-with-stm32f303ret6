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

/**
 * @brief  Clarke ve Park dönüşümlerini birlikte uygulayarak ölçülen faz
 *         akımlarını (Ia, Ib) döner referans çerçevesindeki d-q akımlarına
 *         dönüştürür.
 *
 * @param[in]  Ia        A fazı akımı.
 * @param[in]  Ib        B fazı akımı.
 * @param[in]  sin_theta Rotor elektriksel açısının sinüsü.
 * @param[in]  cos_theta Rotor elektriksel açısının kosinüsü.
 * @param[out] Id        Hesaplanan d-ekseni akımı buraya yazılır.
 * @param[out] Iq        Hesaplanan q-ekseni akımı buraya yazılır.
 */
static inline void clarke_park(float_t Ia, float_t Ib, float_t sin_theta, float_t cos_theta, float_t *Id, float_t *Iq)
{

    float_t I_alpha = Ia;
    float_t I_beta  = (Ia * ONE_BY_SQRT3) + (Ib * TWO_BY_SQRT3);

    *Id =  (I_alpha * cos_theta) + (I_beta * sin_theta);
    *Iq = -(I_alpha * sin_theta) + (I_beta * cos_theta);
}

/**
 * @brief  Ters Park ve ters Clarke dönüşümlerini birlikte uygulayarak d-q
 *         gerilim komutlarını (Vd, Vq) üç fazlı gerilim komutlarına
 *         (Va, Vb, Vc) dönüştürür.
 *
 * @param[in]  Vd        D-ekseni gerilim komutu.
 * @param[in]  Vq        Q-ekseni gerilim komutu.
 * @param[in]  sin_theta Rotor elektriksel açısının sinüsü.
 * @param[in]  cos_theta Rotor elektriksel açısının kosinüsü.
 * @param[out] Va        Hesaplanan A fazı gerilimi buraya yazılır.
 * @param[out] Vb        Hesaplanan B fazı gerilimi buraya yazılır.
 * @param[out] Vc        Hesaplanan C fazı gerilimi buraya yazılır.
 */
static inline void inv_clarke_park(float_t Vd, float_t Vq, float_t sin_theta, float_t cos_theta, float_t *Va, float_t *Vb, float_t *Vc)
{

    float_t V_alpha = (Vd * cos_theta) - (Vq * sin_theta);
    float_t V_beta  = (Vd * sin_theta) + (Vq * cos_theta);

    *Va = V_alpha;
    *Vb = (-0.5f * V_alpha) + (SQRT3_BY_2 * V_beta);
    *Vc = (-0.5f * V_alpha) - (SQRT3_BY_2 * V_beta);
}


/**
 * @brief  Hız referansını (`REF.RPM`) sabit adımlarla (`REF.STEP`) hedefe
 *         doğru rampalayarak `REF.RPM_cur` değerini günceller. Ayrıca bu
 *         adım/periyottan izin verilen maksimum ivmeyi (`MAX_RPM_ACCEL`)
 *         hesaplar.
 *
 * @param  MOTOR  Üzerinde işlem yapılacak motor yapısına işaretçi.
 */
static inline void ramp(motor *MOTOR) {

	float_t target_accel_rpm_s = (MOTOR->REF.STEP * 1000.0f) / (float_t)MOTOR->SPEED_PI_PARAMS.SPEED_LOOP_PERIOD_MS;
	MOTOR->PARAMS.MAX_RPM_ACCEL = target_accel_rpm_s * 5.0f;

    if (MOTOR->REF.RPM > MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur += MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur > MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
        }
    }
    else if (MOTOR->REF.RPM < MOTOR->REF.RPM_cur) {
        MOTOR->REF.RPM_cur -= MOTOR->REF.STEP;
        if (MOTOR->REF.RPM_cur < MOTOR->REF.RPM) {
            MOTOR->REF.RPM_cur = MOTOR->REF.RPM;
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


#endif /* CONTROL_H */
