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

/**
 * @brief   3-Şöntlü (3-Shunt) Clarke ve Park dönüşümlerini uygular.
 *
 * @details İki fazdan üçüncü fazı tahmin etmek yerine, Ia, Ib ve Ic şöntlerinin
 *          tamamından okunan verileri matematiksel modele dahil ederek stator
 *          akımlarını dönen D-Q referans düzlemine aktarır.
 *
 * @note    **Avantajları:**
 *          - **Ortak Mod (Common-Mode) Reddi:** Üç sensörün verisini aynı formülde
 *            harmanlamak, op-amp'lardaki ısıl kaymaları (thermal drift) ve
 *            elektriksel gürültüleri 1/3 oranında kendi içinde sönümler.
 *          - **Yüksek Devirde Körlük Koruması:** PWM duty-cycle değerlerinin uç
 *            noktalara (%99 veya %1) ulaştığı yüksek devirlerde, ADC'nin okumakta
 *            zorlandığı fazı diğer iki sağlıklı faz dengeler. Tork salınımını önler.
 *
 * @param   m  Üzerinde işlem yapılacak motor yapısına işaretçi.
 */
static inline void clarke_park(motor* m)
{

	float_t I_alpha = (2.0f*m->STATUS.Ia_curr_map - m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map)/3.0f;
	float_t I_beta  = (m->STATUS.Ib_curr_map - m->STATUS.Ic_curr_map) * ONE_BY_SQRT3;

    m->STATUS.Id_curr =  (I_alpha * m->STATUS.foc_cos) + (I_beta * m->STATUS.foc_sin);
    m->STATUS.Iq_curr = -(I_alpha * m->STATUS.foc_sin) + (I_beta * m->STATUS.foc_cos);
}

/**
 * @brief   Ters Park ve ters Clarke dönüşümlerini uygular.
 *
 * @details D-Q eksenindeki hedef gerilim komutlarını (E_d, E_q), statik
 *          koordinat sistemindeki üç fazlı (Va, Vb, Vc) SVPWM/PWM komutlarına
 *          dönüştürür ve doğrudan motor çıkış yapısına (m->OUT) kaydeder.
 *
 * @param   m  Üzerinde işlem yapılacak motor yapısına işaretçi.
 */

static inline void inv_clarke_park(motor* m)
{

    float_t V_alpha = (m->OUT.E_d * m->STATUS.foc_cos) - (m->OUT.E_q * m->STATUS.foc_sin);
    float_t V_beta  = (m->OUT.E_d * m->STATUS.foc_sin) + (m->OUT.E_q * m->STATUS.foc_cos);

    m->OUT.Va = V_alpha;
    m->OUT.Vb = (-0.5f * V_alpha) + (SQRT3_BY_2 * V_beta);
    m->OUT.Vc = (-0.5f * V_alpha) - (SQRT3_BY_2 * V_beta);
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


#endif /* CONTROL_H */
