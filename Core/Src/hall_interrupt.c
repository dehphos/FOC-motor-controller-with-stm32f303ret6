#include "hall_interrupt.h"
#include "control.h"
#include "math.h"
#include "map.h"
#include "clampf.h"

extern motor MOTOR_1;


//__attribute__((section(".ccmram")))
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
    motor *m = NULL;

    if (htim->Instance == TIM3) {
        m = &MOTOR_1;
    }
    // ileride 2. motor gelirse: else if (htim->Instance == TIM4) { m = &MOTOR_2; }

    if (m == NULL) return;
    if (!m->STATUS.ALIGNED) return;

    if (htim->Instance == TIM3)
    {
    	uint32_t new_tim_raw = __HAL_TIM_GET_COMPARE(htim, m->OUT.A);
		if (new_tim_raw <= 0) new_tim_raw += 65536;
		static uint32_t period_accumulator = 0;
		period_accumulator += new_tim_raw;
		if (period_accumulator < 20) return;
		m->STATUS.period = period_accumulator;
		period_accumulator = 0;
		m->STATUS.last_hall_edge_tick = HAL_GetTick();
		m->STATUS.STOPPED = false;
		m->STATUS.hall_state = (m->IN.HALL.CHANNEL->IDR >> __builtin_ctz(m->IN.HALL.A)) & 0x07;


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

        // --- GERÇEK SENSÖR FÜZYONU (HYBRID ANCHORING) ---
        // 1. Açıyı kesin olarak gerçek Hall açısına hizala (Sıfır Hata)
        m->OBSERVER.theta_est = (float_t)m->STATUS.rotor_angle;

        m->STATUS.tim = m->STATUS.period;
        float_t inst_rpm = (float_t)m->OBSERVER.hall_direction * (10.0f * (float_t)TIM3_CNT_HZ) / ((float_t)m->STATUS.period * m->PARAMS.NUM_OF_POLE_PAIRS);

        // ---------------- BDF2 İVME SINIRLAYICI ----------------
        float_t dt = (float)m->STATUS.period / (float)TIM3_CNT_HZ;
        float_t bdf2_deriv = (3.0f * inst_rpm - 4.0f * m->OBSERVER.prev_rpm + m->OBSERVER.prev2_rpm) / (2.0f * dt);

        if (bdf2_deriv > m->PARAMS.MAX_RPM_ACCEL) {
            inst_rpm = (2.0f * m->PARAMS.MAX_RPM_ACCEL + 4.0f * m->OBSERVER.prev_rpm - m->OBSERVER.prev2_rpm) / 3.0f;}
        else if (bdf2_deriv < -m->PARAMS.MAX_RPM_ACCEL) {
            inst_rpm = (-2.0f * m->PARAMS.MAX_RPM_ACCEL + 4.0f * m->OBSERVER.prev_rpm - m->OBSERVER.prev2_rpm) / 3.0f;}

        m->OBSERVER.prev2_rpm = m->OBSERVER.prev_rpm;
        m->OBSERVER.prev_rpm = inst_rpm;

        // ---------------- İKİNCİ DERECE IIR FİLTRE ----------------

        float_t abs_inst = fabsf(inst_rpm);
        float_t alpha = clampf(map(abs_inst, 300.0f, 2000.0f, 0.1f, 0.7f), 0.1f, 0.7f);
        float_t beta  = 1.0f - alpha;

        m->OBSERVER.rpm_filter_stage1 = (m->OBSERVER.rpm_filter_stage1 * alpha) + (inst_rpm * beta);
        m->STATUS.rotor_rpm = (m->STATUS.rotor_rpm * alpha) + (m->OBSERVER.rpm_filter_stage1 * beta);
        m->STATUS.kama_rpm = m->STATUS.rotor_rpm / 4.5f;

        // 2. Filtrelenmiş ve pürüzsüz hızı modele çivile (RPM -> Deg/s)
        m->OBSERVER.omega_est = m->STATUS.rotor_rpm * 6.0f;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
}
