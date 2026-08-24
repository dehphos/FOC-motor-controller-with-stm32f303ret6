#include "hall_interrupt.h"
#include "control.h"
#include "math.h"
#include "map.h"
#include "clampf.h"

extern motor MOTOR_1;

// ----------------------------------------------------------------------
// HALL SENSÖR KESMESİ (HIZ VE AÇI HESABI)
// ----------------------------------------------------------------------
//__attribute__((section(".ccmram")))
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // DİNAMİK MOTOR SEÇİMİ (MODÜLER YAPI)
    motor *m = NULL;

    if (htim->Instance == TIM3) {
        m = &MOTOR_1;
    }
    // İleride 2. motor gelirse: else if (htim->Instance == TIM4) { m = &MOTOR_2; }

    if (m == NULL) return;
    if (!m->STATUS.ALIGNED) return;

    if (htim->Instance == TIM3)
    {
        uint32_t new_tim_raw = __HAL_TIM_GET_COMPARE(htim, m->OUT.A);
        m->STATUS.period = new_tim_raw;
        if (m->STATUS.period <= 0) m->STATUS.period += 65536;
        if (m->STATUS.period < 20) return;

        m->STATUS.last_hall_edge_tick = HAL_GetTick();
        m->STATUS.STOPPED = false;
        m->STATUS.hall_state = (m->IN.HAL.CHANNEL->IDR >> 6) & 0x07;

        static uint8_t prev_hall = 0;
        static int8_t hall_direction = 1;

        if (prev_hall != 0 && prev_hall != m->STATUS.hall_state) {
            if ((prev_hall == 1 && m->STATUS.hall_state == 3) || (prev_hall == 3 && m->STATUS.hall_state == 2) ||
                (prev_hall == 2 && m->STATUS.hall_state == 6) || (prev_hall == 6 && m->STATUS.hall_state == 4) ||
                (prev_hall == 4 && m->STATUS.hall_state == 5) || (prev_hall == 5 && m->STATUS.hall_state == 1)) {
                hall_direction = 1;
            } else if ((prev_hall == 1 && m->STATUS.hall_state == 5) || (prev_hall == 5 && m->STATUS.hall_state == 4) ||
                    (prev_hall == 4 && m->STATUS.hall_state == 6) || (prev_hall == 6 && m->STATUS.hall_state == 2) ||
                    (prev_hall == 2 && m->STATUS.hall_state == 3) || (prev_hall == 3 && m->STATUS.hall_state == 1)) {
                hall_direction = -1;
            }
        }
        prev_hall = m->STATUS.hall_state;

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

        m->STATUS.tim = m->STATUS.period;

        // --- 6'LI ORTALAMA SİLİNDİ (Osilasyonları kesmek için) ---
        // 1. Ham RPM Hesabı (Doğrudan o anki anlık periyot ile)
        float_t inst_rpm = (float_t)hall_direction * (10.0f * (float_t)TIM3_CNT_HZ) / ((float_t)m->STATUS.period * m->PARAMS.NUM_OF_POLE_PAIRS);

        // ---------------- BDF2 İVME SINIRLAYICI ----------------
        static float_t prev_inst_rpm = 0.0f;
        static float_t prev2_inst_rpm = 0.0f;
        float_t bdf2_deriv = (3.0f * inst_rpm - 4.0f * prev_inst_rpm + prev2_inst_rpm) / 2.0f;

        if (bdf2_deriv > m->PARAMS.MAX_RPM_CHANGE) {
            inst_rpm = (2.0f * m->PARAMS.MAX_RPM_CHANGE + 4.0f * prev_inst_rpm - prev2_inst_rpm) / 3.0f;
        }
        else if (bdf2_deriv < -m->PARAMS.MAX_RPM_CHANGE) {
            inst_rpm = (-2.0f * m->PARAMS.MAX_RPM_CHANGE + 4.0f * prev_inst_rpm - prev2_inst_rpm) / 3.0f;
        }

        prev2_inst_rpm = prev_inst_rpm;
        prev_inst_rpm = inst_rpm;

        // ---------------- İKİNCİ DERECE IIR FİLTRE ----------------
        static float_t rpm_filter_stage1 = 0.0f;
        float_t abs_inst = fabsf(inst_rpm);
        float_t alpha = clampf(map(abs_inst, 300.0f, 2000.0f, 0.1f, 0.7f), 0.1f, 0.7f);
        float_t beta  = 1.0f - alpha;

        rpm_filter_stage1 = (rpm_filter_stage1 * alpha) + (inst_rpm * beta);
        m->STATUS.rotor_rpm = (m->STATUS.rotor_rpm * alpha) + (rpm_filter_stage1 * beta);
        m->STATUS.kama_rpm = m->STATUS.rotor_rpm / 4.5f;
    }
}
